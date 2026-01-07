"""
状态基类

定义状态的生命周期方法和通用接口
参考：engineai_humanoid/FSM_States/FSM_State.h
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple, TYPE_CHECKING, List
import logging
import math

from .enums import WeddingStateName, WeddingEvent
from ..perception import FaceFollowingHelper

if TYPE_CHECKING:
    from .wedding_fsm import WeddingFSM
    from ..perception import FaceInfo
    from ..action_pack import ActionPack, ActionLibrary


@dataclass
class TransitionData:
    """状态转换数据"""
    done: bool = False              # 转换是否完成
    next_state: Optional[WeddingStateName] = None  # 下一个状态
    duration: float = 0.0           # 转换持续时间
    
    def reset(self) -> None:
        """重置转换数据"""
        self.done = False
        self.next_state = None
        self.duration = 0.0


class WeddingState(ABC):
    """
    婚礼互动状态基类
    
    生命周期方法：
    - on_enter(): 进入状态时调用
    - run(): 每个控制周期调用
    - check_transition(): 检查是否需要状态转换
    - transition(): 执行状态转换逻辑
    - on_exit(): 退出状态时调用
    
    参考：engineai_humanoid/FSM_States/FSM_State.h
    """
    
    def __init__(self, fsm: 'WeddingFSM', state_name: WeddingStateName):
        """
        初始化状态
        
        Args:
            fsm: FSM 控制器引用，用于访问共享数据
            state_name: 状态名称枚举
        """
        self.fsm = fsm
        self.state_name = state_name
        self.state_string = str(state_name)
        
        # 日志
        # 优先使用 ROS2 logger（如果 FSM 提供了），否则使用 Python logging
        if hasattr(fsm, '_ros2_logger') and fsm._ros2_logger is not None:
            # 使用 ROS2 logger
            # ROS2 logger 可以直接使用，我们使用同一个 logger 但添加状态名称前缀
            self.logger = fsm._ros2_logger
            self._state_log_prefix = f"[State.{self.state_string}]"
            self._use_ros2_logger = True
        else:
            # 使用 Python logging
            self.logger = logging.getLogger(f"State.{self.state_string}")
            self._state_log_prefix = ""
            self._use_ros2_logger = False
        
        # 状态转换相关
        self.next_state_name: WeddingStateName = state_name
        self.transition_data = TransitionData()
        
        # 状态内部计数器
        self.iter_count = 0
        self.enter_time: float = 0.0
        
        # 安全检查标志
        self.check_safe_distance = True
        
        # ========== 跟随相关公共属性 ==========
        # 跟踪目标信息（用于匹配特定对象，SEARCH 和 TRACKING 使用）
        self._tracked_face_id: Optional[int] = None
        self._tracked_position: Optional[Tuple[float, float]] = None
        self._match_distance_threshold: float = 0.2  # 位置匹配距离阈值（归一化坐标）
        
        # ========== 动作包管理（新增） ==========
        self._action_library: Optional['ActionLibrary'] = None
        self._current_pack: Optional['ActionPack'] = None
        self._pack_start_time: float = 0.0
        self._speech_played: bool = False
        # 从配置读取开关（默认开启）
        self._enable_action: bool = self.fsm.data.config.get('enable_action', True)
        self._enable_speech: bool = self.fsm.data.config.get('enable_speech', True)
        
        # ========== 人脸检测配置（子类可覆盖） ==========
        self.VALID_REGION: Tuple[float, float, float, float] = (0.3, 0.7, 0.1, 0.9)  # (x_min, x_max, y_min, y_max)
        self.FACE_MIN_AREA: float = 0.01
        self.FACE_MAX_AREA: float = 0.30
        self.CENTER_WEIGHT: float = 0.6  # 中心位置权重（vs 面积）
        self.MATCH_DISTANCE_THRESHOLD: float = 0.15  # 跨帧匹配距离阈值
        
        # ========== 目标丢失检测（可选，子类启用） ==========
        self._target_lost_time: float = 0.0
        self._target_lost_timeout: Optional[float] = None
        self._state_timeout: Optional[float] = None
        
    @abstractmethod
    def on_enter(self) -> None:
        """
        进入状态时的初始化逻辑
        - 重置状态内部变量
        - 设置初始动作/语音
        - 记录进入时间
        """
        pass
    
    @abstractmethod
    def run(self) -> None:
        """
        状态的主循环逻辑
        - 每个控制周期调用一次
        - 执行状态特定行为（跟随、Pose 等）
        """
        pass
    
    def check_transition(self) -> WeddingStateName:
        """
        检查是否需要状态转换
        
        Returns:
            下一个状态名称（如果不需要转换，返回当前状态名称）
        
        默认实现：检查外部命令和安全事件
        子类可以覆盖此方法添加状态特定的转换逻辑
        """
        # 检查安全事件（最高优先级）
        if self.fsm.data.safety_triggered:
            return WeddingStateName.SAFE_STOP
        
        # 检查外部命令
        cmd = self.fsm.data.pending_command
        if cmd == WeddingEvent.CMD_STOP:
            return WeddingStateName.IDLE
        elif cmd == WeddingEvent.CMD_GOTO_IDLE:
            return WeddingStateName.IDLE
        elif cmd == WeddingEvent.CMD_GOTO_PHOTO:
            return WeddingStateName.PHOTO_POSING
        elif cmd == WeddingEvent.CMD_GOTO_TRACKING:
            return WeddingStateName.TRACKING
        
        # 默认：保持当前状态
        return self.state_name
    
    def transition(self) -> TransitionData:
        """
        执行状态转换逻辑
        
        Returns:
            TransitionData 包含转换是否完成等信息
        
        默认实现：立即完成转换
        子类可以覆盖此方法实现渐进式转换（如平滑过渡动作）
        """
        self.transition_data.done = True
        return self.transition_data
    
    @abstractmethod
    def on_exit(self) -> None:
        """
        退出状态时的清理逻辑
        - 停止状态特定行为
        - 重置状态变量
        """
        pass
    
    # ========== 辅助方法 ==========
    
    def get_elapsed_time(self) -> float:
        """获取在当前状态的持续时间（秒）"""
        return self.fsm.get_current_time() - self.enter_time
    
    def get_current_time(self) -> float:
        """获取当前时间（统一接口）"""
        return self.fsm.get_current_time()
    
    def log(self, message: str) -> None:
        """状态日志输出"""
        # 方法1: 使用 ROS2 logger
        if self._use_ros2_logger:
            # 使用 ROS2 logger，添加状态名称前缀
            self.logger.info(f"{self._state_log_prefix} {message}")
        else:
            # 使用 Python logging
            self.logger.info(message)
        
        # 方法2: 发布到 ROS2 topic（如果可用）
        if hasattr(self.fsm, '_debug_pub') and self.fsm._debug_pub is not None:
            try:
                from std_msgs.msg import String
                msg = String()
                msg.data = f"[{self.state_string}] {message}"
                self.fsm._debug_pub.publish(msg)
            except:
                pass
        
        # 方法3: 写入文件（如果启用）
        if hasattr(self.fsm, '_debug_log_file') and self.fsm._debug_log_file is not None:
            try:
                import time
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                log_line = f"{timestamp} [{self.state_string}] {message}\n"
                self.fsm._debug_log_file.write(log_line)
                self.fsm._debug_log_file.flush()  # 立即刷新到文件
            except:
                pass
    
    def log_debug(self, message: str) -> None:
        """状态调试日志"""
        if self._use_ros2_logger:
            # 使用 ROS2 logger，添加状态名称前缀
            self.logger.debug(f"{self._state_log_prefix} {message}")
        else:
            # 使用 Python logging
            self.logger.debug(message)
    
    def set_pose(self, pose_name: str) -> None:
        """设置目标 Pose"""
        self.fsm.data.motion.target_pose = pose_name
        self.log_debug(f"Set pose: {pose_name}")
    
    def set_speech(self, speech_id: str) -> None:
        """设置待播放语音"""
        self.fsm.data.audio.pending_speech = speech_id
        self.log_debug(f"Set speech: {speech_id}")
    
    def set_look_at(self, x: float, y: float) -> None:
        """设置注视目标"""
        self.fsm.data.motion.look_at_target[0] = x
        self.fsm.data.motion.look_at_target[1] = y
    
    # ========== 跟随相关公共方法 ==========
    
    def pid_follow_target_face(self, target_face: 'FaceInfo', log_prefix: str = "") -> None:
        """
        基于 PID 控制的跟随（基于角度误差）
        
        核心思想：
        1. 从图像像素偏差计算角度：θ = atan((x_image - cx) / fx)
        2. 使用角度误差进行PID控制
        3. 将角度变化转换为look_at变化
        
        改进：添加数据有效性检查
        - 检查数据时间戳，避免使用过期数据
        - 根据实际FSM频率计算控制周期
        
        Args:
            target_face: 目标人脸信息
            log_prefix: 日志前缀（用于区分不同状态，如 "[SEARCH跟随]"）
        """
        import time
        from ..perception import FaceFollowingHelper
        
        # 检查相机参数是否有效
        camera = self.fsm.data.camera
        if not camera.is_valid():
            self.log_debug(f"{log_prefix} Camera info not available, check error！！！！！！！！！！！")
            return
        
        # ========== 数据有效性检查 ==========
        current_time = time.time()
        data_age = current_time - target_face.timestamp
        
        # 数据过期阈值（秒）：如果数据超过此时间，认为过期
        MAX_DATA_AGE = 0.1  # 100ms，约3帧@30Hz
        
        if data_age > MAX_DATA_AGE:
            self.log_debug(f"{log_prefix} [数据过期] 数据年龄 {data_age*1000:.1f}ms > {MAX_DATA_AGE*1000:.0f}ms，跳过本次控制")
            return
        
        # 获取目标位置（直接使用检测位置，不进行延时补偿）
        target_x = target_face.center_x
        target_y = target_face.center_y
        
        # 当前 look_at 位置
        current_x = self.fsm.data.motion.look_at_target[0]
        current_y = self.fsm.data.motion.look_at_target[1]
        
        # 获取 PID 状态（需要外部维护）
        if not hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
            self._last_pid_time = current_time
        
        # 计算实际控制周期（基于上次PID执行时间）
        if not hasattr(self, '_last_pid_time'):
            self._last_pid_time = current_time
        
        actual_dt = current_time - self._last_pid_time
        # 限制dt在合理范围内（避免异常值）
        actual_dt = max(0.01, min(0.1, actual_dt))  # 10ms ~ 100ms
        self._last_pid_time = current_time
        
        # 如果dt异常大，说明控制周期不稳定，使用默认值
        if actual_dt > 0.1:
            dt = 0.04  # 25Hz的周期
            self.log_debug(f"{log_prefix} [控制周期异常] actual_dt={actual_dt*1000:.1f}ms，使用默认dt={dt*1000:.0f}ms")
        else:
            dt = actual_dt
        
        # 调用 PID 控制（传入相机参数、logger 和 log_prefix）
        new_x, new_y, new_integral_x, new_integral_y = FaceFollowingHelper.pid_follow(
            target_x, target_y,
            current_x, current_y,
            camera.fx, camera.fy,
            camera.width, camera.height,
            self._pid_integral_x, self._pid_integral_y,
            self._pid_last_error_x, self._pid_last_error_y,
            dt,
            logger=self.logger,
            log_prefix=log_prefix
        )
        
        # 更新 PID 状态
        self._pid_integral_x = new_integral_x
        self._pid_integral_y = new_integral_y
        
        # 计算角度误差（用于下次的微分项）
        normalized_offset_x = target_x - 0.5
        normalized_offset_y = target_y - 0.5
        pixel_offset_x = FaceFollowingHelper.normalized_to_pixel_offset(normalized_offset_x, camera.width)
        pixel_offset_y = FaceFollowingHelper.normalized_to_pixel_offset(normalized_offset_y, camera.height)
        angle_error_x = FaceFollowingHelper.pixel_to_angle(pixel_offset_x, camera.fx)
        angle_error_y = FaceFollowingHelper.pixel_to_angle(pixel_offset_y, camera.fy)
        self._pid_last_error_x = angle_error_x
        self._pid_last_error_y = angle_error_y
        
        # ========== 状态平滑（可选）==========
        smoothed_x = new_x
        smoothed_y = new_y
        
        self._smoothed_look_at_x = smoothed_x
        self._smoothed_look_at_y = smoothed_y
        
        # 设置平滑后的 look_at
        self.set_look_at(smoothed_x, smoothed_y)
        
    

    
    def get_target_face_position(self, target_face: 'FaceInfo') -> Tuple[float, float]:
        """
        获取目标人脸的位置
        
        统一的方法，从 FaceInfo 对象获取中心点坐标
        
        Args:
            target_face: 目标人脸对象
        
        Returns:
            目标位置 (x, y) 归一化坐标
        """
        return (target_face.center_x, target_face.center_y)
    
    def find_tracked_face_by_id_and_position(self, faces: list, 
                                            tracked_face_id: Optional[int],
                                            tracked_position: Optional[Tuple[float, float]],
                                            match_distance_threshold: float = 0.2) -> Optional['FaceInfo']:
        """
        通过 face_id 或位置匹配查找跟踪的目标脸
        
        统一的匹配方法，供 SEARCH 和 TRACKING 状态使用
        
        Args:
            faces: 当前帧检测到的人脸列表
            tracked_face_id: 跟踪的 face_id
            tracked_position: 跟踪的位置
            match_distance_threshold: 位置匹配距离阈值（归一化坐标）
        
        Returns:
            匹配的目标人脸（可以是侧脸），或 None
        """
        if not faces:
            return None
        
        # 方法 1: 通过 face_id 匹配（如果 face_id 稳定）
        if tracked_face_id is not None and tracked_face_id >= 0:
            for face in faces:
                if face.face_id == tracked_face_id:
                    return face
        
        # 方法 2: 通过位置匹配（如果 face_id 不稳定或未设置）
        if tracked_position is not None:
            best_match = None
            best_distance = match_distance_threshold
            
            for face in faces:
                # 计算位置距离（不要求是正脸）
                distance = ((face.center_x - tracked_position[0]) ** 2 +
                           (face.center_y - tracked_position[1]) ** 2) ** 0.5
                if distance < best_distance:
                    best_distance = distance
                    best_match = face
            
            if best_match is not None:
                return best_match
        
        return None
    
    
    # ========== 动作包管理（新增） ==========
    
    def _init_action_library(self) -> None:
        """初始化动作库（延迟加载）"""
        if self._action_library is None:
            from ..action_pack import get_action_library
            self._action_library = get_action_library()
    
    def load_action_pack(self, pack_name: str) -> bool:
        """
        加载动作包（通用方法）
        
        Args:
            pack_name: 动作包名称
        
        Returns:
            True 如果加载成功，False 如果动作包不存在
        """
        self._init_action_library()
        if self._action_library is None:
            return False
        
        pack = self._action_library.get(pack_name)
        if pack:
            self._current_pack = pack
            self._pack_start_time = self.get_current_time()
            self._speech_played = False
            self.log_debug(f"Loaded action pack: {pack_name}")
            return True
        else:
            self._current_pack = None
            return False
    
    def execute_action_pack(self) -> None:
        """
        执行当前动作包（通用方法）
        
        如果动作禁用或没有动作包，使用默认摆动
        """
        # 如果动作禁用，使用默认摆动
        if not self._enable_action:
            self.default_sway()
            return
        
        if self._current_pack is None:
            self.default_sway()
            return
        
        current_time = self.get_current_time()
        pack_elapsed = current_time - self._pack_start_time
        
        # 获取头部偏移
        head_offset = self._current_pack.get_head_offset(pack_elapsed)
        
        # 转换为归一化坐标
        max_offset = 0.5
        normalized_x = 0.5 + (head_offset / max_offset) * 0.5
        normalized_x = max(0.0, min(1.0, normalized_x))
        
        self.set_look_at(normalized_x, 0.5)
        
        # 播放语音（如果语音启用）
        if self._enable_speech and self._current_pack.should_play_speech(pack_elapsed, self._speech_played):
            speech = self._current_pack.get_speech()
            if speech:
                self.set_speech(speech)
                self._speech_played = True
    
    def default_sway(self, amplitude: float = 0.2, period: float = 2.5) -> None:
        """
        默认待机摆动（通用方法）
        
        Args:
            amplitude: 摆动幅度（归一化坐标，默认 0.2）
            period: 摆动周期（秒，默认 2.5）
        """
        elapsed = self.get_elapsed_time()
        sway_offset = amplitude * math.sin(2 * math.pi * elapsed / period)
        self.set_look_at(0.5 + sway_offset, 0.5)
    
    def switch_action_pack(self, pack_name: str) -> bool:
        """
        切换动作包（外部接口）
        
        Args:
            pack_name: 动作包名称
        
        Returns:
            True 如果切换成功，False 如果动作包不存在
        """
        return self.load_action_pack(pack_name)
    
    # ========== 人脸检测辅助（新增） ==========
    
    def filter_valid_faces(self, faces: List['FaceInfo'], require_frontal: bool = True) -> List['FaceInfo']:
        """
        过滤有效区域内的人脸
        
        Args:
            faces: 人脸列表
            require_frontal: 是否要求正脸
        
        Returns:
            过滤后的人脸列表
        """
        x_min, x_max, y_min, y_max = self.VALID_REGION
        
        valid_faces = []
        for face in faces:
            if require_frontal and not face.is_frontal:
                continue
            if not face.is_in_region(x_min, x_max, y_min, y_max):
                continue
            if not (self.FACE_MIN_AREA <= face.area <= self.FACE_MAX_AREA):
                continue
            valid_faces.append(face)
        
        return valid_faces
    
    def select_best_face(self, faces: List['FaceInfo'], center_x: float = 0.5, 
                        center_y: float = 0.5) -> Optional['FaceInfo']:
        """
        选择最佳正脸（考虑位置和面积）
        
        评分 = 面积分数 * (1 - CENTER_WEIGHT) + 中心分数 * CENTER_WEIGHT
        
        Args:
            faces: 人脸列表
            center_x: 中心 x 坐标（归一化，默认 0.5）
            center_y: 中心 y 坐标（归一化，默认 0.5）
        
        Returns:
            最佳正脸，或 None
        """
        # 过滤：正脸 + 有效区域
        valid_faces = self.filter_valid_faces(faces, require_frontal=True)
        
        if not valid_faces:
            return None
        
        # 计算评分
        def score(face: 'FaceInfo') -> float:
            # 面积分数（归一化到 0-1）
            area_score = min(1.0, face.area / 0.1)  # 0.1 为"大脸"阈值
            
            # 中心分数（距离中心越近越高）
            distance_to_center_x = abs(face.center_x - center_x)
            distance_to_center_y = abs(face.center_y - center_y)
            distance_to_center = distance_to_center_x + distance_to_center_y
            center_score = max(0.0, 1.0 - distance_to_center * 2)  # 0.5 距离 = 0 分
            
            return area_score * (1 - self.CENTER_WEIGHT) + center_score * self.CENTER_WEIGHT

        best_face = max(valid_faces, key=score)        
        return best_face
    
    def find_matching_face(self, faces: List['FaceInfo'], target: 'FaceInfo', 
                          threshold: Optional[float] = None) -> Optional['FaceInfo']:
        """
        在当前帧中查找与目标匹配的人脸
        
        优先使用持久化 face_id 匹配（如果启用追踪），否则使用位置距离匹配
        
        Args:
            faces: 当前帧的人脸列表
            target: 目标人脸
            threshold: 匹配距离阈值（归一化坐标），如果为 None 则使用类常量
        
        Returns:
            匹配的人脸，或 None
        """
        if not faces:
            return None
        
        # 优先使用持久化 ID 匹配（如果目标有有效 ID）
        if target.face_id >= 0:
            for face in faces:
                if face.face_id == target.face_id:
                    # ID 匹配成功，直接返回（可以是侧脸，但这里只匹配正脸）
                    if face.is_frontal:
                        return face
        
        # 如果没有 ID 匹配，使用位置距离匹配（向后兼容）
        if threshold is None:
            threshold = self.MATCH_DISTANCE_THRESHOLD
        
        target_pos = target.center
        
        best_match = None
        best_distance = threshold
        
        for face in faces:
            if not face.is_frontal:
                continue
            
            distance = ((face.center_x - target_pos[0]) ** 2 + 
                       (face.center_y - target_pos[1]) ** 2) ** 0.5
            
            if distance < best_distance:
                best_distance = distance
                best_match = face
        
        return best_match
    
    def is_face_in_region(self, face: 'FaceInfo', region: Optional[Tuple[float, float, float, float]] = None) -> bool:
        """
        检查人脸是否在有效区域内
        
        Args:
            face: 人脸对象
            region: 区域 (x_min, x_max, y_min, y_max)，如果为 None 则使用类常量
        
        Returns:
            True 如果人脸在有效区域内
        """
        if region is None:
            region = self.VALID_REGION
        
        x_min, x_max, y_min, y_max = region
        return face.is_in_region(x_min, x_max, y_min, y_max)
    
    # ========== 目标丢失检测（新增） ==========
    
    def enable_target_lost_detection(self, timeout: float) -> None:
        """
        启用目标丢失检测
        
        Args:
            timeout: 目标丢失超时时间（秒）
        """
        self._target_lost_timeout = timeout
        self._target_lost_time = 0.0
    
    def enable_state_timeout(self, timeout: float) -> None:
        """
        启用状态持续时间超时
        
        Args:
            timeout: 状态最大持续时间（秒）
        """
        self._state_timeout = timeout
    
    def update_target_lost_time(self, has_target: bool, dt: float = 0.02) -> None:
        """
        更新目标丢失时间
        
        Args:
            has_target: 当前是否有目标
            dt: 时间间隔（秒），默认 0.02（50Hz）
        """
        if has_target:
            self._target_lost_time = 0.0
        else:
            self._target_lost_time += dt
    
    def is_target_lost_timeout(self) -> bool:
        """
        检查目标是否丢失超时
        
        Returns:
            True 如果目标丢失超时
        """
        if self._target_lost_timeout is None:
            return False
        return self._target_lost_time > self._target_lost_timeout
    
    def is_state_timeout(self) -> bool:
        """
        检查状态是否超时
        
        Returns:
            True 如果状态持续时间超时
        """
        if self._state_timeout is None:
            return False
        return self.get_elapsed_time() > self._state_timeout
    
    def get_default_smooth_coefficient(self) -> float:
        """
        获取状态默认平滑系数（子类可覆盖）
        
        Returns:
            默认平滑系数
        """
        return FaceFollowingHelper.get_state_smooth(self.state_string)

