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
    """
    
    def __init__(self, fsm: 'WeddingFSM', state_name: WeddingStateName):
        self.fsm = fsm
        self.state_name = state_name
        self.state_string = str(state_name)
        
        # 日志配置
        if hasattr(fsm, '_ros2_logger') and fsm._ros2_logger is not None:
            self.logger = fsm._ros2_logger
            self._state_log_prefix = f"[State.{self.state_string}]"
            self._use_ros2_logger = True
        else:
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
        self._tracked_face_id: Optional[int] = None
        self._tracked_position: Optional[Tuple[float, float]] = None
        self._match_distance_threshold: float = 0.2
        
        # ========== 人脸检测配置 ==========
        self.VALID_REGION: Tuple[float, float, float, float] = (0.3, 0.7, 0.1, 0.9)
        self.FACE_MIN_AREA: float = 0.01
        self.FACE_MAX_AREA: float = 0.30
        self.CENTER_WEIGHT: float = 0.6
        self.MATCH_DISTANCE_THRESHOLD: float = 0.15
        
        # ========== 目标丢失检测 ==========
        self._target_lost_time: float = 0.0
        self._target_lost_timeout: Optional[float] = None
        self._state_timeout: Optional[float] = None
        
        # ========== 全局配置缓存 ==========
        self._enable_speech = True
        if self.fsm and self.fsm.data.config:
             self._enable_speech = self.fsm.data.config.get('enable_speech', True)
        
    @abstractmethod
    def on_enter(self) -> None:
        pass
    
    @abstractmethod
    def run(self) -> None:
        pass
    
    def check_transition(self) -> WeddingStateName:
        # 检查安全事件
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
        
        return self.state_name
    
    def transition(self) -> TransitionData:
        self.transition_data.done = True
        return self.transition_data
    
    @abstractmethod
    def on_exit(self) -> None:
        pass
    
    # ========== 辅助方法 ==========
    
    def get_elapsed_time(self) -> float:
        return self.fsm.get_current_time() - self.enter_time
    
    def get_current_time(self) -> float:
        return self.fsm.get_current_time()
    
    def log(self, message: str) -> None:
        if self._use_ros2_logger:
            self.logger.info(f"{self._state_log_prefix} {message}")
        else:
            self.logger.info(message)
        
        if hasattr(self.fsm, '_debug_pub') and self.fsm._debug_pub is not None:
            try:
                from std_msgs.msg import String
                msg = String()
                msg.data = f"[{self.state_string}] {message}"
                self.fsm._debug_pub.publish(msg)
            except: pass
    
    def log_debug(self, message: str) -> None:
        if self._use_ros2_logger:
            self.logger.debug(f"{self._state_log_prefix} {message}")
        else:
            self.logger.debug(message)
    
    # ========== 动作与策略执行 (新版) ==========

    @property
    def current_motion_strategy(self) -> Optional[str]:
        if hasattr(self, '_current_motion_strategy'):
            return self._current_motion_strategy
        return None

    def execute_action(self, action_name: str) -> None:
        """执行综合动作包（委托给 ActionManager）"""
        if self.fsm.action_manager:
            enable_speech = self._enable_speech # 使用类成员变量 (已在init中从config读取)
            # 或者重新从 config 读取以支持动态修改
            if self.fsm.data.config:
                 enable_speech = self.fsm.data.config.get('enable_speech', True)
            
            strategy = self.fsm.action_manager.execute_action(action_name, enable_speech=enable_speech)
            self._current_motion_strategy = strategy
            self.log_debug(f"Executed action: {action_name}, strategy: {strategy}")
        else:
            self.logger.error("ActionManager not available!")

    def stop_action(self) -> None:
        """停止当前动作"""
        if self.fsm.action_manager:
            self.fsm.action_manager.stop_motion()
            self._current_motion_strategy = None
            self.log_debug("Action stopped")

    def set_pose(self, pose_name: str) -> None:
        """设置目标 Pose（保留以兼容旧代码，但直接更新fsm数据）"""
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
        """基于 PID 控制的跟随"""
        import time
        from ..perception import FaceFollowingHelper
        
        camera = self.fsm.data.camera
        if not camera.is_valid():
            return
        
        current_time = time.time()
        data_age = current_time - target_face.timestamp
        MAX_DATA_AGE = 0.1
        
        if data_age > MAX_DATA_AGE:
            return
        
        target_x = target_face.center_x
        target_y = target_face.center_y
        
        current_x = self.fsm.data.motion.look_at_target[0]
        current_y = self.fsm.data.motion.look_at_target[1]
        
        if not hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
            self._last_pid_time = current_time
        
        if not hasattr(self, '_last_pid_time'):
            self._last_pid_time = current_time
        
        actual_dt = current_time - self._last_pid_time
        actual_dt = max(0.01, min(0.1, actual_dt))
        self._last_pid_time = current_time
        
        dt = 0.04 if actual_dt > 0.1 else actual_dt
        
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
        
        self._pid_integral_x = new_integral_x
        self._pid_integral_y = new_integral_y
        
        normalized_offset_x = target_x - 0.5
        normalized_offset_y = target_y - 0.5
        pixel_offset_x = FaceFollowingHelper.normalized_to_pixel_offset(normalized_offset_x, camera.width)
        pixel_offset_y = FaceFollowingHelper.normalized_to_pixel_offset(normalized_offset_y, camera.height)
        angle_error_x = FaceFollowingHelper.pixel_to_angle(pixel_offset_x, camera.fx)
        angle_error_y = FaceFollowingHelper.pixel_to_angle(pixel_offset_y, camera.fy)
        self._pid_last_error_x = angle_error_x
        self._pid_last_error_y = angle_error_y
        
        self._smoothed_look_at_x = new_x
        self._smoothed_look_at_y = new_y
        
        self.set_look_at(new_x, new_y)
    
    def get_target_face_position(self, target_face: 'FaceInfo') -> Tuple[float, float]:
        return (target_face.center_x, target_face.center_y)
    
    def find_tracked_face_by_id_and_position(self, faces: list, 
                                            tracked_face_id: Optional[int],
                                            tracked_position: Optional[Tuple[float, float]],
                                            match_distance_threshold: float = 0.2) -> Optional['FaceInfo']:
        if not faces:
            return None
        
        if tracked_face_id is not None and tracked_face_id >= 0:
            for face in faces:
                if face.face_id == tracked_face_id:
                    return face
        
        if tracked_position is not None:
            best_match = None
            best_distance = match_distance_threshold
            for face in faces:
                distance = ((face.center_x - tracked_position[0]) ** 2 +
                           (face.center_y - tracked_position[1]) ** 2) ** 0.5
                if distance < best_distance:
                    best_distance = distance
                    best_match = face
            return best_match
        
        return None

    # ========== 人脸检测辅助 ==========
    
    def filter_valid_faces(self, faces: List['FaceInfo'], require_frontal: bool = True) -> List['FaceInfo']:
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
        valid_faces = self.filter_valid_faces(faces, require_frontal=True)
        if not valid_faces:
            return None
        
        def score(face: 'FaceInfo') -> float:
            area_score = min(1.0, face.area / 0.1)
            distance_to_center_x = abs(face.center_x - center_x)
            distance_to_center_y = abs(face.center_y - center_y)
            distance_to_center = distance_to_center_x + distance_to_center_y
            center_score = max(0.0, 1.0 - distance_to_center * 2)
            return area_score * (1 - self.CENTER_WEIGHT) + center_score * self.CENTER_WEIGHT

        best_face = max(valid_faces, key=score)        
        return best_face
    
    def find_matching_face(self, faces: List['FaceInfo'], target: 'FaceInfo', 
                          threshold: Optional[float] = None) -> Optional['FaceInfo']:
        if not faces:
            return None
        
        if target.face_id >= 0:
            for face in faces:
                if face.face_id == target.face_id:
                    if face.is_frontal:
                        return face
        
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
        if region is None:
            region = self.VALID_REGION
        x_min, x_max, y_min, y_max = region
        return face.is_in_region(x_min, x_max, y_min, y_max)
    
    # ========== 目标丢失检测 ==========
    
    def enable_target_lost_detection(self, timeout: float) -> None:
        self._target_lost_timeout = timeout
        self._target_lost_time = 0.0
    
    def enable_state_timeout(self, timeout: float) -> None:
        self._state_timeout = timeout
    
    def update_target_lost_time(self, has_target: bool, dt: float = 0.02) -> None:
        if has_target:
            self._target_lost_time = 0.0
        else:
            self._target_lost_time += dt
    
    def is_target_lost_timeout(self) -> bool:
        if self._target_lost_timeout is None:
            return False
        return self._target_lost_time > self._target_lost_timeout
    
    def is_state_timeout(self) -> bool:
        if self._state_timeout is None:
            return False
        return self.get_elapsed_time() > self._state_timeout
    
    def get_default_smooth_coefficient(self) -> float:
        return FaceFollowingHelper.get_state_smooth(self.state_string)
