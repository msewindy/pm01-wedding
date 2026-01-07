"""
IDLE 状态（空闲）

行为：
- 执行待机动作包（头部/腰部摆动 + 可选语音）
- 按规则随机切换动作包（每10秒）
- 等待目标出现，使用稳定性跟踪确认目标

正对前方检测机制：
- 持续检测机器人是否正对前方（look_at_target 接近 0.5, 0.5）
- 当正对前方时，停止转动，开启检测窗口（0.5秒）
- 在检测窗口内检测人脸：
  - 如果检测到人脸，保持静止，等待确认（不清除action pack）
  - 如果0.5s内没有检测到人脸，退出检测窗口，继续执行action pack
  - 确认要进入SEARCH时，才清除action pack
- 精准的跟随让SEARCH状态去做，IDLE只负责检测和确认

IDLE → SEARCH 切换逻辑：
- 只在检测窗口内检测人脸
- 选择靠近画面中心的正脸作为候选
- 候选需要稳定出现（跨帧匹配）0.3s 以上
- 确认后进入 SEARCH，跟随逻辑由SEARCH处理
"""

import math
import random
import time
from typing import TYPE_CHECKING, Optional, List, Tuple

from ..enums import WeddingStateName
from ..wedding_state import WeddingState
from ...action_pack import ActionPack, ActionType

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM
    from ...perception import FaceInfo


class IdleState(WeddingState):
    """
    空闲状态
    
    改进的目标检测逻辑：
    - 不是简单选择"最大正脸"
    - 优先选择靠近当前视野中心的正脸
    - 候选目标需要跨帧稳定匹配
    - 短暂丢失有容忍机制
    """
    
    # ========== 配置参数 ==========
    
    # 动作包参数
    SWITCH_INTERVAL = 10.0       # 动作包切换间隔（秒）
    SPEECH_PROBABILITY = 0.3     # 选择带语音动作包的概率
    SPEECH_COOLDOWN = 30.0       # 语音冷却时间（秒）
    
    # 目标检测参数
    FACE_CONFIRM_TIME = 0.3      # 候选确认时间（秒）
    FACE_LOST_TOLERANCE = 0.1    # 候选丢失容忍时间（秒）
    MATCH_DISTANCE_THRESHOLD = 0.25  # 跨帧匹配距离阈值（从0.15增加到0.25，容忍机器人旋转导致的位置变化）
    
    # 正对前方检测参数
    FORWARD_FACING_THRESHOLD = 0.05  # 正对前方判断阈值（归一化坐标，0.05 = 5%）
    FORWARD_FACING_DETECTION_TIME = 0.5  # 正对前方检测时间（秒）
    
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.IDLE)
        
        # 动作包池（状态特定）
        self._silent_packs: List[str] = []
        self._speech_packs: List[str] = []
        self._init_pack_pools()
        
        self.log(f"Action enabled: {self._enable_action}, Speech enabled: {self._enable_speech}")
        
        # 切换控制
        self._last_switch_time = 0.0
        self._last_speech_time = 0.0
        self._played_speeches: List[str] = []
        
        # ===== 目标检测状态 =====
        self._candidate_face: Optional['FaceInfo'] = None  # 候选目标
        self._candidate_start_time: float = 0.0            # 候选开始时间
        self._candidate_last_seen: float = 0.0             # 候选最后一次看到的时间
        self._candidate_lost_duration: float = 0.0         # 候选丢失持续时间
        
        # ===== 正对前方检测状态 =====
        self._forward_facing_start_time: float = 0.0  # 开始正对前方的时间
        self._detection_window_start_time: float = 0.0  # 检测窗口开始的时间
        self._is_in_detection_window: bool = False     # 是否在检测窗口内
        self._just_exited_window: bool = False         # 是否刚退出检测窗口（用于强制执行action pack）
        
    def _init_pack_pools(self) -> None:
        """初始化动作包池（状态特定）"""
        self._init_action_library()
        if self._action_library is None:
            return
        
        idle_packs = self._action_library.get_by_type(ActionType.IDLE_SWAY)
        
        for pack in idle_packs:
            if pack.has_speech():
                self._speech_packs.append(pack.name)
            else:
                self._silent_packs.append(pack.name)
        
        self.log(f"Pack pools: {len(self._silent_packs)} silent, {len(self._speech_packs)} with speech")
        
    def on_enter(self) -> None:
        self.log("Entering IDLE state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        self._last_switch_time = self.fsm.get_current_time()
        
        # 重置目标检测状态
        self._reset_candidate()
        
        # 重置正对前方检测状态
        self._forward_facing_start_time = 0.0
        self._detection_window_start_time = 0.0
        self._is_in_detection_window = False
        
        # 重置已播放列表
        self._played_speeches.clear()
        
        # 选择初始动作包（无语音）
        self._select_random_pack(allow_speech=False)
        
        # 设置中立 Pose
        self.set_pose("neutral")
    
    def _reset_candidate(self) -> None:
        """重置候选目标状态"""
        self._candidate_face = None
        self._candidate_start_time = 0.0
        self._candidate_last_seen = 0.0
        self._candidate_lost_duration = 0.0
        # 清除可视化用的target_face_id
        self.fsm.data.perception.target_face_id = None
        
    def _select_random_pack(self, allow_speech: bool = True) -> None:
        """随机选择动作包"""
        current_time = self.fsm.get_current_time()
        
        # 决定是否选择带语音的动作包
        use_speech = False
        if allow_speech and self._speech_packs:
            if current_time - self._last_speech_time >= self.SPEECH_COOLDOWN:
                use_speech = random.random() < self.SPEECH_PROBABILITY
        
        # 选择动作包
        if use_speech:
            available = [p for p in self._speech_packs if p not in self._played_speeches]
            if not available:
                self._played_speeches.clear()
                available = self._speech_packs
            pack_name = random.choice(available)
        else:
            if self._silent_packs:
                pack_name = random.choice(self._silent_packs)
            elif self._speech_packs:
                pack_name = random.choice(self._speech_packs)
            else:
                pack_name = None
        
        if pack_name:
            # 使用基类的 load_action_pack 方法
            if self.load_action_pack(pack_name):
                if use_speech:
                    self._last_speech_time = current_time
                    self._played_speeches.append(pack_name)
                # 记录加载信息
                if self._current_pack:
                    self.log(f"Loaded pack: {pack_name} (head={math.degrees(self._current_pack.head_amplitude):.1f}°)")
    
    def switch_action_pack(self, pack_name: str) -> bool:
        """切换动作包（外部接口，覆盖基类方法以添加额外逻辑）"""
        if super().switch_action_pack(pack_name):
            self._last_switch_time = self.get_current_time()
            return True
        return False
        
    def run(self) -> None:
        """执行待机逻辑"""
        current_time = self.fsm.get_current_time()
        
        # 1. 检查是否需要切换动作包（仅在无候选且不在检测窗口时）
        if self._candidate_face is None and not self._is_in_detection_window:
            if current_time - self._last_switch_time >= self.SWITCH_INTERVAL:
                self._select_random_pack(allow_speech=True)
                self._last_switch_time = current_time
        
        # 2. 更新正对前方检测状态（仅在无候选时）
        if self._candidate_face is None:
            self._update_forward_facing_state(current_time)
        
        # 3. 执行动作（根据正对前方状态和检测窗口状态）
        if self._is_in_detection_window:
            # 在检测窗口内：停止转动，保持正对前方（不执行action pack，不跟随）
            # 保持静止，等待确认（不清除action pack，直到确认进入SEARCH）
            self.set_look_at(0.5, 0.5)
        elif self._candidate_face is None:
            # 不在检测窗口且无候选
            # 如果刚退出检测窗口，强制执行一次action pack，让机器人继续摆动
            if self._just_exited_window:
                self.execute_action_pack()
                self._just_exited_window = False  # 重置标志
            else:
                # 检查是否正对前方
                is_facing = self._is_facing_forward()
                if is_facing:
                    # 正对前方：停止执行action pack，保持静止，开始计时
                    # 这样可以让 _is_facing_forward() 持续返回 True，累积到 0.5 秒
                    self.set_look_at(0.5, 0.5)
                else:
                    # 不正对前方：执行 action pack（正常摆动）
                    self.execute_action_pack()
        
        # 4. 更新目标检测（在 check_transition 中处理）
        
        self.iter_count += 1
    
    # 注意：_focus_on_candidate() 方法已移除
    # IDLE状态不再进行跟随，跟随逻辑由SEARCH状态处理
    
    def check_transition(self) -> WeddingStateName:
        """检查状态转换"""
        # 先检查基类的转换逻辑（安全/命令）
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 更新目标检测
        self._update_target_detection()
            
        # 检查是否确认目标
        if self._is_candidate_confirmed():
            # 确认要进入SEARCH，清除action pack
            if self._current_pack is not None:
                self.log("[INFO] 清除action pack，准备进入SEARCH")
                self._current_pack = None
                self._pack_start_time = 0.0
            
            # 保存候选到感知数据，传递给 SEARCH
            if self._candidate_face:
                from ...perception import LockedTarget
                self.fsm.data.perception.locked_target = LockedTarget.from_face(self._candidate_face)
                self.log(f"[INFO] 目标确认，保存到locked_target: "
                        f"face_id={self._candidate_face.face_id}, "
                        f"pos=({self._candidate_face.center_x:.2f}, {self._candidate_face.center_y:.2f})")
            
            duration = time.time() - self._candidate_start_time
            self.log(f"[INFO] 目标确认，进入SEARCH (持续时间 {duration:.2f}s)")
            return WeddingStateName.SEARCH
        
        return self.state_name
    
    def _is_facing_forward(self, threshold: float = None) -> bool:
        """
        检查机器人是否正对前方
        
        Args:
            threshold: 判断阈值（归一化坐标），如果为None则使用类常量
        
        Returns:
            True 如果正对前方
        """
        if threshold is None:
            threshold = self.FORWARD_FACING_THRESHOLD
        
        look_at = self.fsm.data.motion.look_at_target
        # 检查是否接近中心 (0.5, 0.5)
        dx = abs(look_at[0] - 0.5)
        dy = abs(look_at[1] - 0.5)
        is_facing = dx < threshold and dy < threshold
        
        return is_facing
    
    def _update_forward_facing_state(self, current_time: float) -> None:
        """
        更新正对前方检测状态
        
        逻辑：
        - 当正对前方时，开始计时
        - 持续正对前方0.5s后，进入检测窗口（停止转动，开启检测）
        - 在检测窗口内，如果0.5s内没有检测到人脸，退出检测窗口
        
        Args:
            current_time: 当前时间
        """
        is_facing = self._is_facing_forward()
        
        if is_facing:
            # 正对前方
            if self._forward_facing_start_time == 0.0:
                # 刚开始正对前方，记录开始时间
                self._forward_facing_start_time = current_time
                self._is_in_detection_window = False
            else:
                # 已经正对前方一段时间
                elapsed = current_time - self._forward_facing_start_time
                
                if elapsed >= self.FORWARD_FACING_DETECTION_TIME:
                    # 持续正对前方超过检测时间，进入检测窗口
                    if not self._is_in_detection_window:
                        self._is_in_detection_window = True
                        self._detection_window_start_time = current_time
                        self.log(f"[INFO] 进入检测窗口 (正对前方持续 {elapsed:.2f}s)")
                    # 检查检测窗口是否超时（0.5s内没有检测到人脸）
                    elif self._candidate_face is None:
                        # 检测窗口已开启，但还没有候选
                        window_duration = current_time - self._detection_window_start_time
                        if window_duration >= self.FORWARD_FACING_DETECTION_TIME:
                            # 检测窗口超时（0.5s内没有检测到人脸），退出检测窗口
                            self.log(f"[INFO] 退出检测窗口 (窗口内 {window_duration:.2f}s 未检测到人脸)")
                            self._forward_facing_start_time = 0.0
                            self._detection_window_start_time = 0.0
                            self._is_in_detection_window = False
                            self._just_exited_window = True  # 标记刚退出检测窗口，下次run()时强制执行action pack
        else:
            # 不正对前方，重置状态
            if self._forward_facing_start_time > 0.0:
                # 刚离开正对前方状态
                elapsed = current_time - self._forward_facing_start_time
                if self._is_in_detection_window:
                    if self._candidate_face is None:
                        # 没有候选，退出检测窗口
                        self.log(f"[INFO] 退出检测窗口 (离开正对前方，已正对 {elapsed:.2f}s，无候选)")
                        self._forward_facing_start_time = 0.0
                        self._detection_window_start_time = 0.0
                        self._is_in_detection_window = False
                        self._just_exited_window = True  # 标记刚退出检测窗口，下次run()时强制执行action pack
                    else:
                        # 有候选但离开正对前方，保持检测窗口（等待确认）
                        pass
                else:
                    # 还没进入检测窗口就离开正对前方，重置
                    self._forward_facing_start_time = 0.0
    
    def _update_target_detection(self) -> None:
        """
        更新目标检测逻辑
        
        逻辑：
        - 只在检测窗口内执行检测
        - 如果检测到人脸，立即清除action pack，保持静止
        - 候选需要稳定出现（跨帧匹配）0.3s以上才确认
        """
        # 只在检测窗口内执行检测（或已有候选时继续匹配）
        if not self._is_in_detection_window and self._candidate_face is None:
            return
        
        perception = self.fsm.data.perception
        faces = perception.faces
        current_time = time.time()
        
        # 获取当前头部朝向（用于计算"中心"）
        current_look_x = self.fsm.data.motion.look_at_target[0]        

        
        if self._candidate_face is None and self._is_in_detection_window:
            # 没有候选，尝试获取新候选（只在检测窗口内）
                    # 选择最佳正脸（使用基类方法）
            best_face = self.select_best_face(faces, current_look_x, 0.5)
            if best_face is not None :
                self._candidate_face = best_face
                self._candidate_start_time = current_time
                self._candidate_last_seen = current_time
                self._candidate_lost_duration = 0.0
                # 更新可视化用的target_face_id
                self.fsm.data.perception.target_face_id = best_face.face_id
                # 检测到候选，但不立即清除action pack（在确认进入SEARCH前才清除）
                # 在检测窗口内，run()方法会保持静止，不执行action pack
                self.log(f"[INFO] 检测到新候选: face_id={best_face.face_id}, "
                        f"pos=({best_face.center_x:.2f}, {best_face.center_y:.2f}), "
                        f"area={best_face.area:.4f}")
        else:
            # 有候选，尝试匹配（使用基类方法）
            matched = self.find_matching_face(faces, self._candidate_face)
            
            if matched is not None and matched.is_frontal:
                # 匹配成功，更新候选
                # 检查是否使用了 ID 匹配
                used_id_match = (self._candidate_face.face_id >= 0 and 
                               matched.face_id == self._candidate_face.face_id)
                self._candidate_face = matched
                self._candidate_last_seen = current_time
                self._candidate_lost_duration = 0.0
                # 更新可视化用的target_face_id
                self.fsm.data.perception.target_face_id = matched.face_id
                if used_id_match and self.iter_count % 30 == 0:  # 每30帧记录一次
                    self.log(f"[INFO] 候选匹配成功 (使用ID匹配): face_id={matched.face_id}")
            else:
                # 匹配失败，累计丢失时间
                self._candidate_lost_duration = current_time - self._candidate_last_seen
                
                if self._candidate_lost_duration > self.FACE_LOST_TOLERANCE:
                    # 丢失超时，放弃候选
                    self.log(f"[WARN] 候选丢失 (丢失持续时间 {self._candidate_lost_duration:.2f}s)")
                    self._reset_candidate()
                    # 清除可视化用的target_face_id
                    self.fsm.data.perception.target_face_id = None
    
    def _is_candidate_confirmed(self) -> bool:
        """检查候选是否已确认"""
        if self._candidate_face is None:
            return False
        
        current_time = time.time()
        duration = current_time - self._candidate_start_time
        
        # 排除丢失时间
        effective_duration = duration - self._candidate_lost_duration
        
        return effective_duration >= self.FACE_CONFIRM_TIME
    
    def on_exit(self) -> None:
        self.log("Exiting IDLE state")
        # 不重置候选，让 SEARCH 继续使用
        self._current_pack = None
    
    def get_idle_info(self) -> dict:
        """获取 IDLE 状态信息（调试用）"""
        return {
            'has_candidate': self._candidate_face is not None,
            'candidate_duration': time.time() - self._candidate_start_time if self._candidate_face else 0,
            'candidate_lost_duration': self._candidate_lost_duration,
            'current_pack': self._current_pack.name if self._current_pack else None,
        }
