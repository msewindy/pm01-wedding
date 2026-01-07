"""
TRACKING 状态（跟随）

行为：
- 头部/腰部跟随目标位置
- 目标丢失超时后进入送别
"""

from typing import TYPE_CHECKING, Optional, Tuple

from ..enums import WeddingStateName, WeddingEvent
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM
    from ...perception import FaceInfo


class TrackingState(WeddingState):
    """
    跟随状态
    
    行为：
    - 头部/腰部平滑跟随目标位置
    - 目标丢失超时后进入送别
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.TRACKING)
        
        
        # 目标丢失控制（使用基类的 _target_lost_time 和 _target_lost_timeout）
        self.lost_timeout = 1.5  # 丢失超时（秒）
        
        # 跟踪目标信息（使用基类的公共属性）
        # self._tracked_face_id 和 self._tracked_position 已在基类中定义
        self._match_distance_threshold = 0.2  # 位置匹配距离阈值（TRACKING 特定）
        
    def on_enter(self) -> None:
        self.log("Entering TRACKING state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 启用目标丢失检测（使用基类方法）
        self.enable_target_lost_detection(self.lost_timeout)
        
        # 获取 SEARCH 锁定的目标
        locked_target = self.fsm.data.perception.locked_target
        if locked_target:
            # 保存目标信息用于跟踪
            self._tracked_face_id = locked_target.face_id
            self._tracked_position = locked_target.center
            # 设置可视化用的target_face_id
            self.fsm.data.perception.target_face_id = locked_target.face_id
            self.log(f"Tracking target: face_id={self._tracked_face_id}, "
                    f"pos=({self._tracked_position[0]:.2f}, {self._tracked_position[1]:.2f})")
        else:
            self._tracked_face_id = None
            self._tracked_position = None
            # 清除可视化用的target_face_id
            self.fsm.data.perception.target_face_id = None
            self.log("Warning: No locked_target, tracking may fail")
        
        # ========== 重置 PID 状态 ==========
        # 清除 PID 积分项和上次误差，避免从 SEARCH 状态残留的 PID 状态影响 TRACKING
        if hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
            self.log("Reset PID state")
        
        # ========== 重置平滑状态 ==========
        # 清除平滑状态，避免从 SEARCH 状态残留的平滑值影响 TRACKING
        if hasattr(self, '_smoothed_look_at_x'):
            delattr(self, '_smoothed_look_at_x')
            delattr(self, '_smoothed_look_at_y')
            self.log("Reset smoothing state")
        
        # 清除动作包，确保不会执行大幅摆动
        if self._current_pack is not None:
            self.log("Clearing action pack in TRACKING state")
            self._current_pack = None
            self._pack_start_time = 0.0
        
        # 设置中立 Pose
        self.set_pose("neutral")
    
    def run(self) -> None:
        """
        执行跟随逻辑
        
        跟踪特定的脸对象（允许侧脸，只要还能检测到这个脸）
        使用和 SEARCH 状态相同的 PID 跟随方法
        """
        perception = self.fsm.data.perception
        
        # 查找跟踪的目标脸（使用基类的统一方法）
        target_face = self._find_tracked_face(perception.faces)
        
        # 记录跟随前的 look_at 位置
        look_at_before = (self.fsm.data.motion.look_at_target[0], self.fsm.data.motion.look_at_target[1])
        
        # 根据是否找到目标执行动作
        if target_face is not None:
            # 找到目标，执行跟随（使用基类的统一跟随流程，和 SEARCH 状态一样）
            target_pos = (target_face.center_x, target_face.center_y)
            
            # 更新跟踪位置（用于后续匹配）
            self._tracked_position = target_pos
            
            # 更新可视化用的target_face_id
            self.fsm.data.perception.target_face_id = target_face.face_id
            
            # 使用 PID 控制跟随（和 SEARCH 状态使用相同的方法）
            self.pid_follow_target_face(target_face, "[TRACKING跟随]")
            
            look_at_after = (self.fsm.data.motion.look_at_target[0], self.fsm.data.motion.look_at_target[1])
            self.log(f"[TRACKING跟随] 执行跟随: target_pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}), "
                    f"face_id={target_face.face_id}, is_frontal={target_face.is_frontal}, "
                    f"look_at_before=({look_at_before[0]:.3f}, {look_at_before[1]:.3f}), "
                    f"look_at_after=({look_at_after[0]:.3f}, {look_at_after[1]:.3f})")
            
            # 更新目标丢失检测（有目标，重置丢失时间）
            self.update_target_lost_time(has_target=True)
        else:
            # 未找到目标，清除可视化用的target_face_id
            self.fsm.data.perception.target_face_id = None
            # 更新目标丢失检测
            self.update_target_lost_time(has_target=False)
            
            if self.iter_count % 10 == 0:
                self.log(f"[TRACKING调试] 目标丢失，face_id={self._tracked_face_id}, "
                        f"lost_time={self._target_lost_time:.2f}s")
        
        self.iter_count += 1
    
    def _find_tracked_face(self, faces: list) -> Optional['FaceInfo']:
        """
        查找跟踪的目标脸
        
        使用基类的统一匹配方法，通过 face_id 或位置匹配，允许侧脸
        
        Args:
            faces: 当前帧检测到的人脸列表
        
        Returns:
            匹配的目标人脸（可以是侧脸），或 None
        """
        matched = self.find_tracked_face_by_id_and_position(
            faces, self._tracked_face_id, self._tracked_position, self._match_distance_threshold
        )
        
        # 更新 face_id（如果之前没有）
        if matched is not None and (self._tracked_face_id is None or self._tracked_face_id < 0):
            self._tracked_face_id = matched.face_id
        
        return matched
    
    def check_transition(self) -> WeddingStateName:
        # 先检查基类的转换逻辑
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 检查采访触发
        if self.fsm.data.pending_command == WeddingEvent.CMD_START_INTERVIEW:
            self.log("Interview command received, transitioning to INTERVIEW")
            return WeddingStateName.INTERVIEW
        
        # 或者通过语音命令
        if self.fsm.data.audio.voice_command == "interview":
            self.log("Interview voice command received, transitioning to INTERVIEW")
            return WeddingStateName.INTERVIEW
        
        # 目标丢失超时 -> 送别（使用基类方法）
        if self.is_target_lost_timeout():
            self.log("Target lost, transitioning to FAREWELL")
            return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting TRACKING state")

