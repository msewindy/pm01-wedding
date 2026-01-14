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
        
        
        # 目标丢失控制
        self.lost_timeout = 5.0  # 丢失超时（用于基类检测）
        
        # 跟踪目标信息
        self._match_distance_threshold = 0.2  # 位置匹配距离阈值
        
        # 状态转换参数
        self.stable_tracking_duration = 2.0      # 进入稳定跟踪所需时长(秒)
        self.auto_interview_duration = 1.5      # 稳定后自动触发采访的等待时长(秒)
        self.interview_trigger_min_lost = 0.5    # 触发采访的最小丢失时长(秒)
        self.interview_trigger_max_lost = 1.5    # 触发采访的最大丢失时长(秒)
        self.idle_trigger_lost_time = 2.0        # 触发IDLE的丢失时长(秒)

        
    def on_enter(self) -> None:
        self.log("Entering TRACKING state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 启用目标丢失检测（使用基类方法）
        self.enable_target_lost_detection(self.lost_timeout)
        
        # 跟踪稳定性状态初始化
        self._tracking_start_time = 0.0
        self._is_stable_tracking = False

        
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
        
        # 跟踪稳定性检测
        if target_face is not None:
            # 有目标，检查是否满足稳定跟踪条件
            if self._tracking_start_time == 0.0:
                self._tracking_start_time = self.fsm.get_current_time()
            
            # 如果连续跟踪超过一定时间，标记为稳定
            if not self._is_stable_tracking:
                if (self.fsm.get_current_time() - self._tracking_start_time) > self.stable_tracking_duration:
                    self._is_stable_tracking = True
                    self.log("Tracking detected as STABLE")
 
        else:
            # 目标丢失，重置跟踪开始时间（但不重置稳定标记，稳定标记在整个跟踪会话中保持有效，直到完全丢失退出）
            # 或者：如果丢失，是否重置稳定标记？
            # 逻辑：一旦稳定，就认为这是一次成功的跟踪交互。如果在交互中丢失，根据时间触发采访。
            # 如果在这里重置，那么丢失时就无法判断之前是否稳定了。
            # 所以不应该重置 _is_stable_tracking，只重置 _tracking_start_time 以备下次重新计算
            self._tracking_start_time = 0.0
        
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
        
        # 自动采访触发：如果稳定跟踪超过一定时间（7秒），自动触发
        if self._is_stable_tracking:
            elapsed_since_stable_start = self.fsm.get_current_time() - self._tracking_start_time
            if elapsed_since_stable_start > (self.stable_tracking_duration + self.auto_interview_duration):
                 self.log(f"Stable tracking for {elapsed_since_stable_start:.2f}s (auto trigger), transitioning to INTERVIEW")
                 return WeddingStateName.INTERVIEW
        
        # 目标丢失处理逻辑
        # 获取当前目标丢失时长
        lost_time = self._target_lost_time if self.fsm.data.perception.locked_target is None and self.fsm.data.perception.target_face_id is None else 0.0
        # 注意: 上面的判断可能不准确，因为 _target_lost_time 是由 update_target_lost_time 更新的
        # 直接使用 _target_lost_time 即可，因为它在 run() 中被维护
        lost_time = self._target_lost_time
        
        if lost_time > 0:
            # 1. 如果之前是稳定跟踪（成功tracking），且丢失时间较短（在1秒内），触发采访
            # 这里的 "在1秒内" 理解为：丢失时长达到一个短阈值（如0.5s-1.5s），确认不是瞬间闪烁
            # 同时必须小于回IDLE的阈值
            if self._is_stable_tracking:
                if self.interview_trigger_min_lost < lost_time < self.interview_trigger_max_lost:  
                    self.log(f"Stable target lost for {lost_time:.2f}s (trigger in range {self.interview_trigger_min_lost}-{self.interview_trigger_max_lost}s), transitioning to INTERVIEW")
                    return WeddingStateName.INTERVIEW
            
            # 2. 如果目标丢失超过设定时长，回到 IDLE
            if lost_time > self.idle_trigger_lost_time:
                self.log(f"Target lost for {lost_time:.2f}s (> {self.idle_trigger_lost_time}s), transitioning to IDLE")
                return WeddingStateName.IDLE
        
        # 所有的超时都在上面处理了，这里不需要再调用 update_target_lost_timeout
        # 或者保留它作为安全网（但参数需要调整）
        
        return self.state_name

    
    def on_exit(self) -> None:
        self.log("Exiting TRACKING state")

