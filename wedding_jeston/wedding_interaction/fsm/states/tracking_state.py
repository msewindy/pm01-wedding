"""
TRACKING 状态（跟随）

行为：
- 执行跟踪动作 (tracking_active)
- 启用 PID 跟随
- 目标丢失检测与超时处理
- 稳定跟踪后触发 INTERVIEW
"""

import random
from typing import TYPE_CHECKING, Optional, Tuple

from ..enums import WeddingStateName, WeddingEvent
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM
    from ...perception import FaceInfo


class TrackingState(WeddingState):
    """
    跟随状态
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.TRACKING)
        
        # Load params from config
        config = self.fsm.data.config or {}
        
        # 目标丢失控制
        self.lost_timeout = config.get('tracking_lost_timeout', 5.0)
        
        # 状态转换参数
        self.stable_tracking_duration = config.get('tracking_stable_duration', 2.0)
        self.auto_interview_duration = config.get('tracking_auto_interview_duration', 1.5)
        self.interview_trigger_min_lost = config.get('tracking_interview_trigger_min_lost', 0.5)
        self.interview_trigger_max_lost = config.get('tracking_interview_trigger_max_lost', 1.5)

    def on_enter(self) -> None:
        self.log("Entering TRACKING state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 配置全局跟踪器
        self.fsm.face_tracker.configure(lost_timeout=self.lost_timeout)
        
        # 获取 SEARCH 锁定的目标
        locked_target = self.fsm.data.perception.locked_target
        if locked_target:
            self.fsm.face_tracker.set_target(locked_target)
            self.fsm.data.perception.target_face_id = locked_target.face_id
            self.log(f"Tracking target: {locked_target.face_id}")
        else:
            self.fsm.data.perception.target_face_id = None
            self.log("Warning: No locked_target, tracking may fail")
        
        # 重置 PID 和平滑状态
        self._reset_pid_state()
        
        # 启动跟踪动作
        self.execute_action("tracking_active")
    
    def _reset_pid_state(self):
        """重置 PID 和平滑相关状态"""
        if hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
        
        if hasattr(self, '_smoothed_look_at_x'):
            delattr(self, '_smoothed_look_at_x')
            delattr(self, '_smoothed_look_at_y')

    
    def run(self) -> None:
        """执行跟随逻辑"""
        perception = self.fsm.data.perception
        
        # 1. 更新跟踪器
        state = self.fsm.face_tracker.update(perception.faces)
        
        # 2. 执行跟随 (包含 Grace Period 期间的预测跟随)
        if state.has_target:
            target_face = state.target_face
            
            # 更新感知数据中的目标ID
            self.fsm.data.perception.target_face_id = target_face.face_id
            
            # 使用 PID 跟随
            self.pid_follow_target_face(target_face, "[TRACKING跟随]")
            
            # 确保 motion strategy 正确
            if self.current_motion_strategy != "track_face":
                 self.execute_action("tracking_active")
                 
            # 稳定日志
            is_stable = (state.tracking_duration > self.stable_tracking_duration)
            if is_stable and self.iter_count % 50 == 0:
                 pass
        else:
            # 目标彻底丢失
            self.fsm.data.perception.target_face_id = None
            if self.iter_count % 20 == 0:
                self.log_debug("Target completely lost")
        
        # 3. 周期性语音 (交由 ActionManager 托管)
        pass

        self.iter_count += 1
            
    def check_transition(self) -> WeddingStateName:
        # 先检查基类转换
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 检查命令 (Interview)
        if self.fsm.data.pending_command == WeddingEvent.CMD_START_INTERVIEW:
            return WeddingStateName.INTERVIEW
        if self.fsm.data.audio.voice_command == "interview":
            return WeddingStateName.INTERVIEW
        
        # 更新并获取状态
        state = self.fsm.face_tracker.update(self.fsm.data.perception.faces)
        
        # 1. 目标彻底丢失 -> IDLE
        if not state.has_target:
            self.log(f"Target lost timeout, returning to IDLE")
            return WeddingStateName.IDLE
            
        # 2. 自动触发采访 (稳定跟踪超时)
        if state.tracking_duration > (self.stable_tracking_duration + self.auto_interview_duration):
             self.log(f"Stable tracking timeout ({state.tracking_duration:.1f}s), triggering INTERVIEW")
             return WeddingStateName.INTERVIEW
        
        # 3. 稳定后的短暂丢失 -> 采访 (假设目标走近)
        if state.is_lost:
             # 判断是否稳定过: tracking_duration 包含了整个过程
             if state.tracking_duration > self.stable_tracking_duration:
                 if self.interview_trigger_min_lost < state.lost_duration < self.interview_trigger_max_lost:
                     self.log(f"Stable target lost ({state.lost_duration:.2f}s), triggering INTERVIEW")
                     return WeddingStateName.INTERVIEW
        
        return self.state_name
    
    def on_exit(self) -> None:
        # 如果切换到 INTERVIEW，不停止动作（保持 PID 和移动）
        if self.fsm.next_state_name == WeddingStateName.INTERVIEW:
            pass
        else:
            self.stop_action()
            
        self.log("Exiting TRACKING state")
