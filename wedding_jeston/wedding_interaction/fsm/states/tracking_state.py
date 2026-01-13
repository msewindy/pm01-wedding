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
        
        # 目标丢失控制
        self.lost_timeout = 5.0
        
        # 跟踪目标信息
        self._match_distance_threshold = 0.2
        
        # 状态转换参数
        self.stable_tracking_duration = 2.0      # 进入稳定跟踪所需时长(秒)
        self.auto_interview_duration = 1.5       # 稳定后自动触发采访的等待时长(秒)
        self.interview_trigger_min_lost = 0.5    # 触发采访的最小丢失时长(秒)
        self.interview_trigger_max_lost = 1.5    # 触发采访的最大丢失时长(秒)
        self.idle_trigger_lost_time = 2.0        # 触发IDLE的丢失时长(秒)
        
        self.idle_trigger_lost_time = 2.0        # 触发IDLE的丢失时长(秒)

        
    def on_enter(self) -> None:
        self.log("Entering TRACKING state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 启用目标丢失检测
        self.enable_target_lost_detection(self.lost_timeout)
        
        # 跟踪稳定性状态初始化
        self._tracking_start_time = 0.0
        self._is_stable_tracking = False

        self._is_stable_tracking = False
        
        # 获取 SEARCH 锁定的目标
        locked_target = self.fsm.data.perception.locked_target
        if locked_target:
            self._tracked_face_id = locked_target.face_id
            self._tracked_position = locked_target.center
            self.fsm.data.perception.target_face_id = locked_target.face_id
            self.log(f"Tracking target: {self._tracked_face_id}")
        else:
            self._tracked_face_id = None
            self._tracked_position = None
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
        
        # 1. 查找目标
        target_face = self._find_tracked_face(perception.faces)
        
        # 2. 执行跟随或处理丢失
        if target_face is not None:
            # 更新跟踪信息
            target_pos = (target_face.center_x, target_face.center_y)
            self._tracked_position = target_pos
            self.fsm.data.perception.target_face_id = target_face.face_id
            
            # 使用 PID 跟随
            self.pid_follow_target_face(target_face, "[TRACKING跟随]")
            
            # 更新丢失检测
            self.update_target_lost_time(has_target=True)
            
            # 确保 motion strategy 正确 (防止被外部干扰)
            if self.current_motion_strategy != "track":
                 self.execute_action("tracking_active")
                 
        else:
            # 目标丢失
            self.fsm.data.perception.target_face_id = None
            self.update_target_lost_time(has_target=False)
            
            # 丢失时可以停止移动或保持最后位置（当前 PID 逻辑会保持最后位置）
            if self.iter_count % 10 == 0:
                self.log(f"Target lost time: {self._target_lost_time:.2f}s")
        
        # 3. 跟踪稳定性检测
        if target_face is not None:
            if self._tracking_start_time == 0.0:
                self._tracking_start_time = self.fsm.get_current_time()
            
            if not self._is_stable_tracking:
                if (self.fsm.get_current_time() - self._tracking_start_time) > self.stable_tracking_duration:
                    self._is_stable_tracking = True
                    self.log("Tracking detected as STABLE")
        else:
            self._tracking_start_time = 0.0
            
        # 4. 周期性语音 (交由 ActionManager 托管)
        # ActionManager 会根据配置自动播放
        pass

        self.iter_count += 1
            
    
    def _find_tracked_face(self, faces: list) -> Optional['FaceInfo']:
        """查找跟踪的目标脸"""
        matched = self.find_tracked_face_by_id_and_position(
            faces, self._tracked_face_id, self._tracked_position, self._match_distance_threshold
        )
        if matched is not None and (self._tracked_face_id is None or self._tracked_face_id < 0):
            self._tracked_face_id = matched.face_id
        return matched
    
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
        
        # 自动触发采访 (稳定跟踪超时)
        if self._is_stable_tracking:
            elapsed = self.fsm.get_current_time() - self._tracking_start_time
            if elapsed > (self.stable_tracking_duration + self.auto_interview_duration):
                 self.log(f"Stable tracking timeout, triggering INTERVIEW")
                 return WeddingStateName.INTERVIEW
        
        # 目标丢失处理
        lost_time = self._target_lost_time
        if lost_time > 0:
            # 稳定跟踪后的短暂丢失 -> 采访 (假设目标走近或互动结束)
            if self._is_stable_tracking:
                if self.interview_trigger_min_lost < lost_time < self.interview_trigger_max_lost:  
                    self.log(f"Stable target lost ({lost_time:.2f}s), triggering INTERVIEW")
                    return WeddingStateName.INTERVIEW
            
            # 丢失超时 -> IDLE
            if lost_time > self.idle_trigger_lost_time:
                self.log(f"Target lost timeout ({lost_time:.2f}s), returning to IDLE")
                return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.stop_action()
        self.log("Exiting TRACKING state")
