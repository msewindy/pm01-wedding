"""
PHOTO_POSING 状态（合影）

行为：
- 执行 Pose 动作
- 倒数 3-2-1
- 播放快门声
"""

from typing import TYPE_CHECKING

from ..enums import WeddingStateName
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM


class PhotoPosingState(WeddingState):
    """
    合影状态
    
    行为：
    - 播放确认语
    - 执行 Pose 动作
    - 倒数 3-2-1
    - 播放快门声
    - 播放完成语
    """
    
    # 合影流程阶段
    PHASE_CONFIRM = "confirm"
    PHASE_POSE = "pose"
    PHASE_COUNTDOWN = "countdown"
    PHASE_SHUTTER = "shutter"
    PHASE_DONE = "done"
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.PHOTO_POSING)
        
        # 合影流程阶段
        self.phase = self.PHASE_CONFIRM
        self.phase_start_time = 0.0
        
        # 时间参数
        self.confirm_duration = 1.0   # 确认语时长
        self.pose_duration = 1.5      # Pose 准备时长
        self.countdown_duration = 3.0 # 倒数时长
        self.shutter_duration = 0.5   # 快门音效时长
        self.done_duration = 2.0      # 完成后等待时长
        
        # 选择的 Pose
        self.selected_pose = "heart"
        
    def on_enter(self) -> None:
        self.log("Entering PHOTO_POSING state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 开始确认阶段
        self.phase = self.PHASE_CONFIRM
        self.phase_start_time = self.enter_time
        
        # 播放确认语
        self.set_speech("photo_confirm")
        
        # 根据手势选择 Pose
        gesture = self.fsm.data.perception.gesture
        if gesture == "heart":
            self.selected_pose = "heart"
        elif gesture == "v":
            self.selected_pose = "v_sign"
        else:
            self.selected_pose = "thumbs_up"
        
        self.log(f"Selected pose: {self.selected_pose}")
    
    def run(self) -> None:
        """执行合影流程"""
        current_time = self.fsm.get_current_time()
        phase_elapsed = current_time - self.phase_start_time
        
        if self.phase == self.PHASE_CONFIRM:
            if phase_elapsed > self.confirm_duration:
                self._enter_phase(self.PHASE_POSE)
                # 执行 Pose 动作
                self.set_pose(self.selected_pose)
                
        elif self.phase == self.PHASE_POSE:
            if phase_elapsed > self.pose_duration:
                self._enter_phase(self.PHASE_COUNTDOWN)
                self.set_speech("countdown")
                
        elif self.phase == self.PHASE_COUNTDOWN:
            if phase_elapsed > self.countdown_duration:
                self._enter_phase(self.PHASE_SHUTTER)
                self.set_speech("shutter")
                
        elif self.phase == self.PHASE_SHUTTER:
            if phase_elapsed > self.shutter_duration:
                self._enter_phase(self.PHASE_DONE)
                self.set_speech("photo_done")
        
        self.iter_count += 1
    
    def _enter_phase(self, new_phase: str) -> None:
        """进入新阶段"""
        self.phase = new_phase
        self.phase_start_time = self.fsm.get_current_time()
        self.log(f"Phase: {new_phase}")
    
    def check_transition(self) -> WeddingStateName:
        # 先检查基类的转换逻辑
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 合影完成 -> 回到跟随
        if self.phase == self.PHASE_DONE:
            phase_elapsed = self.fsm.get_current_time() - self.phase_start_time
            if phase_elapsed > self.done_duration:
                self.log("Photo complete, returning to TRACKING")
                return WeddingStateName.TRACKING
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting PHOTO_POSING state")
        # 回到中立位
        self.set_pose("neutral")
        self.phase = self.PHASE_CONFIRM

