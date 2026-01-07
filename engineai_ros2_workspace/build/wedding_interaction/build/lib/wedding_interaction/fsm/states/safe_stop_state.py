"""
SAFE_STOP 状态（安全停止）

行为：
- 停止所有动作
- 回到中立位
- 播放提示语
"""

from typing import TYPE_CHECKING

from ..enums import WeddingStateName
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM


class SafeStopState(WeddingState):
    """
    安全停止状态
    
    行为：
    - 立即停止所有动作
    - 回到中立位
    - 播放提示语（如"请稍微退后一点"）
    - 安全条件解除后回到空闲
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.SAFE_STOP)
        
        # 安全状态参数
        self.recovery_wait_time = 2.0  # 恢复等待时间（秒）
        self._safe_start_time = 0.0    # 安全条件开始解除的时间
        
    def on_enter(self) -> None:
        self.log("Entering SAFE_STOP state - Emergency!")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        self._safe_start_time = 0.0
        
        # 立即回到中立位
        self.set_pose("neutral")
        
        # 设置注视中心
        self.set_look_at(0.5, 0.5)
        
        # 播放提示语
        self.set_speech("too_close")
    
    def run(self) -> None:
        """保持安全停止"""
        # 检查是否可以恢复
        if not self.fsm.data.perception.too_close:
            current_time = self.fsm.get_current_time()
            
            if self._safe_start_time == 0.0:
                self._safe_start_time = current_time
                self.log("Safety condition cleared, waiting for recovery")
            
            # 等待一段时间确认安全
            if current_time - self._safe_start_time > self.recovery_wait_time:
                self.fsm.data.safety_triggered = False
                self.log("Safety recovery confirmed")
        else:
            # 安全条件仍然触发，重置计时
            self._safe_start_time = 0.0
        
        self.iter_count += 1
    
    def check_transition(self) -> WeddingStateName:
        # 不调用基类的 check_transition，因为安全状态有特殊逻辑
        
        # 检查外部强制命令
        from ..enums import WeddingEvent
        cmd = self.fsm.data.pending_command
        if cmd == WeddingEvent.CMD_STOP or cmd == WeddingEvent.CMD_GOTO_IDLE:
            self.fsm.data.safety_triggered = False
            return WeddingStateName.IDLE
        
        # 安全条件解除 -> 回到空闲
        if not self.fsm.data.safety_triggered:
            return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting SAFE_STOP state")
        self._safe_start_time = 0.0

