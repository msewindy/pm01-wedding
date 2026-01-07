"""
FAREWELL 状态（送别）

行为：
- 挥手动作
- 播放送别语
"""

from typing import TYPE_CHECKING

from ..enums import WeddingStateName
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM


class FarewellState(WeddingState):
    """
    送别状态
    
    行为：
    - 播放送别语
    - 执行挥手动作
    - 完成后回到空闲
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.FAREWELL)
        
        # 送别持续时间
        self.farewell_duration = 2.5  # 秒
        
    def on_enter(self) -> None:
        self.log("Entering FAREWELL state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 播放送别语
        self.set_speech("farewell")
        
        # 执行挥手动作
        self.set_pose("wave")
    
    def run(self) -> None:
        """执行送别"""
        # 挥手动作由 motion adapter 处理
        self.iter_count += 1
    
    def check_transition(self) -> WeddingStateName:
        # 先检查基类的转换逻辑
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 送别完成 -> 回到空闲
        if self.get_elapsed_time() > self.farewell_duration:
            self.log("Farewell complete, returning to IDLE")
            return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting FAREWELL state")
        # 回到中立位
        self.set_pose("neutral")

