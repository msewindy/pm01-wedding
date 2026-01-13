"""
SEARCH 状态（搜寻目标）

行为：
- 执行搜寻扫描动作 (search_scan)
- 检测到潜在目标时，切换到跟踪动作 (tracking_active) 并启用 PID 跟随进行确认
- 确认目标的互动意向（正脸持续时间 ≥ 2s）
- 锁定后传递给 TRACKING 状态
"""

import time
from typing import TYPE_CHECKING, Optional, Tuple

from ..enums import WeddingStateName
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM


class SearchState(WeddingState):
    """
    搜寻目标状态（简化版）
    """
    
    # ========== 配置参数 ==========
    
    # 时间参数
    T_FACE_CONFIRM = 2.0       # 正脸持续确认时间
    T_SEARCH_TIMEOUT = 10.0    # 状态最大持续时间
    T_NO_TARGET_TIMEOUT = 5.0  # 无目标超时时间
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.SEARCH)
        
        # 跟踪器（延迟初始化）
        self._tracker = None
        
        # 无目标状态开始时间（用于检测无目标超时）
        self._no_target_start_time: Optional[float] = None
        
    def _get_tracker(self):
        """延迟初始化 FaceTracker"""
        if self._tracker is None:
            from ...perception import FaceTracker
            self._tracker = FaceTracker()
            self._tracker.CONFIRM_DURATION = self.T_FACE_CONFIRM
        return self._tracker
    
    def on_enter(self) -> None:
        self.log("Entering SEARCH state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 重置无目标计时
        self._no_target_start_time = None
        
        # 启用状态超时
        self.enable_state_timeout(self.T_SEARCH_TIMEOUT)
        
        # 获取 IDLE 传递的候选
        locked_target = self.fsm.data.perception.locked_target
        
        # 初始化跟踪器
        tracker = self._get_tracker()
        if locked_target:
            tracker.set_initial_target(locked_target)
            self._no_target_start_time = None # 有目标
            
            # 如果有初始目标，直接进入跟踪模式
            self.execute_action("tracking_active")
            self.log(f"Initial target from IDLE: {locked_target.face_id}")
        else:
            self.log("No initial target, starting scan")
            tracker.reset()
            self._no_target_start_time = self.fsm.get_current_time()
            
            # 开始扫描
            self.execute_action("search_scan")
            
    def run(self) -> None:
        """执行搜寻逻辑"""
        perception = self.fsm.data.perception
        
        # 更新跟踪器
        tracker = self._get_tracker()
        state = tracker.update(perception.faces)
        
        # 状态切换逻辑：如果在扫描中发现目标 -> 切换到跟踪
        if state.has_target:
            # 有目标
            self._no_target_start_time = None
            
            # 如果当前不是 tracking 模式，切换到 tracking
            if self.current_motion_strategy != "track":
                self.execute_action("tracking_active")
                self.log("Target found, switching to tracking strategy")
            
            # 执行 PID 跟随
            if state.target_face:
                self.pid_follow_target_face(state.target_face, "[SEARCH跟随]")
                self.fsm.data.perception.target_face_id = state.target_face.face_id
        else:
            # 无目标
            if self._no_target_start_time is None:
                self._no_target_start_time = self.fsm.get_current_time()
            
            # 如果丢失目标且当前是 tracking 模式，切回 scan
            if self.current_motion_strategy != "scan":
                self.execute_action("search_scan")
                self.log("Target lost, switching back to scan strategy")
            
            self.fsm.data.perception.target_face_id = None
            
        self.iter_count += 1
    
    def check_transition(self) -> WeddingStateName:
        """检查状态转换"""
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        tracker = self._get_tracker()
        
        # 1. 检查进入 TRACKING 条件（正脸持续 ≥ 2s）
        if tracker.is_target_confirmed:
            self.log(f"Target confirmed! Transitioning to TRACKING")
            
            # 保存锁定目标
            locked = tracker.get_locked_target()
            if locked:
                self.fsm.data.perception.locked_target = locked
            
            return WeddingStateName.TRACKING
        
        # 2. 检查无目标超时
        if self._no_target_start_time is not None:
            if self.fsm.get_current_time() - self._no_target_start_time >= self.T_NO_TARGET_TIMEOUT:
                self.log("No target timeout, returning to IDLE")
                return WeddingStateName.IDLE
        
        # 3. 检查状态超时
        if self.is_state_timeout():
            self.log("SEARCH timeout, returning to IDLE")
            return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.stop_action() # 停止当前动作（可选，或由下一状态覆盖）
        self.log("Exiting SEARCH state")
