"""
SEARCH 状态（搜寻目标）

简化版：只检测人脸，正脸持续 2s 后锁定进入 TRACKING

行为：
- 检测并锁定最大正脸目标
- 确认目标的互动意向（正脸持续时间 ≥ 2s）
- 锁定后传递给 TRACKING 状态
"""

import time
import math
from typing import TYPE_CHECKING, Optional, Tuple

from ..enums import WeddingStateName
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM


class SearchState(WeddingState):
    """
    搜寻目标状态（简化版）
    
    核心逻辑：
    - 选择最大正脸作为目标
    - 正脸持续 ≥ 2s → 锁定 → 进入 TRACKING
    - 正脸丢失 → 重置计时
    - 状态超时 → 回到 IDLE
    
    状态转换：
    - → TRACKING: 正脸持续 ≥ 2s
    - → IDLE: 状态超时（> 10s）
    """
    
    # ========== 配置参数 ==========
    
    # 时间参数
    T_FACE_CONFIRM = 2.0       # 正脸持续确认时间（秒），临时改为20s用于测试跟随功能
    T_SEARCH_TIMEOUT = 3.0     # 状态最大持续时间（秒）
    T_NO_TARGET_TIMEOUT = 1.0     # 无目标超时时间（秒），超过此时间无目标则进入 IDLE
    
    
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.SEARCH)
        
        # 跟踪器（延迟初始化）
        self._tracker = None
        
        # 用于跟踪目标移动速度（用于优化跟随）
        self._last_target_pos: Optional[Tuple[float, float]] = None
        
        # 无目标状态开始时间（用于检测无目标超时）
        self._no_target_start_time: Optional[float] = None
        
    
    def _get_tracker(self):
        """延迟初始化 FaceTracker"""
        if self._tracker is None:
            from ...perception import FaceTracker
            self._tracker = FaceTracker()
            # 同步配置参数
            self._tracker.CONFIRM_DURATION = self.T_FACE_CONFIRM
        return self._tracker
    
    def on_enter(self) -> None:
        self.log("Entering SEARCH state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 重置目标位置跟踪（用于检测目标移动速度）
        self._last_target_pos = None
        
        # 重置无目标计时
        self._no_target_start_time = None
        
        # ========== 修复：重置 look_at 到图像中心 ==========
        # 确保进入 SEARCH 状态时，look_at 从中心开始
        self.set_look_at(0.5, 0.5)
        self.log("Reset look_at to center (0.5, 0.5)")
        
        # ========== 修复：重置 PID 状态 ==========
        # 清除 PID 积分项和上次误差，避免状态残留
        if hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
            self.log("Reset PID state")
        
        # ========== 修复：重置平滑状态 ==========
        # 清除平滑状态，避免从IDLE状态残留的平滑值影响SEARCH状态
        if hasattr(self, '_smoothed_look_at_x'):
            delattr(self, '_smoothed_look_at_x')
            delattr(self, '_smoothed_look_at_y')
            self.log("Reset smoothing state")
        
        # 清除动作包，确保不会执行大幅摆动（从 IDLE 进入时可能残留）
        if self._current_pack is not None:
            self.log("Clearing action pack in SEARCH state")
            self._current_pack = None
            self._pack_start_time = 0.0
        
        # 启用状态持续时间超时（使用基类方法）
        self.enable_state_timeout(self.T_SEARCH_TIMEOUT)
        
        # 获取 IDLE 传递的候选
        locked_target = self.fsm.data.perception.locked_target
        
        # 初始化跟踪器
        tracker = self._get_tracker()
        if locked_target:
            # 使用 IDLE 的候选初始化跟踪器
            tracker.set_initial_target(locked_target)
            self.log(f"Using IDLE candidate: face_id={locked_target.face_id}, "
                    f"pos=({locked_target.center[0]:.2f}, {locked_target.center[1]:.2f})")
            # 初始化上一帧目标位置
            self._last_target_pos = (locked_target.center[0], locked_target.center[1])
            # 设置可视化用的target_face_id
            self.fsm.data.perception.target_face_id = locked_target.face_id
        else:
            # 如果没有候选，重置（不应该发生，但容错处理）
            self.log("Warning: No locked_target from IDLE, resetting tracker")
            tracker.reset()
            # 清除可视化用的target_face_id
            self.fsm.data.perception.target_face_id = None
        
        # 设置中立 Pose
        self.set_pose("neutral")
        
        self.log(f"SEARCH config: T_FACE_CONFIRM={self.T_FACE_CONFIRM}s, "
                f"T_SEARCH_TIMEOUT={self.T_SEARCH_TIMEOUT}s, "
                f"T_NO_TARGET_TIMEOUT={self.T_NO_TARGET_TIMEOUT}s")
    
    def run(self) -> None:
        """执行搜寻逻辑"""
        # 获取感知数据
        perception = self.fsm.data.perception
        
        # 更新跟踪器
        tracker = self._get_tracker()
        state = tracker.update(perception.faces)
        
        self.log(f"[SEARCH调试] 跟踪器状态: has_target={state.has_target}, is_frontal={state.is_frontal}, "
                f"frontal_duration={state.frontal_duration:.2f}s, position=({state.position[0]:.3f}, {state.position[1]:.3f})")
        if state.target_face:
            self.log(f"[SEARCH调试]   target_face: id={state.target_face.face_id}, "
                    f"pos=({state.target_face.center_x:.3f}, {state.target_face.center_y:.3f}), "
                    f"frontal={state.target_face.is_frontal}, area={state.target_face.area:.4f}")
        else:
            self.log(f"[SEARCH调试]   target_face: None")
        
        # 更新无目标计时
        current_time = self.fsm.get_current_time()
        if state.has_target:
            # 有目标，重置无目标计时
            self._no_target_start_time = None
        else:
            # 无目标，开始或继续计时
            if self._no_target_start_time is None:
                self._no_target_start_time = current_time
                self.log(f"[SEARCH调试] 开始无目标计时")
        
        # 记录跟随前的 look_at 位置
        look_at_before = (self.fsm.data.motion.look_at_target[0], self.fsm.data.motion.look_at_target[1])
        
        # 根据跟踪状态执行动作
        if state.has_target:
            # 有目标，执行跟随（使用基类的统一跟随流程）
            target_face = state.target_face
            if target_face:
                # 更新可视化用的target_face_id
                self.fsm.data.perception.target_face_id = target_face.face_id
                
                # ========== 验证日志：记录 target_x 历史变化（用于验证图像坐标系方向） ==========
                target_pos = (target_face.center_x, target_face.center_y)
                
                # ========== 使用新的 PID 控制跟随（不依赖 look_at 和 target 的偏差） ==========
                self.pid_follow_target_face(target_face, "[SEARCH跟随]")
                look_at_after = (self.fsm.data.motion.look_at_target[0], self.fsm.data.motion.look_at_target[1])
                self.log(f"[SEARCH跟随] 执行跟随: target_pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}), "
                        f"look_at_before=({look_at_before[0]:.3f}, {look_at_before[1]:.3f}), "
                        f"look_at_after=({look_at_after[0]:.3f}, {look_at_after[1]:.3f})")
                
        else:
            # 无目标，清除可视化用的target_face_id
            self.fsm.data.perception.target_face_id = None
            # 微小扫描
            if self.iter_count % 10 == 0:
                self.log(f"[SEARCH调试] 无目标，执行微小扫描")
            #self._idle_scan()
        
        self.iter_count += 1
    
    
    def _idle_scan(self) -> None:
        """无目标时的空闲扫描"""
        elapsed = self.get_elapsed_time()
        sway = 0.05 * math.sin(2 * math.pi * elapsed / 3.0)
        self.set_look_at(0.5 + sway, 0.5)
    
    def check_transition(self) -> WeddingStateName:
        """检查状态转换"""
        # 先检查基类的转换逻辑（安全/命令）
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        tracker = self._get_tracker()
        
        # 1. 检查进入 TRACKING 条件（正脸持续 ≥ 2s）
        if tracker.is_target_confirmed:
            self.log(f"Target confirmed after {tracker._frontal_duration:.1f}s, "
                    f"transitioning to TRACKING")
            
            # 保存锁定目标到感知数据
            locked = tracker.get_locked_target()
            if locked:
                self.fsm.data.perception.locked_target = locked
            
            return WeddingStateName.TRACKING
        
        # 2. 检查无目标超时（无目标持续 ≥ 2s）
        if self._no_target_start_time is not None:
            no_target_duration = self.fsm.get_current_time() - self._no_target_start_time
            if no_target_duration >= self.T_NO_TARGET_TIMEOUT:
                self.log(f"No target for {no_target_duration:.1f}s (>= {self.T_NO_TARGET_TIMEOUT}s), "
                        f"returning to IDLE")
                return WeddingStateName.IDLE
        
        # 3. 检查状态超时（使用基类方法）
        if self.is_state_timeout():
            self.log(f"SEARCH timeout ({self.T_SEARCH_TIMEOUT}s), returning to IDLE")
            return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        tracker = self._get_tracker()
        info = tracker.get_info()
        self.log(f"Exiting SEARCH state (elapsed={self.get_elapsed_time():.1f}s, "
                f"frontal_duration={info.get('frontal_duration', 0):.1f}s)")
    
    def get_search_info(self) -> dict:
        """获取搜寻状态信息（用于调试）"""
        tracker = self._get_tracker()
        info = tracker.get_info()
        
        return {
            'elapsed_time': self.get_elapsed_time(),
            'timeout_remaining': max(0, self.T_SEARCH_TIMEOUT - self.get_elapsed_time()),
            **info
        }
