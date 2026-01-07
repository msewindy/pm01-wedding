"""
FSM 控制器

管理状态机的主循环、状态转换、安全检查等
参考：engineai_humanoid/FSM_States/ControlFSM.h
"""

import time
import logging
from typing import Dict, Optional, Callable, List

from .enums import WeddingStateName, FSMOperatingMode, WeddingEvent
from .wedding_state import WeddingState, TransitionData
from .wedding_fsm_data import WeddingFSMData


class WeddingFSM:
    """
    婚礼互动状态机控制器
    
    职责：
    - 管理所有状态实例
    - 执行状态转换逻辑
    - 运行主控制循环
    - 提供对外接口
    
    参考：engineai_humanoid/FSM_States/ControlFSM.h
    """
    
    def __init__(self, config: Dict = None, ros2_logger = None):
        """
        初始化 FSM
        
        Args:
            config: 配置参数字典
            ros2_logger: ROS2 logger 对象（可选），如果提供，状态类将使用 ROS2 logger
        """
        self.logger = logging.getLogger("WeddingFSM")
        self._ros2_logger = ros2_logger  # ROS2 logger（如果提供）
        self._debug_pub = None  # 调试日志发布器（如果提供）
        self._debug_log_file = None  # 调试日志文件（如果启用）
        
        # 共享数据
        self.data = WeddingFSMData()
        if config:
            self.data.config = config
        
        # 状态列表
        self.states: Dict[WeddingStateName, WeddingState] = {}
        
        # 当前状态
        self.current_state: Optional[WeddingState] = None
        self.next_state: Optional[WeddingState] = None
        self.next_state_name: WeddingStateName = WeddingStateName.INVALID
        
        # 操作模式
        self.operating_mode: FSMOperatingMode = FSMOperatingMode.NORMAL
        
        # 状态转换回调
        self._on_state_change_callbacks: List[Callable] = []
        
        # 运行标志
        self._running = False
        self._start_time = 0.0
        
        # 迭代计数
        self.iter_count = 0
        
        # 转换数据
        self._transition_data = TransitionData()
        
    # ========== 初始化方法 ==========
    
    def register_state(self, state: WeddingState) -> None:
        """
        注册状态到 FSM
        
        Args:
            state: 状态实例
        """
        self.states[state.state_name] = state
        self.logger.info(f"Registered state: {state.state_name}")
    
    def initialize(self, initial_state: WeddingStateName = WeddingStateName.IDLE) -> None:
        """
        初始化 FSM
        
        Args:
            initial_state: 初始状态
        """
        if not self.states:
            raise RuntimeError("No states registered")
        
        if initial_state not in self.states:
            raise ValueError(f"Initial state {initial_state} not registered")
        
        # 设置初始状态
        self.current_state = self.states[initial_state]
        self.next_state = self.current_state
        self.next_state_name = initial_state
        
        # 记录启动时间
        self._start_time = time.time()
        
        # 进入初始状态
        self.current_state.enter_time = self.get_current_time()
        self.current_state.on_enter()
        
        # 设置操作模式
        self.operating_mode = FSMOperatingMode.NORMAL
        
        self._running = True
        
        self.logger.info(f"FSM initialized with state: {initial_state}")
    
    # ========== 主循环方法 ==========
    
    def run_once(self) -> None:
        """
        执行一次 FSM 主循环
        
        调用频率由外部控制（建议 10-50Hz）
        """
        if not self._running or self.current_state is None:
            return
        
        # 1. 安全前检查
        self._safety_pre_check()
        
        # 2. 根据操作模式执行不同逻辑
        if self.operating_mode == FSMOperatingMode.SAFE_STOP:
            # 安全停止模式：执行安全状态
            self._handle_safe_stop()
        
        elif self.operating_mode == FSMOperatingMode.NORMAL:
            # 正常模式：检查转换 + 执行状态逻辑
            self.next_state_name = self.current_state.check_transition()
            
            if self.next_state_name != self.current_state.state_name:
                # 需要状态转换
                self.operating_mode = FSMOperatingMode.TRANSITIONING
                self.next_state = self._get_state(self.next_state_name)
                self.logger.info(
                    f"Transition initiated: {self.current_state.state_name} -> {self.next_state_name}"
                )
            else:
                # 正常执行当前状态
                self.current_state.run()
        
        elif self.operating_mode == FSMOperatingMode.TRANSITIONING:
            # 转换模式：执行转换逻辑
            self._transition_data = self.current_state.transition()
            
            if self._transition_data.done:
                # 转换完成
                self._complete_transition()
        
        # 3. 安全后检查
        self._safety_post_check()
        
        # 4. 重置单次命令
        self.data.reset_pending_command()
        
        # 5. 迭代计数
        self.iter_count += 1
    
    def _complete_transition(self) -> None:
        """完成状态转换"""
        # 退出当前状态
        old_state_name = self.current_state.state_name
        self.current_state.on_exit()
        
        # 切换状态
        self.current_state = self.next_state
        
        # 进入新状态
        self.current_state.enter_time = self.get_current_time()
        self.current_state.iter_count = 0
        self.current_state.transition_data.reset()
        self.current_state.on_enter()
        
        # 恢复正常模式
        self.operating_mode = FSMOperatingMode.NORMAL
        
        # 触发回调
        self._notify_state_change(old_state_name, self.current_state.state_name)
        
        self.logger.info(f"Transition complete: now in {self.current_state.state_name}")
    
    def _get_state(self, state_name: WeddingStateName) -> Optional[WeddingState]:
        """获取状态实例"""
        if state_name in self.states:
            return self.states[state_name]
        else:
            self.logger.warning(f"State {state_name} not found, returning IDLE")
            return self.states.get(WeddingStateName.IDLE)
    
    # ========== 安全检查 ==========
    
    def _safety_pre_check(self) -> None:
        """运行前安全检查"""
        # 检查目标是否太近
        if self.data.perception.too_close:
            self.data.safety_triggered = True
            self.operating_mode = FSMOperatingMode.SAFE_STOP
            self.logger.warning("Safety triggered: target too close")
    
    def _safety_post_check(self) -> None:
        """运行后安全检查"""
        # 可以添加更多安全检查
        pass
    
    def _handle_safe_stop(self) -> None:
        """处理安全停止"""
        # 切换到安全停止状态
        if self.current_state.state_name != WeddingStateName.SAFE_STOP:
            if WeddingStateName.SAFE_STOP in self.states:
                self.current_state.on_exit()
                self.current_state = self.states[WeddingStateName.SAFE_STOP]
                self.current_state.enter_time = self.get_current_time()
                self.current_state.on_enter()
            else:
                self.logger.error("SAFE_STOP state not registered!")
                return
        
        # 运行 SAFE_STOP 状态逻辑
        self.current_state.run()
        
        # 检查是否可以退出安全停止
        next_state = self.current_state.check_transition()
        if next_state != WeddingStateName.SAFE_STOP:
            # 恢复正常，进行状态转换
            self.next_state_name = next_state
            self.next_state = self._get_state(next_state)
            self.operating_mode = FSMOperatingMode.TRANSITIONING
            self.logger.info(f"Safety recovery: transitioning to {next_state}")
    
    # ========== 对外接口 ==========
    
    def send_command(self, event: WeddingEvent) -> bool:
        """
        发送命令到 FSM
        
        Args:
            event: 命令事件
            
        Returns:
            是否接受命令
        """
        self.data.pending_command = event
        self.logger.debug(f"Command received: {event}")
        return True
    
    def force_transition(self, target_state: WeddingStateName) -> bool:
        """
        强制状态转换（外部命令）
        
        Args:
            target_state: 目标状态
            
        Returns:
            是否成功
        """
        if target_state not in self.states:
            self.logger.error(f"State {target_state} not registered")
            return False
        
        # 记录旧状态
        old_state_name = self.current_state.state_name if self.current_state else WeddingStateName.INVALID
        
        # 退出当前状态
        if self.current_state:
            self.current_state.on_exit()
        
        # 切换状态
        self.current_state = self.states[target_state]
        self.current_state.enter_time = self.get_current_time()
        self.current_state.iter_count = 0
        self.current_state.on_enter()
        
        # 恢复正常模式
        self.operating_mode = FSMOperatingMode.NORMAL
        
        # 触发回调
        self._notify_state_change(old_state_name, target_state)
        
        self.logger.info(f"Force transition to: {target_state}")
        return True
    
    def reset(self) -> None:
        """重置 FSM 到初始状态"""
        self.data.safety_triggered = False
        self.force_transition(WeddingStateName.IDLE)
        self.logger.info("FSM reset")
    
    def stop(self) -> None:
        """停止 FSM"""
        self._running = False
        if self.current_state:
            self.current_state.on_exit()
        self.logger.info("FSM stopped")
    
    # ========== 状态查询 ==========
    
    def get_current_state_name(self) -> WeddingStateName:
        """获取当前状态名称"""
        if self.current_state:
            return self.current_state.state_name
        return WeddingStateName.INVALID
    
    def get_operating_mode(self) -> FSMOperatingMode:
        """获取当前操作模式"""
        return self.operating_mode
    
    def get_current_time(self) -> float:
        """获取 FSM 运行时间（秒）"""
        return time.time() - self._start_time
    
    def is_running(self) -> bool:
        """FSM 是否在运行"""
        return self._running
    
    def get_state_elapsed_time(self) -> float:
        """获取当前状态的持续时间"""
        if self.current_state:
            return self.current_state.get_elapsed_time()
        return 0.0
    
    # ========== 回调注册 ==========
    
    def on_state_change(self, callback: Callable[[WeddingStateName, WeddingStateName], None]) -> None:
        """
        注册状态变化回调
        
        Args:
            callback: 回调函数，参数为 (old_state, new_state)
        """
        self._on_state_change_callbacks.append(callback)
    
    def _notify_state_change(self, old_state: WeddingStateName, new_state: WeddingStateName) -> None:
        """通知状态变化"""
        for callback in self._on_state_change_callbacks:
            try:
                callback(old_state, new_state)
            except Exception as e:
                self.logger.error(f"State change callback error: {e}")
    
    # ========== 调试信息 ==========
    
    def get_status_info(self) -> Dict:
        """获取 FSM 状态信息（用于调试）"""
        return {
            'running': self._running,
            'current_state': str(self.get_current_state_name()),
            'operating_mode': str(self.operating_mode),
            'iter_count': self.iter_count,
            'state_elapsed_time': self.get_state_elapsed_time(),
            'safety_triggered': self.data.safety_triggered,
        }

