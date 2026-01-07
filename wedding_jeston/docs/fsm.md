# 婚礼互动机器人状态机设计文档

**文档版本**：V1.0  
**适用项目**：众擎 PM01 婚礼互动机器人  
**开发框架**：ROS2 Humble + Python  
**参考实现**：`engineai_humanoid/EngineAI_Controller/FSM_States/`

---

## 1. 概述

### 1.1 设计目标

为婚礼互动机器人设计一套**可扩展、可测试、易于调试**的有限状态机（FSM）框架，用于管理机器人的互动行为。

### 1.2 设计原则

| 原则 | 说明 |
|------|------|
| **单一职责** | 每个状态类只负责自己的行为逻辑 |
| **开闭原则** | 新增状态不需要修改 FSM 核心代码 |
| **接口清晰** | 状态生命周期方法统一，便于理解和测试 |
| **可观测性** | 状态转换、当前状态、事件均可通过 ROS2 接口暴露 |
| **安全优先** | 任何异常情况都能安全回退到初始状态 |

### 1.3 参考 EngineAI FSM 设计

借鉴 `engineai_humanoid` 中的 FSM 设计模式：

```
engineai_humanoid/EngineAI_Controller/user/EngineAI_Humanoid_Controller/FSM_States/
├── ControlFSM.h/cpp          # FSM 控制器
├── ControlFSMData.h          # FSM 共享数据
├── FSM_State.h/cpp           # 状态基类
├── FSM_State_Passive.h/cpp   # 具体状态实现
├── FSM_State_JointPD.h/cpp   # 具体状态实现
├── SafetyChecker.h/cpp       # 安全检查器
└── TransitionData.h          # 状态转换数据
```

---

## 2. 状态定义

### 2.1 状态枚举（MVP 版本）

```python
from enum import Enum, auto

class WeddingStateName(Enum):
    """婚礼互动状态枚举"""
    
    # 核心状态（MVP）
    IDLE = auto()           # 空闲：待机微动，等待目标
    TRACKING = auto()       # 跟随：头/腰跟随目标
    PHOTO_POSING = auto()   # 合影：执行 Pose + 倒数
    FAREWELL = auto()       # 送别：挥手 + 送别语
    
    # 扩展状态（后续迭代）
    # SEARCHING = auto()        # 找寻目标
    # PASSIVE_TRACKING = auto() # 单纯跟随
    # ACTIVE_TRACKING = auto()  # 互动跟随
    # INTERVIEW = auto()        # 采访模式
    
    # 系统状态
    SAFE_STOP = auto()      # 安全停止：异常时进入
    INVALID = auto()        # 无效状态

    def __str__(self):
        return self.name
```

### 2.2 操作模式枚举

```python
class FSMOperatingMode(Enum):
    """FSM 操作模式"""
    NORMAL = auto()         # 正常运行
    TRANSITIONING = auto()  # 状态转换中
    SAFE_STOP = auto()      # 安全停止
```

### 2.3 状态转换事件枚举

```python
class WeddingEvent(Enum):
    """状态转换事件"""
    
    # 感知事件
    FACE_DETECTED = auto()      # 检测到人脸
    FACE_LOST = auto()          # 人脸丢失
    GESTURE_HEART = auto()      # 检测到比心手势
    GESTURE_V = auto()          # 检测到 V 手势
    TOO_CLOSE = auto()          # 目标太近
    TARGET_LEAVING = auto()     # 目标离开
    
    # 命令事件
    CMD_GOTO_IDLE = auto()      # 外部命令：回到空闲
    CMD_GOTO_PHOTO = auto()     # 外部命令：进入合影
    CMD_STOP = auto()           # 外部命令：停止
    
    # 内部事件
    TIMEOUT = auto()            # 超时
    POSE_COMPLETE = auto()      # Pose 完成
    FAREWELL_COMPLETE = auto()  # 送别完成
    
    # 安全事件
    SAFETY_TRIGGER = auto()     # 安全触发
```

---

## 3. 状态基类设计

### 3.1 状态基类（WeddingState）

```python
from abc import ABC, abstractmethod
from typing import Optional, TYPE_CHECKING
from dataclasses import dataclass

if TYPE_CHECKING:
    from .wedding_fsm import WeddingFSM

@dataclass
class TransitionData:
    """状态转换数据"""
    done: bool = False              # 转换是否完成
    next_state: Optional[WeddingStateName] = None  # 下一个状态
    duration: float = 0.0           # 转换持续时间


class WeddingState(ABC):
    """
    婚礼互动状态基类
    
    生命周期方法：
    - on_enter(): 进入状态时调用
    - run(): 每个控制周期调用
    - check_transition(): 检查是否需要状态转换
    - transition(): 执行状态转换逻辑
    - on_exit(): 退出状态时调用
    
    参考：engineai_humanoid/FSM_States/FSM_State.h
    """
    
    def __init__(self, fsm: 'WeddingFSM', state_name: WeddingStateName):
        """
        初始化状态
        
        Args:
            fsm: FSM 控制器引用，用于访问共享数据
            state_name: 状态名称枚举
        """
        self.fsm = fsm
        self.state_name = state_name
        self.state_string = str(state_name)
        
        # 状态转换相关
        self.next_state_name: WeddingStateName = state_name
        self.transition_data = TransitionData()
        
        # 状态内部计数器
        self.iter_count = 0
        self.enter_time: float = 0.0
        
        # 安全检查标志
        self.check_safe_distance = True
        
    @abstractmethod
    def on_enter(self) -> None:
        """
        进入状态时的初始化逻辑
        - 重置状态内部变量
        - 设置初始动作/语音
        - 记录进入时间
        """
        pass
    
    @abstractmethod
    def run(self) -> None:
        """
        状态的主循环逻辑
        - 每个控制周期调用一次
        - 执行状态特定行为（跟随、Pose 等）
        """
        pass
    
    def check_transition(self) -> WeddingStateName:
        """
        检查是否需要状态转换
        
        Returns:
            下一个状态名称（如果不需要转换，返回当前状态名称）
        
        默认实现：检查外部命令和安全事件
        子类可以覆盖此方法添加状态特定的转换逻辑
        """
        # 检查安全事件（最高优先级）
        if self.fsm.data.safety_triggered:
            return WeddingStateName.SAFE_STOP
        
        # 检查外部命令
        cmd = self.fsm.data.pending_command
        if cmd == WeddingEvent.CMD_STOP:
            return WeddingStateName.IDLE
        elif cmd == WeddingEvent.CMD_GOTO_IDLE:
            return WeddingStateName.IDLE
        elif cmd == WeddingEvent.CMD_GOTO_PHOTO:
            return WeddingStateName.PHOTO_POSING
        
        # 默认：保持当前状态
        return self.state_name
    
    def transition(self) -> TransitionData:
        """
        执行状态转换逻辑
        
        Returns:
            TransitionData 包含转换是否完成等信息
        
        默认实现：立即完成转换
        子类可以覆盖此方法实现渐进式转换（如平滑过渡动作）
        """
        self.transition_data.done = True
        return self.transition_data
    
    @abstractmethod
    def on_exit(self) -> None:
        """
        退出状态时的清理逻辑
        - 停止状态特定行为
        - 重置状态变量
        """
        pass
    
    # ========== 辅助方法 ==========
    
    def get_elapsed_time(self) -> float:
        """获取在当前状态的持续时间（秒）"""
        return self.fsm.get_current_time() - self.enter_time
    
    def log(self, message: str) -> None:
        """状态日志输出"""
        self.fsm.log(f"[{self.state_string}] {message}")
```

### 3.2 状态生命周期

```
                    ┌─────────────────────────────────────────┐
                    │              状态生命周期                │
                    └─────────────────────────────────────────┘
                                      │
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │              on_enter()                 │
                    │  • 初始化状态变量                        │
                    │  • 设置初始动作/语音                     │
                    │  • 记录进入时间                          │
                    └─────────────────────────────────────────┘
                                      │
                                      ▼
        ┌─────────────────────────────────────────────────────────────┐
        │                       主循环                                 │
        │  ┌─────────────────────────────────────────────────────┐   │
        │  │                    run()                             │   │
        │  │  • 执行状态特定行为                                   │   │
        │  │  • 更新动作指令                                       │   │
        │  └─────────────────────────────────────────────────────┘   │
        │                           │                                 │
        │                           ▼                                 │
        │  ┌─────────────────────────────────────────────────────┐   │
        │  │              check_transition()                      │   │
        │  │  • 检查感知事件                                       │   │
        │  │  • 检查外部命令                                       │   │
        │  │  • 检查超时                                           │   │
        │  └─────────────────────────────────────────────────────┘   │
        │                           │                                 │
        │              需要转换？    │                                 │
        │                  ├────────┴────────┐                       │
        │                  │ 否              │ 是                     │
        │                  ▼                 │                        │
        │            继续 run() ◀────────────┘                       │
        └─────────────────────────────────────────────────────────────┘
                                      │ 需要转换
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │              transition()               │
                    │  • 执行渐进式转换（可选）                │
                    │  • 返回 done=True 时完成                 │
                    └─────────────────────────────────────────┘
                                      │ done=True
                                      ▼
                    ┌─────────────────────────────────────────┐
                    │              on_exit()                  │
                    │  • 清理状态资源                          │
                    │  • 停止状态行为                          │
                    └─────────────────────────────────────────┘
                                      │
                                      ▼
                              进入下一个状态的 on_enter()
```

---

## 4. FSM 控制器设计

### 4.1 FSM 共享数据（WeddingFSMData）

```python
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
import numpy as np

@dataclass
class PerceptionData:
    """感知数据"""
    # 目标检测
    face_detected: bool = False
    face_position: np.ndarray = field(default_factory=lambda: np.zeros(2))  # 归一化 (x, y)
    face_distance: float = 0.0  # 估计距离 (m)
    face_count: int = 0
    
    # 手势检测
    gesture: str = "none"  # "none", "heart", "v", "ok"
    gesture_confidence: float = 0.0
    
    # 安全检测
    too_close: bool = False


@dataclass
class MotionData:
    """运动数据"""
    # 当前关节状态
    joint_positions: np.ndarray = field(default_factory=lambda: np.zeros(24))
    
    # 目标关节命令
    joint_commands: np.ndarray = field(default_factory=lambda: np.zeros(24))
    
    # 当前 Pose 名称
    current_pose: str = "neutral"
    target_pose: str = "neutral"
    
    # 注视目标
    look_at_target: np.ndarray = field(default_factory=lambda: np.zeros(2))  # 归一化 (x, y)


@dataclass
class AudioData:
    """音频数据"""
    # 语音命令
    voice_command: str = "none"  # "none", "photo", "interview", "stop"
    
    # 当前播放状态
    is_playing: bool = False
    current_speech: str = ""


@dataclass
class WeddingFSMData:
    """
    FSM 共享数据结构
    
    包含所有状态需要访问的共享数据
    参考：engineai_humanoid/FSM_States/ControlFSMData.h
    """
    # 感知数据
    perception: PerceptionData = field(default_factory=PerceptionData)
    
    # 运动数据
    motion: MotionData = field(default_factory=MotionData)
    
    # 音频数据
    audio: AudioData = field(default_factory=AudioData)
    
    # 待处理命令（外部输入）
    pending_command: Optional[WeddingEvent] = None
    
    # 安全标志
    safety_triggered: bool = False
    
    # 配置参数
    config: Dict[str, Any] = field(default_factory=dict)
    
    def reset_pending_command(self):
        """重置待处理命令"""
        self.pending_command = None
```

### 4.2 FSM 控制器（WeddingFSM）

```python
import time
from typing import Dict, Optional, Callable
import logging

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
    
    def __init__(self, config: Dict = None):
        """
        初始化 FSM
        
        Args:
            config: 配置参数字典
        """
        self.logger = logging.getLogger("WeddingFSM")
        
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
        
        # 进入初始状态
        self.current_state.on_enter()
        
        # 设置操作模式
        self.operating_mode = FSMOperatingMode.NORMAL
        
        # 记录启动时间
        self._start_time = time.time()
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
                    f"Transition: {self.current_state.state_name} -> {self.next_state_name}"
                )
            else:
                # 正常执行当前状态
                self.current_state.run()
        
        elif self.operating_mode == FSMOperatingMode.TRANSITIONING:
            # 转换模式：执行转换逻辑
            transition_data = self.current_state.transition()
            
            if transition_data.done:
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
        self.current_state.on_exit()
        
        # 切换状态
        old_state = self.current_state
        self.current_state = self.next_state
        
        # 进入新状态
        self.current_state.enter_time = self.get_current_time()
        self.current_state.on_enter()
        
        # 恢复正常模式
        self.operating_mode = FSMOperatingMode.NORMAL
        
        # 触发回调
        self._notify_state_change(old_state.state_name, self.current_state.state_name)
        
        self.logger.info(f"Transition complete: now in {self.current_state.state_name}")
    
    def _get_state(self, state_name: WeddingStateName) -> WeddingState:
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
                self.current_state.on_enter()
    
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
        self.logger.info(f"Command received: {event}")
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
        
        # 立即转换
        self.current_state.on_exit()
        self.current_state = self.states[target_state]
        self.current_state.enter_time = self.get_current_time()
        self.current_state.on_enter()
        
        self.operating_mode = FSMOperatingMode.NORMAL
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
    
    # ========== 日志 ==========
    
    def log(self, message: str) -> None:
        """FSM 日志"""
        self.logger.info(message)
```

---

## 5. 具体状态实现

### 5.1 IDLE 状态（空闲）

```python
class IdleState(WeddingState):
    """
    空闲状态
    
    行为：
    - 头部/腰部待机微动
    - 等待目标出现
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.IDLE)
        
        # 待机动作参数
        self.sway_amplitude = 0.035  # ±2°
        self.sway_period = 2.5  # 秒
        
    def on_enter(self) -> None:
        self.log("Entering IDLE state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 设置中立位
        self.fsm.data.motion.target_pose = "neutral"
        
    def run(self) -> None:
        """执行待机微动"""
        elapsed = self.get_elapsed_time()
        
        # 计算待机摆动角度（正弦波）
        import math
        sway_angle = self.sway_amplitude * math.sin(2 * math.pi * elapsed / self.sway_period)
        
        # 设置头部/腰部微动
        self.fsm.data.motion.look_at_target[0] = 0.5 + sway_angle  # 中心 + 偏移
        
        self.iter_count += 1
    
    def check_transition(self) -> WeddingStateName:
        # 先检查基类的转换逻辑（安全/命令）
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 检测到人脸 -> 进入跟随
        if self.fsm.data.perception.face_detected:
            return WeddingStateName.TRACKING
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting IDLE state")
```

### 5.2 TRACKING 状态（跟随）

```python
class TrackingState(WeddingState):
    """
    跟随状态
    
    行为：
    - 头部/腰部跟随目标位置
    - 进入时播放问候语
    - 检测手势触发合影
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.TRACKING)
        
        # 跟随参数
        self.follow_smooth = 0.3  # 平滑系数
        
        # 问候控制
        self.greeted = False
        self.lost_time = 0.0
        self.lost_timeout = 1.5  # 丢失超时（秒）
        
    def on_enter(self) -> None:
        self.log("Entering TRACKING state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        self.greeted = False
        self.lost_time = 0.0
        
        # 播放问候语
        self._play_greeting()
        
    def _play_greeting(self) -> None:
        """播放问候语"""
        # TODO: 实际实现语音播放
        self.fsm.data.audio.current_speech = "greeting"
        self.greeted = True
        self.log("Playing greeting")
    
    def run(self) -> None:
        """执行跟随逻辑"""
        perception = self.fsm.data.perception
        motion = self.fsm.data.motion
        
        if perception.face_detected:
            # 平滑跟随目标位置
            target_x = perception.face_position[0]
            target_y = perception.face_position[1]
            
            # 低通滤波
            motion.look_at_target[0] = (
                motion.look_at_target[0] * (1 - self.follow_smooth) +
                target_x * self.follow_smooth
            )
            motion.look_at_target[1] = (
                motion.look_at_target[1] * (1 - self.follow_smooth) +
                target_y * self.follow_smooth
            )
            
            # 重置丢失计时
            self.lost_time = 0.0
        else:
            # 目标丢失，开始计时
            self.lost_time += 0.02  # 假设 50Hz
        
        self.iter_count += 1
    
    def check_transition(self) -> WeddingStateName:
        # 先检查基类的转换逻辑
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        perception = self.fsm.data.perception
        
        # 检测到合影手势 -> 进入合影
        if perception.gesture in ["heart", "v"]:
            if perception.gesture_confidence > 0.7:
                return WeddingStateName.PHOTO_POSING
        
        # 目标丢失超时 -> 送别
        if self.lost_time > self.lost_timeout:
            return WeddingStateName.FAREWELL
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting TRACKING state")
```

### 5.3 PHOTO_POSING 状态（合影）

```python
class PhotoPosingState(WeddingState):
    """
    合影状态
    
    行为：
    - 执行 Pose 动作
    - 倒数 3-2-1
    - 播放快门声
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.PHOTO_POSING)
        
        # 合影流程阶段
        self.phase = "confirm"  # confirm, pose, countdown, shutter, done
        self.phase_start_time = 0.0
        
        # 时间参数
        self.confirm_duration = 1.0  # 确认语时长
        self.pose_duration = 1.5     # Pose 准备时长
        self.countdown_duration = 3.0  # 倒数时长
        
    def on_enter(self) -> None:
        self.log("Entering PHOTO_POSING state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 开始确认阶段
        self.phase = "confirm"
        self.phase_start_time = self.enter_time
        
        # 播放确认语
        self.fsm.data.audio.current_speech = "photo_confirm"
        
        # 根据手势选择 Pose
        gesture = self.fsm.data.perception.gesture
        if gesture == "heart":
            self.fsm.data.motion.target_pose = "heart"
        elif gesture == "v":
            self.fsm.data.motion.target_pose = "v_sign"
        else:
            self.fsm.data.motion.target_pose = "thumbs_up"
    
    def run(self) -> None:
        """执行合影流程"""
        elapsed = self.get_elapsed_time()
        phase_elapsed = elapsed - (self.phase_start_time - self.enter_time)
        
        if self.phase == "confirm":
            if phase_elapsed > self.confirm_duration:
                self.phase = "pose"
                self.phase_start_time = self.fsm.get_current_time()
                self.log("Phase: pose")
                
        elif self.phase == "pose":
            # 执行 Pose 动作
            if phase_elapsed > self.pose_duration:
                self.phase = "countdown"
                self.phase_start_time = self.fsm.get_current_time()
                self.fsm.data.audio.current_speech = "countdown"
                self.log("Phase: countdown")
                
        elif self.phase == "countdown":
            if phase_elapsed > self.countdown_duration:
                self.phase = "shutter"
                self.phase_start_time = self.fsm.get_current_time()
                self.fsm.data.audio.current_speech = "shutter"
                self.log("Phase: shutter")
                
        elif self.phase == "shutter":
            if phase_elapsed > 0.5:  # 快门音效时长
                self.phase = "done"
                self.fsm.data.audio.current_speech = "photo_done"
                self.log("Phase: done")
        
        self.iter_count += 1
    
    def check_transition(self) -> WeddingStateName:
        # 先检查基类的转换逻辑
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 合影完成 -> 回到跟随
        if self.phase == "done" and self.get_elapsed_time() > 6.0:
            return WeddingStateName.TRACKING
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting PHOTO_POSING state")
        # 回到中立位
        self.fsm.data.motion.target_pose = "neutral"
```

### 5.4 FAREWELL 状态（送别）

```python
class FarewellState(WeddingState):
    """
    送别状态
    
    行为：
    - 挥手动作
    - 播放送别语
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.FAREWELL)
        
        self.farewell_duration = 2.0  # 送别持续时间
        
    def on_enter(self) -> None:
        self.log("Entering FAREWELL state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 播放送别语
        self.fsm.data.audio.current_speech = "farewell"
        
        # 执行挥手动作
        self.fsm.data.motion.target_pose = "wave"
    
    def run(self) -> None:
        """执行送别"""
        self.iter_count += 1
    
    def check_transition(self) -> WeddingStateName:
        # 先检查基类的转换逻辑
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 送别完成 -> 回到空闲
        if self.get_elapsed_time() > self.farewell_duration:
            return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting FAREWELL state")
        # 回到中立位
        self.fsm.data.motion.target_pose = "neutral"
```

### 5.5 SAFE_STOP 状态（安全停止）

```python
class SafeStopState(WeddingState):
    """
    安全停止状态
    
    行为：
    - 停止所有动作
    - 回到中立位
    - 播放提示语
    """
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.SAFE_STOP)
        
    def on_enter(self) -> None:
        self.log("Entering SAFE_STOP state")
        self.enter_time = self.fsm.get_current_time()
        
        # 回到中立位
        self.fsm.data.motion.target_pose = "neutral"
        
        # 播放提示语
        self.fsm.data.audio.current_speech = "too_close"
    
    def run(self) -> None:
        """保持安全停止"""
        # 检查是否可以恢复
        if not self.fsm.data.perception.too_close:
            if self.get_elapsed_time() > 2.0:  # 等待 2 秒
                self.fsm.data.safety_triggered = False
    
    def check_transition(self) -> WeddingStateName:
        # 安全条件解除 -> 回到空闲
        if not self.fsm.data.safety_triggered:
            return WeddingStateName.IDLE
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting SAFE_STOP state")
```

---

## 6. ROS2 接口设计

### 6.1 FSM 节点（WeddingFSMNode）

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger, SetBool

class WeddingFSMNode(Node):
    """
    婚礼互动 FSM ROS2 节点
    
    职责：
    - 封装 WeddingFSM
    - 订阅感知数据
    - 发布状态和动作指令
    - 提供服务接口
    """
    
    def __init__(self):
        super().__init__('wedding_fsm_node')
        
        # 创建 FSM
        self.fsm = WeddingFSM()
        
        # 注册状态
        self.fsm.register_state(IdleState(self.fsm))
        self.fsm.register_state(TrackingState(self.fsm))
        self.fsm.register_state(PhotoPosingState(self.fsm))
        self.fsm.register_state(FarewellState(self.fsm))
        self.fsm.register_state(SafeStopState(self.fsm))
        
        # 初始化 FSM
        self.fsm.initialize(WeddingStateName.IDLE)
        
        # ========== 订阅者 ==========
        
        # 感知数据
        self.target_sub = self.create_subscription(
            PointStamped,
            '/wedding/perception/target',
            self._on_target,
            10
        )
        
        self.gesture_sub = self.create_subscription(
            String,
            '/wedding/perception/gesture',
            self._on_gesture,
            10
        )
        
        self.too_close_sub = self.create_subscription(
            Bool,
            '/wedding/perception/too_close',
            self._on_too_close,
            10
        )
        
        # 外部命令
        self.command_sub = self.create_subscription(
            String,
            '/wedding/fsm/command',
            self._on_command,
            10
        )
        
        # ========== 发布者 ==========
        
        # FSM 状态
        self.state_pub = self.create_publisher(
            String,
            '/wedding/fsm/state',
            10
        )
        
        # 动作指令
        self.pose_pub = self.create_publisher(
            String,
            '/wedding/motion/pose',
            10
        )
        
        self.look_at_pub = self.create_publisher(
            PointStamped,
            '/wedding/motion/look_at',
            10
        )
        
        # 语音指令
        self.speech_pub = self.create_publisher(
            String,
            '/wedding/audio/play',
            10
        )
        
        # ========== 服务 ==========
        
        self.reset_srv = self.create_service(
            Trigger,
            '/wedding/fsm/reset',
            self._on_reset
        )
        
        self.set_state_srv = self.create_service(
            SetBool,  # 简化版，实际可用自定义 srv
            '/wedding/fsm/set_idle',
            self._on_set_idle
        )
        
        # ========== 定时器 ==========
        
        # FSM 主循环（50Hz）
        self.timer = self.create_timer(0.02, self._run_fsm)
        
        # 状态发布（5Hz）
        self.state_timer = self.create_timer(0.2, self._publish_state)
        
        # 注册状态变化回调
        self.fsm.on_state_change(self._on_state_change)
        
        self.get_logger().info("WeddingFSMNode initialized")
    
    # ========== 回调函数 ==========
    
    def _on_target(self, msg: PointStamped) -> None:
        """处理目标位置"""
        self.fsm.data.perception.face_detected = True
        self.fsm.data.perception.face_position[0] = msg.point.x
        self.fsm.data.perception.face_position[1] = msg.point.y
        self.fsm.data.perception.face_distance = msg.point.z
    
    def _on_gesture(self, msg: String) -> None:
        """处理手势"""
        self.fsm.data.perception.gesture = msg.data
        if msg.data != "none":
            self.fsm.data.perception.gesture_confidence = 0.8
    
    def _on_too_close(self, msg: Bool) -> None:
        """处理太近检测"""
        self.fsm.data.perception.too_close = msg.data
    
    def _on_command(self, msg: String) -> None:
        """处理外部命令"""
        cmd = msg.data.lower()
        if cmd == "stop":
            self.fsm.send_command(WeddingEvent.CMD_STOP)
        elif cmd == "photo":
            self.fsm.send_command(WeddingEvent.CMD_GOTO_PHOTO)
        elif cmd == "idle":
            self.fsm.send_command(WeddingEvent.CMD_GOTO_IDLE)
    
    def _on_reset(self, request, response) -> Trigger.Response:
        """重置 FSM"""
        self.fsm.reset()
        response.success = True
        response.message = "FSM reset"
        return response
    
    def _on_set_idle(self, request, response) -> SetBool.Response:
        """设置为空闲状态"""
        self.fsm.force_transition(WeddingStateName.IDLE)
        response.success = True
        response.message = "Set to IDLE"
        return response
    
    # ========== 主循环 ==========
    
    def _run_fsm(self) -> None:
        """FSM 主循环"""
        self.fsm.run_once()
        
        # 发布动作指令
        self._publish_motion()
        
        # 发布语音指令
        self._publish_speech()
        
        # 重置单次感知数据
        self._reset_perception()
    
    def _publish_state(self) -> None:
        """发布 FSM 状态"""
        msg = String()
        msg.data = str(self.fsm.get_current_state_name())
        self.state_pub.publish(msg)
    
    def _publish_motion(self) -> None:
        """发布动作指令"""
        motion = self.fsm.data.motion
        
        # 发布 Pose
        if motion.target_pose != motion.current_pose:
            msg = String()
            msg.data = motion.target_pose
            self.pose_pub.publish(msg)
            motion.current_pose = motion.target_pose
        
        # 发布 LookAt
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = motion.look_at_target[0]
        msg.point.y = motion.look_at_target[1]
        self.look_at_pub.publish(msg)
    
    def _publish_speech(self) -> None:
        """发布语音指令"""
        audio = self.fsm.data.audio
        if audio.current_speech:
            msg = String()
            msg.data = audio.current_speech
            self.speech_pub.publish(msg)
            audio.current_speech = ""  # 清空，只发一次
    
    def _reset_perception(self) -> None:
        """重置单次感知数据"""
        self.fsm.data.perception.face_detected = False
        self.fsm.data.perception.gesture = "none"
        self.fsm.data.perception.gesture_confidence = 0.0
    
    def _on_state_change(self, old_state: WeddingStateName, new_state: WeddingStateName) -> None:
        """状态变化回调"""
        self.get_logger().info(f"State changed: {old_state} -> {new_state}")


def main(args=None):
    rclpy.init(args=args)
    node = WeddingFSMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fsm.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 7. 对外接口汇总

### 7.1 Topic 接口

| Topic | 类型 | 方向 | 描述 |
|-------|------|------|------|
| `/wedding/perception/target` | `PointStamped` | Sub | 目标位置 |
| `/wedding/perception/gesture` | `String` | Sub | 手势类型 |
| `/wedding/perception/too_close` | `Bool` | Sub | 太近标志 |
| `/wedding/fsm/command` | `String` | Sub | 外部命令 |
| `/wedding/fsm/state` | `String` | Pub | 当前状态 |
| `/wedding/motion/pose` | `String` | Pub | 目标 Pose |
| `/wedding/motion/look_at` | `PointStamped` | Pub | 注视目标 |
| `/wedding/audio/play` | `String` | Pub | 语音资源名 |

### 7.2 Service 接口

| Service | 类型 | 描述 |
|---------|------|------|
| `/wedding/fsm/reset` | `Trigger` | 重置 FSM |
| `/wedding/fsm/set_idle` | `SetBool` | 强制回到 IDLE |

### 7.3 外部命令格式

| 命令 | 效果 |
|------|------|
| `"stop"` | 停止，回到 IDLE |
| `"photo"` | 进入合影状态 |
| `"idle"` | 强制回到 IDLE |

---

## 8. 文件结构

```
wedding_interaction/
├── wedding_interaction/
│   ├── __init__.py
│   ├── fsm/
│   │   ├── __init__.py
│   │   ├── wedding_state.py      # 状态基类
│   │   ├── wedding_fsm.py        # FSM 控制器
│   │   ├── wedding_fsm_data.py   # FSM 数据结构
│   │   ├── states/
│   │   │   ├── __init__.py
│   │   │   ├── idle_state.py
│   │   │   ├── tracking_state.py
│   │   │   ├── photo_posing_state.py
│   │   │   ├── farewell_state.py
│   │   │   └── safe_stop_state.py
│   │   └── events.py             # 事件枚举
│   └── nodes/
│       ├── __init__.py
│       └── wedding_fsm_node.py   # ROS2 节点
├── launch/
│   └── wedding_fsm.launch.py
├── config/
│   └── fsm_config.yaml
├── package.xml
├── setup.py
└── README.md
```

---

## 9. 测试方案

### 9.1 单元测试

```python
import pytest
from wedding_interaction.fsm import WeddingFSM, WeddingStateName

def test_fsm_initialization():
    """测试 FSM 初始化"""
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.initialize(WeddingStateName.IDLE)
    
    assert fsm.get_current_state_name() == WeddingStateName.IDLE
    assert fsm.is_running()

def test_idle_to_tracking_transition():
    """测试 IDLE -> TRACKING 转换"""
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.register_state(TrackingState(fsm))
    fsm.initialize(WeddingStateName.IDLE)
    
    # 模拟检测到人脸
    fsm.data.perception.face_detected = True
    fsm.data.perception.face_position = [0.5, 0.5]
    
    # 运行一次
    fsm.run_once()
    
    # 应该转换到 TRACKING
    assert fsm.get_current_state_name() == WeddingStateName.TRACKING

def test_force_transition():
    """测试强制状态转换"""
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.register_state(PhotoPosingState(fsm))
    fsm.initialize(WeddingStateName.IDLE)
    
    # 强制转换到 PHOTO_POSING
    fsm.force_transition(WeddingStateName.PHOTO_POSING)
    
    assert fsm.get_current_state_name() == WeddingStateName.PHOTO_POSING

def test_safety_trigger():
    """测试安全触发"""
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.register_state(SafeStopState(fsm))
    fsm.initialize(WeddingStateName.IDLE)
    
    # 触发安全
    fsm.data.perception.too_close = True
    
    # 运行一次
    fsm.run_once()
    
    # 应该进入安全停止
    assert fsm.get_current_state_name() == WeddingStateName.SAFE_STOP
```

### 9.2 集成测试

```bash
# 启动 FSM 节点
ros2 run wedding_interaction wedding_fsm_node

# 另一个终端，发布测试数据
ros2 topic pub /wedding/perception/target geometry_msgs/PointStamped \
  "{header: {stamp: {sec: 0}, frame_id: 'camera'}, point: {x: 0.5, y: 0.5, z: 1.5}}"

# 查看状态
ros2 topic echo /wedding/fsm/state

# 发送命令
ros2 topic pub /wedding/fsm/command std_msgs/String "data: 'photo'"
```

---

## 10. 后续扩展

### 10.1 新增状态

1. 创建新状态类，继承 `WeddingState`
2. 实现 `on_enter()`, `run()`, `check_transition()`, `on_exit()`
3. 在 `WeddingStateName` 枚举中添加新状态
4. 在 FSM 节点中注册新状态

### 10.2 新增事件

1. 在 `WeddingEvent` 枚举中添加新事件
2. 在相关状态的 `check_transition()` 中处理新事件

### 10.3 状态持久化

可以添加状态序列化功能，用于调试和回放。

