"""
FSM 枚举定义

包含状态名称、操作模式、事件等枚举
"""

from enum import Enum, auto


class WeddingStateName(Enum):
    """婚礼互动状态枚举"""
    
    # 核心状态
    IDLE = auto()           # 空闲：待机微动，等待目标
    SEARCH = auto()         # 搜寻：检测并锁定目标，确认互动意向
    TRACKING = auto()       # 跟随：头/腰跟随目标 + 自由沟通
    PHOTO_POSING = auto()   # 合影：执行 Pose + 倒数
    FAREWELL = auto()       # 送别：挥手 + 送别语
    
    # 扩展状态（后续迭代）
    INTERVIEW = auto()      # 采访：录音录像 + 采访问答
    
    # 系统状态
    SAFE_STOP = auto()      # 安全停止：异常时进入
    INVALID = auto()        # 无效状态

    def __str__(self) -> str:
        return self.name


class FSMOperatingMode(Enum):
    """FSM 操作模式"""
    NORMAL = auto()         # 正常运行
    TRANSITIONING = auto()  # 状态转换中
    SAFE_STOP = auto()      # 安全停止


class WeddingEvent(Enum):
    """状态转换事件"""
    
    # 感知事件
    FACE_DETECTED = auto()      # 检测到人脸
    FACE_LOST = auto()          # 人脸丢失
    TARGET_CONFIRMED = auto()   # 目标确认（正脸/人体持续足够时间）
    TARGET_LOST = auto()        # 目标丢失（人脸+人体都丢失超时）
    GESTURE_HEART = auto()      # 检测到比心手势
    GESTURE_V = auto()          # 检测到 V 手势
    TOO_CLOSE = auto()          # 目标太近
    TARGET_LEAVING = auto()     # 目标离开
    
    # 命令事件
    CMD_GOTO_IDLE = auto()      # 外部命令：回到空闲
    CMD_GOTO_SEARCH = auto()    # 外部命令：进入搜寻
    CMD_GOTO_TRACKING = auto()  # 外部命令：进入跟随
    CMD_GOTO_PHOTO = auto()     # 外部命令：进入合影
    CMD_START_INTERVIEW = auto() # 外部命令：开始采访
    CMD_STOP = auto()           # 外部命令：停止
    
    # 内部事件
    TIMEOUT = auto()            # 超时
    SEARCH_TIMEOUT = auto()     # 搜寻超时
    POSE_COMPLETE = auto()      # Pose 完成
    FAREWELL_COMPLETE = auto()  # 送别完成
    INTERVIEW_COMPLETE = auto() # 采访完成
    
    # 安全事件
    SAFETY_TRIGGER = auto()     # 安全触发
    SAFETY_CLEAR = auto()       # 安全解除

