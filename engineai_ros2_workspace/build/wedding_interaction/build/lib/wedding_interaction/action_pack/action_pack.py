"""
动作包定义

动作包 = 动作 + 语音的组合
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Dict, Any
import numpy as np


class ActionType(Enum):
    """动作类型枚举"""
    # 待机动作
    IDLE_SWAY = auto()          # 待机摆动
    IDLE_NOD = auto()           # 待机点头（用 Yaw 模拟）
    
    # 互动动作
    GREETING_WAVE = auto()      # 问候挥手
    GREETING_BOW = auto()       # 问候鞠躬
    
    # 合影动作
    POSE_HEART = auto()         # 比心
    POSE_V_SIGN = auto()        # V 字手势
    POSE_THUMBS_UP = auto()     # 点赞
    
    # 送别动作
    FAREWELL_WAVE = auto()      # 挥手告别
    
    # 基础动作
    LOOK_LEFT = auto()          # 向左看
    LOOK_RIGHT = auto()         # 向右看
    LOOK_CENTER = auto()        # 看中间
    
    NEUTRAL = auto()            # 中立位


@dataclass
class ActionPack:
    """
    动作包
    
    组合动作和语音，形成完整的交互单元
    """
    
    # 基本信息
    name: str                           # 动作包名称
    action_type: ActionType             # 动作类型
    
    # 动作参数
    head_amplitude: float = 0.0         # 头部摆动幅度（弧度）
    waist_amplitude: float = 0.0        # 腰部摆动幅度（弧度）
    period: float = 2.5                 # 周期（秒）
    duration: float = 0.0               # 持续时间（0 = 持续执行）
    
    # 语音参数
    speech_id: Optional[str] = None     # 语音资源 ID
    speech_text: Optional[str] = None   # 语音文本（用于 TTS）
    speech_delay: float = 0.0           # 语音延迟（秒）
    
    # 额外参数
    extra_params: Dict[str, Any] = field(default_factory=dict)
    
    def has_speech(self) -> bool:
        """是否包含语音"""
        return self.speech_id is not None or self.speech_text is not None
    
    def get_speech(self) -> Optional[str]:
        """获取语音内容"""
        return self.speech_id or self.speech_text
    
    def get_head_offset(self, elapsed_time: float) -> float:
        """
        获取当前头部偏移角度
        
        Args:
            elapsed_time: 动作开始后的经过时间（秒）
            
        Returns:
            头部偏移角度（弧度）
        """
        if self.period <= 0:
            return 0.0
        
        # 正弦波
        phase = 2 * np.pi * elapsed_time / self.period
        return self.head_amplitude * np.sin(phase)
    
    def get_waist_offset(self, elapsed_time: float) -> float:
        """
        获取当前腰部偏移角度
        
        Args:
            elapsed_time: 动作开始后的经过时间（秒）
            
        Returns:
            腰部偏移角度（弧度）
        """
        if self.period <= 0:
            return 0.0
        
        # 正弦波
        phase = 2 * np.pi * elapsed_time / self.period
        return self.waist_amplitude * np.sin(phase)
    
    def is_complete(self, elapsed_time: float) -> bool:
        """
        动作是否完成
        
        Args:
            elapsed_time: 动作开始后的经过时间（秒）
            
        Returns:
            True 如果动作完成（或持续动作永不完成）
        """
        if self.duration <= 0:
            return False  # 持续执行的动作永不完成
        return elapsed_time >= self.duration
    
    def should_play_speech(self, elapsed_time: float, speech_played: bool) -> bool:
        """
        是否应该播放语音
        
        Args:
            elapsed_time: 动作开始后的经过时间（秒）
            speech_played: 语音是否已播放
            
        Returns:
            True 如果应该播放语音
        """
        if not self.has_speech():
            return False
        if speech_played:
            return False
        return elapsed_time >= self.speech_delay

