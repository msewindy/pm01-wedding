"""
音频模块

包含语音播放器和 TTS 引擎
"""

from .audio_player import AudioPlayer
from .speech_manager import SpeechManager, SpeechType

__all__ = [
    'AudioPlayer',
    'SpeechManager',
    'SpeechType',
]

