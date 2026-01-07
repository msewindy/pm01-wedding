"""
音频播放器

简单的音频文件播放封装
"""

import subprocess
import threading
import os
import logging
from typing import Optional, Callable
from pathlib import Path


class AudioPlayer:
    """
    简单的音频播放器
    
    支持：
    - WAV, MP3, OGG 格式
    - 非阻塞播放
    - 可打断
    """
    
    def __init__(self):
        self.logger = logging.getLogger("AudioPlayer")
        self._current_process: Optional[subprocess.Popen] = None
        self._is_playing = False
        self._on_finish: Optional[Callable[[], None]] = None
        
        # 检测可用的播放器
        self._player = self._detect_player()
        if self._player:
            self.logger.info(f"Using audio player: {self._player}")
        else:
            self.logger.warning("No audio player detected!")
    
    def _detect_player(self) -> Optional[str]:
        """检测可用的音频播放器"""
        players = [
            "/usr/bin/aplay",      # ALSA (Linux)
            "/usr/bin/paplay",     # PulseAudio
            "/usr/bin/mpv",        # MPV
            "/usr/bin/ffplay",     # FFmpeg
            "/usr/bin/mplayer",    # MPlayer
        ]
        
        for player in players:
            if os.path.exists(player):
                return player
        
        return None
    
    def play(self, audio_file: str, blocking: bool = False) -> bool:
        """
        播放音频文件
        
        Args:
            audio_file: 音频文件路径
            blocking: 是否阻塞等待播放完成
            
        Returns:
            True 如果开始播放成功
        """
        if not self._player:
            self.logger.error("No audio player available")
            return False
        
        if not os.path.exists(audio_file):
            self.logger.error(f"Audio file not found: {audio_file}")
            return False
        
        # 停止当前播放
        self.stop()
        
        # 构建命令
        if "aplay" in self._player:
            cmd = [self._player, "-q", audio_file]
        elif "paplay" in self._player:
            cmd = [self._player, audio_file]
        elif "mpv" in self._player:
            cmd = [self._player, "--no-video", "--really-quiet", audio_file]
        elif "ffplay" in self._player:
            cmd = [self._player, "-nodisp", "-autoexit", "-loglevel", "quiet", audio_file]
        else:
            cmd = [self._player, audio_file]
        
        try:
            self._is_playing = True
            self._current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            
            if blocking:
                self._current_process.wait()
                self._is_playing = False
                if self._on_finish:
                    self._on_finish()
            else:
                # 非阻塞：启动监控线程
                thread = threading.Thread(target=self._wait_finish, daemon=True)
                thread.start()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Play error: {e}")
            self._is_playing = False
            return False
    
    def _wait_finish(self) -> None:
        """等待播放完成"""
        if self._current_process:
            self._current_process.wait()
        
        self._is_playing = False
        self._current_process = None
        
        if self._on_finish:
            self._on_finish()
    
    def stop(self) -> None:
        """停止播放"""
        if self._current_process:
            try:
                self._current_process.terminate()
                self._current_process.wait(timeout=0.5)
            except:
                try:
                    self._current_process.kill()
                except:
                    pass
            self._current_process = None
        
        self._is_playing = False
    
    @property
    def is_playing(self) -> bool:
        """是否正在播放"""
        return self._is_playing
    
    def on_finish(self, callback: Callable[[], None]) -> None:
        """设置播放结束回调"""
        self._on_finish = callback

