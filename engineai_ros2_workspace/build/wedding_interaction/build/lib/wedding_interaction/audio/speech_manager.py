"""
语音管理器

混合方案：
1. 预录音频 - 用于固定语句（问候、倒计时、引导语）
2. 离线 TTS - 用于动态对话（大模型输出）

推荐 TTS 引擎：Piper TTS
- GitHub: https://github.com/rhasspy/piper
- 轻量级，适合 Jetson 设备
- 中文支持良好
"""

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, Optional, Callable
from pathlib import Path
import logging
import subprocess
import threading
import queue
import os
import asyncio
import tempfile


class SpeechType(Enum):
    """语音类型"""
    PRERECORDED = auto()  # 预录音频
    TTS = auto()          # TTS 生成


@dataclass
class SpeechItem:
    """语音项"""
    id: str                          # 语音 ID
    text: str                        # 文本内容
    speech_type: SpeechType          # 类型
    audio_file: Optional[str] = None # 音频文件路径（预录）
    priority: int = 0                # 优先级（越高越优先）


class SpeechManager:
    """
    语音管理器
    
    特性：
    - 支持预录音频和 TTS 混合
    - 语音队列管理
    - 非阻塞播放
    - 可打断当前播放
    """
    
    def __init__(
        self,
        audio_dir: Optional[str] = None,
        tts_enabled: bool = True,
        tts_engine: str = "edge",  # "edge" | "piper" | "sherpa" | "mock"
        tts_model: Optional[str] = None,
        tts_voice: str = "zh-CN-XiaoxiaoNeural",  # Edge TTS 声音
        tts_rate: str = "+0%",  # Edge TTS 语速
    ):
        """
        初始化语音管理器
        
        Args:
            audio_dir: 预录音频目录
            tts_enabled: 是否启用 TTS
            tts_engine: TTS 引擎类型 ("edge" 推荐，与预录音频音色一致)
            tts_model: TTS 模型路径（Piper/Sherpa 使用）
            tts_voice: Edge TTS 声音（默认: zh-CN-XiaoxiaoNeural）
            tts_rate: Edge TTS 语速（默认: +0%）
        """
        self.logger = logging.getLogger("SpeechManager")
        
        # 音频目录
        if audio_dir:
            self.audio_dir = Path(audio_dir)
        else:
            # 默认使用包内的 audio 目录
            self.audio_dir = Path(__file__).parent.parent.parent / "audio_resources"
        
        # TTS 配置
        self.tts_enabled = tts_enabled
        self.tts_engine = tts_engine
        self.tts_model = tts_model
        self.tts_voice = tts_voice
        self.tts_rate = tts_rate
        self._tts_available = False
        
        # 预录语音库
        self._prerecorded: Dict[str, SpeechItem] = {}
        
        # 播放队列
        self._queue: queue.PriorityQueue = queue.PriorityQueue()
        self._current_process: Optional[subprocess.Popen] = None
        self._is_playing = False
        self._stop_flag = threading.Event()
        
        # 播放线程
        self._play_thread: Optional[threading.Thread] = None
        
        # 回调
        self._on_start: Optional[Callable[[str], None]] = None
        self._on_finish: Optional[Callable[[str], None]] = None
        
        # 初始化
        self._init_prerecorded_library()
        self._check_tts_availability()
    
    def _init_prerecorded_library(self) -> None:
        """初始化预录语音库"""
        
        # 固定语音定义（ID -> 文本）
        # 音频文件命名规则：{id}.wav 或 {id}.mp3
        fixed_speeches = {
            # IDLE 状态
            "idle_greeting": "你好呀，欢迎来到婚礼现场！",
            "idle_welcome": "欢迎欢迎，今天是个好日子！",
            "idle_random_1": "有什么可以帮你的吗？",
            "idle_random_2": "需要合影吗？摆个 Pose 就可以啦！",
            
            # 问候
            "greeting_hello": "你好！很高兴见到你！",
            "greeting_nice": "哇，你今天真好看！",
            "greeting_bride": "恭喜恭喜！新娘子真美！",
            "greeting_groom": "恭喜恭喜！新郎官好帅！",
            
            # 合影
            "photo_ready": "准备好了吗？",
            "photo_countdown_3": "三！",
            "photo_countdown_2": "二！",
            "photo_countdown_1": "一！",
            "photo_cheese": "茄子！",
            "photo_done": "拍好啦！",
            "photo_again": "再来一张吗？",
            
            # 送别
            "farewell_bye": "再见！祝你们幸福美满！",
            "farewell_happy": "百年好合！永结同心！",
            "farewell_thanks": "谢谢光临！",
            
            # 采访
            "interview_start": "让我来采访一下！",
            "interview_question_1": "请问你和新人是什么关系呀？",
            "interview_question_2": "有什么祝福想对新人说的吗？",
            "interview_thanks": "谢谢你的祝福！",
            
            # 系统
            "system_error": "抱歉，出了点小问题。",
            "system_too_close": "请稍微后退一点点。",
        }
        
        for speech_id, text in fixed_speeches.items():
            # 检查是否有预录音频文件
            audio_file = self._find_audio_file(speech_id)
            
            item = SpeechItem(
                id=speech_id,
                text=text,
                speech_type=SpeechType.PRERECORDED if audio_file else SpeechType.TTS,
                audio_file=audio_file,
            )
            self._prerecorded[speech_id] = item
            
            if audio_file:
                self.logger.debug(f"Loaded prerecorded: {speech_id} -> {audio_file}")
    
    def _find_audio_file(self, speech_id: str) -> Optional[str]:
        """查找预录音频文件（优先 MP3，因为 Edge TTS 生成的更自然）"""
        if not self.audio_dir.exists():
            return None
        
        # 优先使用 MP3（Edge TTS 生成，音质更好）
        for ext in [".mp3", ".wav", ".ogg"]:
            path = self.audio_dir / f"{speech_id}{ext}"
            if path.exists():
                return str(path)
        
        return None
    
    def _check_tts_availability(self) -> None:
        """检查 TTS 是否可用"""
        if not self.tts_enabled:
            self._tts_available = False
            return
        
        if self.tts_engine == "mock":
            # Mock 模式，用于测试
            self._tts_available = True
            self.logger.info("TTS engine: mock (for testing)")
            return
        
        if self.tts_engine == "edge":
            # 检查 edge-tts 是否安装
            try:
                import edge_tts
                self._tts_available = True
                self.logger.info(f"TTS engine: edge-tts (voice: {self.tts_voice})")
            except ImportError:
                self._tts_available = False
                self.logger.warning(
                    "Edge TTS not found. Install: pip install edge-tts\n"
                    "This is recommended to match the voice of prerecorded audio files."
                )
            return
        
        if self.tts_engine == "piper":
            # 检查 piper 是否安装
            try:
                result = subprocess.run(
                    ["piper", "--help"],
                    capture_output=True,
                    timeout=5
                )
                self._tts_available = result.returncode == 0
            except (FileNotFoundError, subprocess.TimeoutExpired):
                self._tts_available = False
            
            if self._tts_available:
                self.logger.info("TTS engine: piper (available)")
            else:
                self.logger.warning(
                    "Piper TTS not found. Install: pip install piper-tts\n"
                    "Or download from: https://github.com/rhasspy/piper/releases"
                )
        
        elif self.tts_engine == "sherpa":
            # 检查 sherpa-onnx 是否安装
            try:
                import sherpa_onnx
                self._tts_available = True
                self.logger.info("TTS engine: sherpa-onnx (available)")
            except ImportError:
                self._tts_available = False
                self.logger.warning(
                    "Sherpa-ONNX not found. Install: pip install sherpa-onnx"
                )
    
    def speak(
        self,
        text_or_id: str,
        priority: int = 0,
        interrupt: bool = False,
    ) -> bool:
        """
        播放语音
        
        Args:
            text_or_id: 语音 ID（预录）或文本（TTS）
            priority: 优先级（越高越优先）
            interrupt: 是否打断当前播放
            
        Returns:
            True 如果成功加入队列
        """
        # 检查是否是预录语音 ID
        if text_or_id in self._prerecorded:
            item = self._prerecorded[text_or_id]
        else:
            # 作为 TTS 文本处理
            if not self._tts_available:
                self.logger.warning(f"TTS not available, cannot speak: {text_or_id}")
                return False
            
            item = SpeechItem(
                id=f"tts_{hash(text_or_id)}",
                text=text_or_id,
                speech_type=SpeechType.TTS,
                priority=priority,
            )
        
        # 打断当前播放
        if interrupt:
            self.stop()
        
        # 加入队列（priority 取负值，因为 PriorityQueue 是最小堆）
        self._queue.put((-priority, item))
        
        # 确保播放线程运行
        self._ensure_play_thread()
        
        return True
    
    def _ensure_play_thread(self) -> None:
        """确保播放线程运行"""
        if self._play_thread is None or not self._play_thread.is_alive():
            self._stop_flag.clear()
            self._play_thread = threading.Thread(target=self._play_loop, daemon=True)
            self._play_thread.start()
    
    def _play_loop(self) -> None:
        """播放循环（在独立线程中运行）"""
        while not self._stop_flag.is_set():
            try:
                # 等待队列中的项目
                priority, item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            
            if self._stop_flag.is_set():
                break
            
            self._play_item(item)
    
    def _play_item(self, item: SpeechItem) -> None:
        """播放单个语音项"""
        self._is_playing = True
        
        if self._on_start:
            self._on_start(item.id)
        
        self.logger.info(f"Playing: {item.id} - {item.text[:30]}...")
        
        try:
            if item.speech_type == SpeechType.PRERECORDED and item.audio_file:
                self._play_audio_file(item.audio_file)
            else:
                self._play_tts(item.text)
        except Exception as e:
            self.logger.error(f"Play error: {e}")
        finally:
            self._is_playing = False
            if self._on_finish:
                self._on_finish(item.id)
    
    def _play_audio_file(self, audio_file: str) -> None:
        """播放音频文件"""
        # 根据文件格式选择播放器
        # aplay 只支持 WAV，MP3 需要 ffplay 或 mpv
        try:
            file_ext = os.path.splitext(audio_file)[1].lower()
            
            if file_ext == ".wav" and os.path.exists("/usr/bin/aplay"):
                # WAV 文件用 aplay
                cmd = ["aplay", "-q", audio_file]
            elif os.path.exists("/usr/bin/ffplay"):
                # MP3/其他格式用 ffplay
                cmd = ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", audio_file]
            elif os.path.exists("/usr/bin/mpv"):
                # 或用 mpv
                cmd = ["mpv", "--no-video", "--really-quiet", audio_file]
            elif os.path.exists("/usr/bin/aplay") and file_ext == ".wav":
                # 回退到 aplay（仅 WAV）
                cmd = ["aplay", "-q", audio_file]
            else:
                self.logger.warning(f"No suitable audio player found for {file_ext}")
                return
            
            self._current_process = subprocess.Popen(cmd)
            self._current_process.wait()
            
        except Exception as e:
            self.logger.error(f"Audio playback error: {e}")
        finally:
            self._current_process = None
    
    def _play_tts(self, text: str) -> None:
        """使用 TTS 播放文本"""
        if self.tts_engine == "mock":
            # Mock 模式：打印文本并等待
            self.logger.info(f"[MOCK TTS] {text}")
            import time
            time.sleep(len(text) * 0.1)  # 模拟播放时间
            return
        
        if self.tts_engine == "edge":
            self._play_with_edge_tts(text)
        elif self.tts_engine == "piper":
            self._play_with_piper(text)
        elif self.tts_engine == "sherpa":
            self._play_with_sherpa(text)
    
    def _play_with_edge_tts(self, text: str) -> None:
        """使用 Edge TTS 播放文本（推荐，与预录音频音色一致）"""
        try:
            import edge_tts
            
            # 创建临时文件
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp_file:
                tmp_path = tmp_file.name
            
            # 异步生成并保存音频
            async def generate_and_play():
                communicate = edge_tts.Communicate(text, self.tts_voice, rate=self.tts_rate)
                await communicate.save(tmp_path)
                
                # 播放生成的音频文件
                self._play_audio_file(tmp_path)
                
                # 清理临时文件
                try:
                    os.remove(tmp_path)
                except Exception:
                    pass
            
            # 运行异步任务
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
            
            loop.run_until_complete(generate_and_play())
            
        except ImportError:
            self.logger.error("Edge TTS not installed. Install: pip install edge-tts")
        except Exception as e:
            self.logger.error(f"Edge TTS error: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def _play_with_piper(self, text: str) -> None:
        """使用 Piper TTS"""
        try:
            # Piper 命令行：echo "text" | piper --model model.onnx --output-raw | aplay -r 22050 -f S16_LE
            model_arg = f"--model {self.tts_model}" if self.tts_model else ""
            
            # 管道：piper -> aplay
            piper_cmd = f'echo "{text}" | piper {model_arg} --output-raw | aplay -r 22050 -f S16_LE -q'
            
            self._current_process = subprocess.Popen(
                piper_cmd,
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._current_process.wait()
            
        except Exception as e:
            self.logger.error(f"Piper TTS error: {e}")
        finally:
            self._current_process = None
    
    def _play_with_sherpa(self, text: str) -> None:
        """使用 Sherpa-ONNX TTS"""
        try:
            import sherpa_onnx
            # TODO: 实现 Sherpa TTS
            self.logger.warning("Sherpa TTS not yet implemented")
        except ImportError:
            self.logger.error("Sherpa-ONNX not installed")
    
    def stop(self) -> None:
        """停止当前播放"""
        if self._current_process:
            self._current_process.terminate()
            self._current_process = None
        
        # 清空队列
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
    
    def shutdown(self) -> None:
        """关闭语音管理器"""
        self._stop_flag.set()
        self.stop()
        
        if self._play_thread:
            self._play_thread.join(timeout=1.0)
    
    @property
    def is_playing(self) -> bool:
        """是否正在播放"""
        return self._is_playing
    
    def on_start(self, callback: Callable[[str], None]) -> None:
        """设置播放开始回调"""
        self._on_start = callback
    
    def on_finish(self, callback: Callable[[str], None]) -> None:
        """设置播放结束回调"""
        self._on_finish = callback
    
    def get_text(self, speech_id: str) -> Optional[str]:
        """获取语音 ID 对应的文本"""
        item = self._prerecorded.get(speech_id)
        return item.text if item else None
    
    def list_prerecorded(self) -> Dict[str, str]:
        """列出所有预录语音"""
        return {
            item.id: item.text
            for item in self._prerecorded.values()
        }

