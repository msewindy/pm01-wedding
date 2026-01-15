"""
采访录制器

负责视频+音频的录制功能
基于双线程架构：视频和音频分别在线程中录制，最后使用 ffmpeg 合并
"""

import os
import sys
import time
import threading
import subprocess
from datetime import datetime
from typing import Optional, Callable
import logging
import ctypes
from contextlib import contextmanager
from collections import deque
import numpy as np

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

try:
    import pyaudio
    import wave
    PYAUDIO_AVAILABLE = True
except ImportError:
    PYAUDIO_AVAILABLE = False

from ...utils.audio_utils import find_audio_device, get_alsa_card_index_from_info
import queue

# ========== Global FFMPEG Worker ==========
class FFMPEGProcessor:
    """
    Global background worker for FFMPEG tasks to prevent CPU overload
    from multiple concurrent conversions.
    """
    _instance = None
    _lock = threading.Lock()
    
    def __init__(self):
        self.job_queue = queue.Queue()
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True, name="FFMPEG_Worker")
        self.worker_thread.start()
        self.logger = logging.getLogger("FFMPEGProcessor")
        
    @classmethod
    def get_instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
        return cls._instance
        
    def submit_job(self, func, *args, **kwargs):
        """Submit a job to the queue"""
        self.job_queue.put((func, args, kwargs))
        self.logger.info(f"Job submitted. Queue size: {self.job_queue.qsize()}")
        
    def _worker_loop(self):
        while self.running:
            try:
                func, args, kwargs = self.job_queue.get(timeout=1.0)
                try:
                    func(*args, **kwargs)
                except Exception as e:
                    self.logger.error(f"Job execution failed: {e}")
                finally:
                    self.job_queue.task_done()
            except queue.Empty:
                continue
                
# Initialize global processor
ffmpeg_processor = FFMPEGProcessor.get_instance()


# ========== ALSA Error Suppression ==========
# ALSA/PyAudio tends to spam "unable to open slave" errors to stderr.
# This context manager suppresses stderr at the C level.

ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def no_alsa_err():
    if sys.platform != 'linux':
        yield
        return
        
    try:
        asound = ctypes.cdll.LoadLibrary('libasound.so.2')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None)
    except:
        yield


def init_hardware_mixer(card_id=0, logger=None):
    """
    执行硬件混音器初始化命令 (ALSA)
    主要用于开启 USB 麦克风的 Capture Switch 和设置最大音量
    """
    if logger is None:
        logger = logging.getLogger("System")
        
    try:
        # 1. 开启 Mic Capture Switch (numid=3)，该开关默认是关闭的
        cmd_switch = ["amixer", "-c", str(card_id), "cset", "numid=3", "on"]
        subprocess.run(cmd_switch, check=True, capture_output=True)
        
        # 2. 确保硬件捕获音量已取消静音并设为 100%
        cmd_vol = ["amixer", "-c", str(card_id), "sset", "Mic", "100%", "unmute"]
        subprocess.run(cmd_vol, check=True, capture_output=True)
        
        logger.info(f"ALSA hardware mixer initialized successfully for Card {card_id}")
    except subprocess.CalledProcessError as e:
        logger.warning(f"Failed to set ALSA mixer for Card {card_id} (cmd failed): {e}")
    except Exception as e:
        logger.warning(f"Failed to set ALSA mixer for Card {card_id}: {e}")


# ========== PyAudio Singleton ==========
# PulseAudio/ALSA can be unstable if PyAudio is initialized/terminated repeatedly.
# We use a singleton to keep one instance alive for the process duration.
class PyAudioSingleton:
    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        if not PYAUDIO_AVAILABLE:
            return None
            
        with cls._lock:
            if cls._instance is None:
                try:
                    with no_alsa_err():
                        cls._instance = pyaudio.PyAudio()
                except Exception as e:
                    logging.getLogger("PyAudioSingleton").error(f"Failed to init PyAudio: {e}")
                    return None
        return cls._instance

    @classmethod
    def terminate(cls):
        pass 
        # Ideally we never terminate it to avoid the crash, let OS cleanup on exit.
        # calls to terminate() caused 'unaligned fastbin chunk' or assertion errors.


class VideoRecorder:
    """
    视频录制器（内部类）
    
    支持两种模式：
    1. 直接从摄像头读取（camindex）
    2. 从回调函数获取帧（frame_callback）
    """
    
    def __init__(self, filename: str, fourcc: str = "MJPG", 
                 sizex: int = 640, sizey: int = 480, 
                 camindex: Optional[int] = None,
                 fps: int = 30,
                 frame_callback: Optional[Callable] = None,
                 start_event: Optional[threading.Event] = None,
                 logger: Optional[logging.Logger] = None):
        """
        初始化视频录制器
        
        Args:
            filename: 输出视频文件名
            fourcc: 视频编码格式
            sizex, sizey: 视频分辨率
            camindex: 摄像头索引（如果为None，使用frame_callback）
            fps: 帧率
            frame_callback: 帧回调函数（返回OpenCV格式的frame，或None）
            start_event: 同步启动事件（用于与音频同步）
            logger: 日志记录器
        """
        self.open = True
        self.device_index = camindex
        self.fps = fps
        self.fourcc = fourcc
        self.frameSize = (sizex, sizey)
        self.video_filename = filename
        self.frame_counts = 0
        self.start_time = 0.0  # 将在record()中设置
        self.frame_callback = frame_callback
        self.start_event = start_event
        self.logger = logger or logging.getLogger("VideoRecorder")
        
        # 初始化视频写入器
        self.video_writer = cv2.VideoWriter_fourcc(*self.fourcc)
        self.video_out = cv2.VideoWriter(self.video_filename, self.video_writer, 
                                        self.fps, self.frameSize)
        
        if not self.video_out.isOpened():
            raise RuntimeError(f"无法创建视频文件: {self.video_filename}")
        
        # 如果使用摄像头，初始化摄像头
        self.video_cap = None
        if camindex is not None:
            if not CV_AVAILABLE:
                raise RuntimeError("OpenCV 不可用，无法使用摄像头")
            
            # 检查摄像头设备（Linux）
            if sys.platform.startswith('linux'):
                video_device = f"/dev/video{camindex}"
                if not os.path.exists(video_device):
                    self.logger.warning(f"摄像头设备 {video_device} 不存在")
            
            self.video_cap = cv2.VideoCapture(camindex)
            if not self.video_cap.isOpened():
                raise RuntimeError(f"无法打开摄像头设备 (索引: {camindex})")
            
            self.video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, sizex)
            self.video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, sizey)
            self.video_cap.set(cv2.CAP_PROP_FPS, fps)
            
    def record(self):
        """视频录制循环（在线程中运行）"""
        # 等待同步启动信号
        if self.start_event is not None:
            self.start_event.wait()
        
        # 记录实际启动时间
        self.start_time = time.time()
        
        # 计算每帧间隔
        frame_interval = 1.0 / self.fps
        next_frame_time = self.start_time
        
        while self.open:
            current_time = time.time()
            
            # 如果使用回调模式，检查是否到了下一帧的时间
            if self.frame_callback is not None:
                if current_time >= next_frame_time:
                    frame = self.frame_callback()
                    if frame is not None:
                        if self.frame_counts == 0:
                            self.logger.info(f"First frame captured from callback. Size: {frame.shape}")
                        
                        # 调整帧大小（使用高质量插值）
                        if frame.shape[:2] != self.frameSize[::-1]:
                            # 使用 INTER_LINEAR 或 INTER_CUBIC 提高缩放质量
                            # INTER_CUBIC 质量更好但稍慢，INTER_LINEAR 是平衡选择
                            frame = cv2.resize(frame, self.frameSize, interpolation=cv2.INTER_LINEAR)
                            
                        self.video_out.write(frame)
                        self.frame_counts += 1
                        next_frame_time += frame_interval
                    else:
                        # 如果没有帧，仍然更新next_frame_time，避免累积延迟
                        # 这样可以保持目标帧率，即使某些帧丢失
                        next_frame_time += frame_interval
                        # 等待一小段时间，避免CPU占用过高
                        time.sleep(0.01)
                else:
                    # 等待到下一帧时间
                    sleep_time = next_frame_time - current_time
                    if sleep_time > 0:
                        time.sleep(min(sleep_time, 0.1))  # 限制最大sleep时间
            elif self.video_cap is not None:
                # 从摄像头读取（固定帧率）
                ret, video_frame = self.video_cap.read()
                if ret:
                    if self.frame_counts == 0:
                        self.logger.info(f"First frame captured from camera. Size: {video_frame.shape}")
                    
                    self.video_out.write(video_frame)
                    self.frame_counts += 1
                    # 使用精确的时间控制
                    next_frame_time += frame_interval
                    sleep_time = next_frame_time - time.time()
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                else:
                    break
            else:
                break
        
        self.logger.info(f"Video recording finished. Total frames: {self.frame_counts}")
        
        # 释放资源（确保在线程内释放）
        if self.video_out:
            self.video_out.release()
        if self.video_cap:
            self.video_cap.release()
        if CV_AVAILABLE:
            cv2.destroyAllWindows()
    
    def stop(self):
        """停止录制"""
        self.open = False
        # 资源释放移至 record 线程结束时进行，避免线程竞争导致 crash
    
    def start(self):
        """启动录制线程"""
        video_thread = threading.Thread(target=self.record, daemon=True)
        video_thread.start()
        return video_thread


class AudioRecorder:
    """
    音频录制器（内部类）
    
    从麦克风录制音频
    """
    
    def __init__(self, filename: str, rate: int = 44100, 
                 fpb: int = 1024, channels: int = 2,
                 input_device_index: Optional[int] = None,
                 gain: float = 1.0,
                 start_event: Optional[threading.Event] = None,
                 logger: Optional[logging.Logger] = None):
        """
        初始化音频录制器
        
        Args:
            filename: 输出音频文件名
            rate: 采样率
            fpb: 每帧采样数
            channels: 声道数
            input_device_index: 输入设备索引（None=默认，-1=自动检测USB）
            gain: 软件数字增益倍数 (1.0 = 原始音量)
            start_event: 同步启动事件（用于与视频同步）
            logger: 日志记录器
        """
        if not PYAUDIO_AVAILABLE:
            raise RuntimeError("PyAudio 不可用，无法录制音频")
        
        self.open = True
        self.rate = rate
        self.frames_per_buffer = fpb
        self.channels = channels
        self.format = pyaudio.paInt16
        self.audio_filename = filename
        self.gain = gain
        self.start_event = start_event
        self.start_time = 0.0  # 将在record()中设置
        self.logger = logger or logging.getLogger("AudioRecorder")
        
        # 使用单例获取 PyAudio 实例
        # 避免重复创建/销毁导致的 PulseAudio 崩溃
        self.audio = PyAudioSingleton.get_instance()
        
        if self.audio is None:
             raise RuntimeError("无法初始化 PyAudio")
        
        # 使用通用工具查找设备
        # 注意: 如果 input_device_index 为 None, 这里的逻辑可能需要调整以匹配 find_audio_device 的行为
        # 我们的 find_audio_device 接受 device_id_param (-1 for auto)
        # 兼容旧逻辑：如果 input_device_index is None, 视为 -1 (Auto detect USB) or Default?
        # 通常 AudioRecorder(input_device_index=None) 意味着使用系统默认。
        # 但我们希望统一逻辑。如果 passed None, we default to system default? 
        # 用户界面传进来的是 audio_device_id (-1 auto). 
        # 所以我们这里 input_device_index 直接透传给 find_audio_device.
        # find_audio_device handles -1 (auto search) and >=0 (manual).
        # find_audio_device doesn't handle None explicitely as 'system default', 
        # but if we pass -1 it looks for USB. 
        # If we want pure system default, we should pass something else or modify find_audio_device.
        # But 'unify' means using the same logic. So we should use find_audio_device.
        
        target_device_index = input_device_index if input_device_index is not None else -1
        
        actual_index, device_info = find_audio_device(
            self.audio, 
            device_id_param=target_device_index,
            target_name="USB Audio", 
            logger=self.logger
        )
        
        if actual_index is None:
             # Fallback to default if utility failed? (Utility keeps logging but returns None if critical fail)
             # But utility usually falls back to default if USB not found.
             # If it returns None, it really failed.
             self.logger.warning("find_audio_device failed to return valid index. Trying system default.")
             try:
                 actual_index = self.audio.get_default_input_device_info()['index']
             except:
                 pass

        self.logger.info(f"AudioRecorder using device index: {actual_index}")
        
        # 打开音频流
        try:
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                input_device_index=actual_index,
                frames_per_buffer=self.frames_per_buffer
            )
        except Exception as e:
            # self.audio.terminate() # Do not terminate singleton
            raise RuntimeError(f"无法打开音频输入设备 (idx={actual_index}): {e}")
        
        self.audio_frames = []
    
    def record(self):
        """音频录制循环（在线程中运行）"""
        # 等待同步启动信号
        if self.start_event is not None:
            self.start_event.wait()
        
        # 记录实际启动时间
        self.start_time = time.time()
        
        try:
            self.stream.start_stream()
            while self.open:
                try:
                    data = self.stream.read(self.frames_per_buffer, exception_on_overflow=False)
                    
                    # 应用软件增益
                    if self.gain != 1.0:
                        try:
                            # 转换为 numpy 数组
                            audio_data = np.frombuffer(data, dtype=np.int16)
                            # 应用增益并裁剪防止溢出
                            amplified_data = np.clip(audio_data * self.gain, -32768, 32767).astype(np.int16)
                            self.audio_frames.append(amplified_data.tobytes())
                        except Exception as e:
                            # 如果处理失败，回退到原始数据
                            self.logger.warning(f"Audio gain processing failed: {e}")
                            self.audio_frames.append(data)
                    else:
                        self.audio_frames.append(data)
                except Exception as e:
                    self.logger.error(f"音频读取错误: {e}")
                    break
        except Exception as e:
            self.logger.error(f"音频流错误: {e}")
        finally:
            # 在同一线程中关闭流，避免 PulseAudio 崩溃
            if self.stream:
                try:
                    self.stream.stop_stream()
                    self.stream.close()
                    self.logger.info("Audio stream closed safely in thread")
                except Exception as e:
                    self.logger.warning(f"Error closing audio stream: {e}")
    
            # 这里不做文件保存，避免在主线程做耗时操作或线程未结束时操作
            # 文件保存由 save_file() 显式调用，或者在线程结束后进行
            pass
    
    def stop(self):
        """停止录制"""
        self.open = False

    def save_file(self):
        """保存音频文件 (需要在 stop 后，线程 join 后调用)"""
        if self.audio_frames:
            try:
                # 检查线程是否确实结束了
                if threading.current_thread() != threading.main_thread():
                     # 如果是在录制线程中调用（不推荐），则没问题
                     pass
                
                waveFile = wave.open(self.audio_filename, 'wb')
                waveFile.setnchannels(self.channels)
                waveFile.setsampwidth(self.audio.get_sample_size(self.format))
                waveFile.setframerate(self.rate)
                waveFile.writeframes(b''.join(self.audio_frames))
                waveFile.close()
                self.logger.info(f"Audio saved to {self.audio_filename}")
            except Exception as e:
                self.logger.error(f"保存音频文件失败: {e}")
    
    def release(self):
        """释放 PyAudio 资源（必须在线程结束后调用）"""
        if self.audio:
            self.logger.info("Releasing AudioRecorder resources (PyAudio instance kept alive)")
            # Do not terminate the instance, as it is shared singleton
            # PyAudioSingleton.terminate() 
            self.audio = None
    
    def get_current_energy(self) -> float:
        """获取当前音频能量 (RMS)"""
        if not self.audio_frames:
            return 0.0
        try:
            # 获取最后一帧数据
            last_chunk = self.audio_frames[-1]
            # 计算 RMS (Root Mean Square)
            import math
            import struct
            
            # 假设是 16-bit 整数
            count = len(last_chunk) // 2
            if count == 0:
                return 0.0
                
            shorts = struct.unpack(f"{count}h", last_chunk)
            sum_squares = sum(s ** 2 for s in shorts)
            rms = math.sqrt(sum_squares / count)
            
            # 标准化到 0-1 (16bit max is 32768)
            return rms / 32768.0
        except Exception:
            return 0.0

    
    def start(self):
        """启动录制线程"""
        audio_thread = threading.Thread(target=self.record, daemon=True)
        audio_thread.start()
        return audio_thread


class InterviewRecorder:
    """
    采访录制器
    
    功能：
    - 录制视频+音频流
    - 保存为媒体文件（MP4格式）
    
    架构：
    - 使用双线程分别录制视频和音频
    - 先保存到临时文件，最后使用 ffmpeg 合并
    """
    
    def __init__(self, save_path: str, 
                 video_size: tuple = (640, 480),
                 video_fps: int = 30,
                 audio_sample_rate: int = 44100,
                 camera_index: Optional[int] = None,
                 frame_callback: Optional[Callable] = None,

                 video_quality: int = 18,  # CRF 值：18=高质量，23=中等质量，28=低质量
                 audio_gain: float = 1.0,
                 audio_device_index: int = -1, # PyAudio Device Index
                 logger: Optional[logging.Logger] = None):
        """
        初始化录制器
        
        Args:
            save_path: 保存路径（目录或完整文件路径）
            video_size: 视频分辨率 (width, height)
            video_fps: 视频帧率
            audio_sample_rate: 音频采样率
            camera_index: 摄像头索引（如果为None且frame_callback也为None，则只录制音频）
            frame_callback: 帧回调函数（返回OpenCV格式的frame）
            video_quality: 视频质量（CRF值，18=高质量，23=中等质量，28=低质量）
            audio_gain: 音频软件增益
            audio_device_index: 音频设备索引 (PyAudio)
            logger: 日志记录器
        """
        self.save_path = save_path
        self.logger = logger or logging.getLogger("InterviewRecorder")
        self.logger = logger or logging.getLogger(__name__)
        self.video_size = video_size
        self.video_fps = video_fps
        self.audio_sample_rate = audio_sample_rate
        self.camera_index = camera_index
        self.frame_callback = frame_callback
        self.video_quality = video_quality
        self.audio_gain = audio_gain
        self.audio_device_index = audio_device_index
        self.logger = logger or logging.getLogger(__name__)
        
        # 自动检测最佳编码器
        self._encoder_config = self._detect_video_encoder()
        self.logger.info(f"Selected video encoder: {self._encoder_config['name']}")

        self._stop_event = threading.Event()
        self._video_thread = None
        
        # 录制状态
        self._is_recording = False
        self._video_recorder: Optional[VideoRecorder] = None
        self._audio_recorder: Optional[AudioRecorder] = None
        self._audio_thread: Optional[threading.Thread] = None
        
        # 录制参数
        self.max_duration = 300.0  # 最大录制时长（秒）
        self.min_duration = 5.0   # 最小录制时长（秒）
        
        # 录制统计
        self._start_time = 0.0
        self._actual_save_path = None
        
        # 临时文件路径
        self._temp_dir = "/tmp"
        self._temp_video = None
        self._temp_audio = None
        
        # 同步启动事件（确保视频和音频同时开始）
        self._start_event = threading.Event()

    def _detect_video_encoder(self):
        """
        检测视频编码器
        
        注意：根据测试，Jetson 上的 h264_v4l2m2m 和 h264_nvmpi 均不可用（报错无法找到设备）。
        因此，我们在 Jetson 上强制使用高度优化的 libx264 (ultrafast preset)。
        在台式机上保留 h264_nvenc 支持。
        """
        try:
            cmd = ["ffmpeg", "-encoders"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            output = result.stdout
            
            # 优先级 1: Desktop NVIDIA NVENC (台式机可用)
            if "h264_nvenc" in output:
                return {
                    "name": "h264_nvenc",
                    "flags": [
                        "-c:v", "h264_nvenc", 
                        "-preset", "p4", 
                        "-rc", "vbr", 
                        "-cq", str(self.video_quality),
                        "-pix_fmt", "yuv420p"
                    ]
                }

            # 默认 & Jetson: 优化的软件编码
            # 使用 ultrafast 预设和限制线程数，确保不卡顿系统
            return {
                "name": "libx264-ultrafast",
                "flags": [
                    "-c:v", "libx264", 
                    "-preset", "ultrafast",  # 牺牲画质/体积换取极高速度和低CPU占用
                    "-crf", "28",           # 稍微降低画质以进一步减轻负载
                    "-threads", "4",        # 限制线程数，防止占满CPU
                    "-profile:v", "baseline", # 使用最简单的 profile
                    "-pix_fmt", "yuv420p"
                ]
            }
        except Exception as e:
            self.logger.error(f"Failed to detect encoder: {e}, falling back to libx264")
            return {
                "name": "libx264",
                "flags": ["-c:v", "libx264", "-preset", "ultrafast", "-crf", "28", "-threads", "4"]
            }
    
    def start(self) -> bool:
        """
        开始录制
        
        Returns:
            True 如果成功，False 如果失败
        """
        if self._is_recording:
            self.logger.warning("Recording already in progress")
            return False
        
        # 尝试创建目录（如果看起来像目录）
        if not os.path.exists(self.save_path) and not os.path.splitext(self.save_path)[1]:
            try:
                os.makedirs(self.save_path, exist_ok=True)
            except Exception as e:
                self.logger.warning(f"Could not create directory {self.save_path}: {e}")

        # 生成保存路径
        if os.path.isdir(self.save_path):
            # 如果是目录，生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"blessing_{timestamp}.mp4"
            self._actual_save_path = os.path.join(self.save_path, filename)
        else:
            # 如果是文件路径，直接使用
            self._actual_save_path = self.save_path
            # 确保父目录存在
            output_dir = os.path.dirname(self._actual_save_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)
        
        # 生成临时文件路径
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._temp_video = os.path.join(self._temp_dir, f"temp_video_{timestamp}.avi")
        self._temp_audio = os.path.join(self._temp_dir, f"temp_audio_{timestamp}.wav")
        
        try:
            # 重置同步事件
            self._start_event.clear()
            
            # 启动视频录制（如果可用）
            if CV_AVAILABLE and (self.camera_index is not None or self.frame_callback is not None):
                try:
                    self._video_recorder = VideoRecorder(
                        filename=self._temp_video,
                        fourcc="MJPG",
                        sizex=self.video_size[0],
                        sizey=self.video_size[1],
                        camindex=self.camera_index,
                        fps=self.video_fps,
                        frame_callback=self.frame_callback,
                        start_event=self._start_event,
                        logger=self.logger
                    )
                    self._video_thread = self._video_recorder.start()
                    self.logger.info("Video recording thread started")
                except Exception as e:
                    self.logger.warning(f"Failed to start video recording: {e}")
                    self._video_recorder = None
            
            # 启动音频录制
            if PYAUDIO_AVAILABLE:
                try:
                    self._audio_recorder = AudioRecorder(
                        filename=self._temp_audio,
                        rate=self.audio_sample_rate,
                        gain=self.audio_gain,
                        input_device_index=self.audio_device_index,
                        start_event=self._start_event,
                        logger=self.logger
                    )
                    self._audio_thread = self._audio_recorder.start()
                    self.logger.info("Audio recording thread started")
                except Exception as e:
                    self.logger.warning(f"Failed to start audio recording: {e}")
                    self._audio_recorder = None
            
            if self._video_recorder is None and self._audio_recorder is None:
                self.logger.error("Both video and audio recording failed")
                return False
            
            # 等待一小段时间确保线程已启动
            time.sleep(0.1)
            
            # 同步启动：同时触发视频和音频开始录制
            self._start_time = time.time()
            self._start_event.set()
            self.logger.info("Synchronized start signal sent")
            
            self._is_recording = True
            
            self.logger.info(f"Recording started: {self._actual_save_path}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start recording: {e}")
            self._cleanup()
            return False

    def _cleanup(self, exclude_temps=False):
        """清理资源"""
        self._is_recording = False
        self._video_recorder = None
        self._audio_recorder = None
        
        # 如果不是正在异步处理，清理临时路径
        if not exclude_temps:
            self._temp_video = None
            self._temp_audio = None
    
    def write_frame(self, frame=None, audio_chunk=None) -> bool:
        """
        写入一帧数据（用于回调模式）
        
        注意：如果使用摄像头模式，此方法不需要调用
        如果使用frame_callback模式，此方法也不需要使用（回调会自动调用）
        
        Args:
            frame: 视频帧（OpenCV格式，BGR）
            audio_chunk: 音频数据块（暂不支持，音频自动从麦克风录制）
        
        Returns:
            True 如果成功，False 如果失败
        """
        if not self._is_recording:
            return False
        
        # 检查最大时长
        elapsed = time.time() - self._start_time
        if elapsed >= self.max_duration:
            self.logger.info(f"Recording reached max duration ({self.max_duration}s)")
            return False
        
        # 注意：实际的帧写入由 VideoRecorder 线程处理
        # 这里主要用于兼容性
        return True
    
    def stop(self) -> Optional[str]:
        """
        停止录制并保存文件
        
        Returns:
            保存的文件路径，如果失败返回 None
        """
        if not self._is_recording:
            self.logger.warning("No recording in progress")
            return None
        
        try:
            # 检查最小时长
            elapsed = time.time() - self._start_time
            if elapsed < self.min_duration:
                self.logger.warning(f"Recording too short ({elapsed:.1f}s < {self.min_duration}s), discarding")
                self._cleanup()
                return None
            
            # 停止录制
            self._is_recording = False
            
            # 停止视频录制
            # 停止视频录制
            # 提取资源引用以传递给后台线程
            video_out_ref = None
            video_cap_ref = None
            saved_frames_count = 0
            
            if self._video_recorder:
                self._video_recorder.stop()
                if self._video_thread:
                    self._video_thread.join(timeout=2.0)
                # 获取底层 CV2 对象引用，以便在后台线程 release
                video_out_ref = self._video_recorder.video_out
                video_cap_ref = self._video_recorder.video_cap
                saved_frames_count = self._video_recorder.frame_counts
                
                # 防止 VideoRecorder 析构时重复 release (虽然 Python 不一定会析构)
                self._video_recorder.video_out = None
                self._video_recorder.video_cap = None
            
            # 停止音频录制 (仅标记 stop，实际 join/save 在 finalize 中做)
            # 提取引用
            audio_recorder_ref = self._audio_recorder
            audio_thread_ref = self._audio_thread
            
            # 等待音频线程 (非阻塞检查，因为后面有 join)
            # while threading.active_count() > 1: # 不要阻塞主线程
            #    time.sleep(0.1)
            
            # 合并音视频 (Async via Queue)
            # 使用全局队列来串行处理 FFMPEG 任务，避免并发导致 CPU 过载
            
            # 提交任务到队列
            ffmpeg_processor.submit_job(
                self._finalize_recording_task,
                elapsed, saved_frames_count, video_out_ref, video_cap_ref, audio_recorder_ref, audio_thread_ref
            )
            
            # 清理句柄，但保留临时文件引用供 Worker 使用
            # 注意: 这里不能把 self._temp_video 设为 None，因为 Worker 需要用到它
            # 但 _cleanup 默认会清理 worker 只需要路径字符串，或者是已经复制过去的值?
            # _finalize_recording_task 使用的是 self 实例的方法吗?
            # 下面的 _finalize_recording_task 是实例方法，它访问 self._temp_video 等成员。
            # 这意味着 self 不能被销毁。
            # 但 InterviewState 会把 recorder 设为 None。由于闭包/Job引用，Instance 会保持存活直到 Job 完成。
            
            self._cleanup(exclude_temps=True) 
            return "Async_Processing"
            
        except Exception as e:
            self.logger.error(f"Failed to stop recording: {e}")
            self._cleanup()
            return None

    def _finalize_recording_task(self, elapsed_time, frames_count, video_out, video_cap, audio_rec, audio_thread):
        """执行具体的资源释放和合并任务 (在 Worker 线程中运行)"""
        try:
            # 1. 确保视频资源释放
            if video_out: video_out.release()
            if video_cap: video_cap.release()
            if CV_AVAILABLE: cv2.destroyAllWindows()
            
            # 2. 确保音频线程结束并保存
            if audio_rec and audio_thread:
                audio_rec.stop()
                audio_thread.join(timeout=5.0)
                if audio_rec.open: 
                     audio_rec.open = False
                audio_rec.save_file()
                audio_rec.release()

            # 3. 合并
            saved_path = self._merge_audio_video(elapsed_time)
            
            if saved_path:
                self.logger.info("="*50)
                self.logger.info(f" [RESULT] Interview Video Saved: {saved_path} (duration: {elapsed_time:.1f}s)")
                self.logger.info("="*50)
            
            # 4. 清理临时文件
            if self._temp_video and os.path.exists(self._temp_video):
               try: os.remove(self._temp_video) 
               except: pass
            if self._temp_audio and os.path.exists(self._temp_audio):
               try: os.remove(self._temp_audio) 
               except: pass
               
        except Exception as e:
            self.logger.error(f"Async finalize task failed: {e}")
            
        except Exception as e:
            self.logger.error(f"Failed to stop recording: {e}")
            self._cleanup()
            return None
    
    def _merge_audio_video(self, elapsed_time: float) -> Optional[str]:
        """
        使用 ffmpeg 合并音视频
        
        Args:
            elapsed_time: 录制时长（秒）
        
        Returns:
            合并后的文件路径，如果失败返回 None
        """
        # 检查临时文件是否存在
        has_video = self._temp_video and os.path.exists(self._temp_video)
        has_audio = self._temp_audio and os.path.exists(self._temp_audio)
        
        # 记录临时文件信息
        if self._temp_video:
            video_size = os.path.getsize(self._temp_video) if os.path.exists(self._temp_video) else 0
            self.logger.info(f"Temp video file: {self._temp_video} (exists: {has_video}, size: {video_size} bytes)")
        if self._temp_audio:
            audio_size = os.path.getsize(self._temp_audio) if os.path.exists(self._temp_audio) else 0
            self.logger.info(f"Temp audio file: {self._temp_audio} (exists: {has_audio}, size: {audio_size} bytes)")
        
        if not has_video and not has_audio:
            self.logger.error("No video or audio file to merge")
            return None
        
        # 如果只有视频或只有音频，直接复制
        if has_video and not has_audio:
            self.logger.warning("Only video available, no audio")
            # 检查视频文件大小
            video_size = os.path.getsize(self._temp_video)
            if video_size == 0:
                self.logger.error("Video file is empty (0 bytes), cannot convert")
                return None
            # 转换为 MP4（重新编码以确保兼容性）
            try:
                cmd = [
                    "ffmpeg", "-y",
                    "-v", "info",  # Enable info logs for ffmpeg
                    "-i", self._temp_video,
                    # 视频编码参数（确保兼容性）
                    "-c:v", "libx264",
                    "-pix_fmt", "yuv420p",
                    "-preset", "medium",
                    "-crf", str(self.video_quality),  # 使用配置的质量值
                    "-profile:v", "high",
                    "-level", "4.0",
                    "-movflags", "+faststart",  # 将元数据移到文件开头
                    self._actual_save_path
                ]
                self.logger.info(f"Running ffmpeg video convert cmd: {' '.join(cmd)}")
                result = subprocess.run(cmd, capture_output=True, text=True)
                if result.returncode == 0:
                    self.logger.info(f"Video converted successfully: {self._actual_save_path}")
                    return self._actual_save_path
                else:
                    self.logger.error(f"ffmpeg conversion failed code={result.returncode}")
                    self.logger.error(f"ffmpeg stdout: {result.stdout}")
                    self.logger.error(f"ffmpeg stderr: {result.stderr}")
            except Exception as e:
                self.logger.error(f"Failed to convert video: {e}")
            return None
        
        if has_audio and not has_video:
            self.logger.warning("Only audio available, no video")
            return None
        
        # 合并音视频
        try:
            # 检查实际录制帧率
            if self._video_recorder:
                frame_counts = self._video_recorder.frame_counts
                recorded_fps = frame_counts / elapsed_time if elapsed_time > 0 else self.video_fps
                
                # 如果帧率不匹配，先重新编码视频（使用高质量参数）
                # 注意：如果帧率差异很小（<1%），可以跳过重新编码，让ffmpeg在合并时处理
                fps_diff_percent = abs(recorded_fps - self.video_fps) / self.video_fps * 100
                if fps_diff_percent >= 1.0:  # 帧率差异超过1%才重新编码
                    self.logger.info(f"Re-encoding video (fps: {recorded_fps:.2f} -> {self.video_fps}, diff: {fps_diff_percent:.1f}%)")
                    temp_video2 = self._temp_video.replace(".avi", "_2.avi")
                    # 使用高质量参数重新编码，避免质量损失
                    cmd = [
                        "ffmpeg", "-y",
                        "-r", str(recorded_fps), "-i", self._temp_video,
                        "-c:v", "libx264",
                        "-pix_fmt", "yuv420p",
                        "-preset", "medium",
                        "-crf", str(self.video_quality),  # 使用相同的质量参数
                        "-profile:v", "high",
                        "-level", "4.0",
                        "-r", str(self.video_fps),  # 目标帧率
                        "-vsync", "cfr",  # 恒定帧率
                        temp_video2
                    ]
                    self.logger.info(f"Running ffmpeg re-encode cmd: {' '.join(cmd)}")
                    result = subprocess.run(cmd, capture_output=True, text=True)
                    if result.returncode != 0:
                        self.logger.error(f"Re-encoding failed code={result.returncode}")
                        self.logger.error(f"ffmpeg stderr: {result.stderr}")
                        temp_video2 = self._temp_video
                    else:
                        self.logger.info(f"Video re-encoded successfully with quality CRF={self.video_quality}")
                        temp_video2 = temp_video2
                else:
                    # 帧率差异很小，直接使用原视频，ffmpeg会在合并时处理
                    self.logger.info(f"Frame rate close enough (recorded: {recorded_fps:.2f}, target: {self.video_fps:.2f}, diff: {fps_diff_percent:.1f}%), skipping re-encoding")
                    temp_video2 = self._temp_video
            else:
                temp_video2 = self._temp_video
            
            # 合并音视频（使用改进的同步选项）
            self.logger.info("Merging audio and video with synchronization...")
            
            # 计算启动时间差（用于补偿）
            video_start_time = self._video_recorder.start_time if self._video_recorder else self._start_time
            audio_start_time = self._audio_recorder.start_time if self._audio_recorder else self._start_time
            time_offset = audio_start_time - video_start_time
            
            self.logger.info(f"Start time offset: video={video_start_time:.3f}s, audio={audio_start_time:.3f}s, offset={time_offset*1000:.1f}ms")
            
            # 构建ffmpeg命令，使用更好的同步选项和兼容性参数
            # 如果时间差超过50ms，使用itsoffset补偿
            # 定义构建命令的内部函数
            def build_merge_cmd(encoder_flags):
                # 基础参数
                base_cmd = ["nice", "-n", "19", "ffmpeg", "-y"]
                
                if abs(time_offset) > 0.01:
                    if time_offset > 0:
                        # 视频启动早，延迟视频
                        inputs = [
                            "-v", "info",
                            "-i", self._temp_audio,
                            "-itsoffset", str(time_offset), "-i", temp_video2,
                            "-map", "0:a", "-map", "1:v"
                        ]
                        self.logger.info(f"Applying video delay compensation: {time_offset*1000:.1f}ms")
                    else:
                        # 视频启动晚，延迟音频
                        inputs = [
                            "-itsoffset", str(-time_offset), "-i", self._temp_audio,
                            "-i", temp_video2,
                            "-map", "0:a", "-map", "1:v"
                        ]
                        self.logger.info(f"Applying audio delay compensation: {-time_offset*1000:.1f}ms")
                else:
                    # 直接合并
                    inputs = [
                        "-i", self._temp_audio,
                        "-i", temp_video2,
                    ]
                
                # 公共输出参数
                output_options = [
                    "-movflags", "+faststart",
                    "-c:a", "aac",
                    "-b:a", "192k",
                    "-ac", "2",
                    "-ar", "44100",
                    "-channel_layout", "stereo",
                    "-async", "1",
                    "-vsync", "cfr",
                    "-shortest",
                    self._actual_save_path
                ]
                
                return base_cmd + inputs + encoder_flags + output_options

            # 尝试策略：首先尝试自动检测的编码器，如果失败则回退到 libx264
            encoders_to_try = []
            
            # 1. 优先尝试检测到的最佳编码器
            encoders_to_try.append({
                "name": self._encoder_config['name'],
                "flags": self._encoder_config['flags']
            })
            
            # 2. 如果最佳编码器不是 libx264，则添加 libx264 作为回退
            if self._encoder_config['name'] != "libx264":
                encoders_to_try.append({
                    "name": "libx264_fallback",
                    "flags": [
                        "-c:v", "libx264", 
                        "-preset", "fast", 
                        "-crf", str(self.video_quality),
                        "-profile:v", "high",
                        "-pix_fmt", "yuv420p"
                    ]
                })

            success = False
            for encoder in encoders_to_try:
                self.logger.info(f"Attempting merge with encoder: {encoder['name']}")
                cmd = build_merge_cmd(encoder['flags'])
                
                self.logger.info(f"Running ffmpeg merge cmd: {' '.join(cmd)}")
                result = subprocess.run(cmd, capture_output=True, text=True)
                
                if result.returncode == 0:
                    success = True
                    self.logger.info(f"Merge successful with {encoder['name']}")
                    break
                else:
                    self.logger.warning(f"Merge failed with {encoder['name']} (code={result.returncode})")
                    self.logger.warning(f"ffmpeg stderr: {result.stderr}")
                    # 继续尝试下一个编码器
            
            if not success:
                self.logger.error("All merge attempts failed.")
                return None
            
            # 清理临时文件
            self._cleanup_temp_files()
            
            if os.path.exists(self._actual_save_path):
                return self._actual_save_path
            else:
                return None
                
        except Exception as e:
            self.logger.error(f"Failed to merge audio and video: {e}")
            return None
    
    def _cleanup_temp_files(self):
        """清理临时文件"""
        temp_files = [self._temp_video, self._temp_audio]
        if self._video_recorder:
            temp_files.append(self._temp_video.replace(".avi", "_2.avi"))
        
        for temp_file in temp_files:
            if temp_file and os.path.exists(temp_file):
                try:
                    os.remove(temp_file)
                    self.logger.debug(f"Removed temp file: {temp_file}")
                except Exception as e:
                    self.logger.warning(f"Failed to remove temp file {temp_file}: {e}")
    

    
    @property
    def is_recording(self) -> bool:
        """是否正在录制"""
        return self._is_recording
    
    @property
    def recording_duration(self) -> float:
        """当前录制时长（秒）"""
        if not self._is_recording:
            return 0.0
        return time.time() - self._start_time
    
    def get_current_audio_energy(self) -> float:
        """获取当前音频能量 (0.0 - 1.0)"""
        if self._audio_recorder:
            return self._audio_recorder.get_current_energy()
        return 0.0

    
    @staticmethod
    def check_dependencies() -> bool:
        """
        检查必要的依赖是否已安装
        
        Returns:
            True 如果所有依赖都可用
        """
        issues = []
        
        # 检查 OpenCV
        if not CV_AVAILABLE:
            issues.append("OpenCV (cv2) 未安装")
        
        # 检查 PyAudio
        if not PYAUDIO_AVAILABLE:
            issues.append("PyAudio 未安装")
        
        # 检查 ffmpeg
        try:
            result = subprocess.run(["ffmpeg", "-version"], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                issues.append("ffmpeg 未正确安装")
        except FileNotFoundError:
            issues.append("ffmpeg 未安装（请运行: sudo apt install ffmpeg）")
        
        if issues:
            logging.warning("InterviewRecorder 依赖检查失败:")
            for issue in issues:
                logging.warning(f"  - {issue}")
            return False
        
        return True

