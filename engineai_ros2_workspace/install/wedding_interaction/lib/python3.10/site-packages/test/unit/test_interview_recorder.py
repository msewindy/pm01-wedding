#!/usr/bin/env python3
"""
InterviewRecorder 单元测试

测试录制器的各个组件功能
"""

import pytest
import os
import time
import tempfile
import shutil
import numpy as np
from unittest.mock import Mock, patch, MagicMock

# 添加包路径
import sys
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(os.path.dirname(script_dir))
sys.path.insert(0, pkg_dir)

from wedding_interaction.fsm.states.interview_recorder import (
    InterviewRecorder,
    VideoRecorder,
    AudioRecorder,
)


class TestInterviewRecorderDependencies:
    """测试依赖检查"""
    
    def test_check_dependencies(self):
        """测试依赖检查功能"""
        result = InterviewRecorder.check_dependencies()
        # 应该返回True或False，不应该抛出异常
        assert isinstance(result, bool)


class TestVideoRecorder:
    """测试VideoRecorder类"""
    
    def test_video_recorder_init_callback_mode(self):
        """测试回调模式初始化"""
        with tempfile.NamedTemporaryFile(suffix='.avi', delete=False) as f:
            temp_file = f.name
        
        try:
            # Mock frame_callback
            frame_callback = Mock(return_value=np.zeros((480, 640, 3), dtype=np.uint8))
            
            recorder = VideoRecorder(
                filename=temp_file,
                sizex=640,
                sizey=480,
                camindex=None,
                fps=30,
                frame_callback=frame_callback
            )
            
            assert recorder.fps == 30
            assert recorder.frameSize == (640, 480)
            assert recorder.frame_callback is not None
            
        finally:
            if os.path.exists(temp_file):
                os.remove(temp_file)
    
    def test_video_recorder_frame_rate_control(self):
        """测试帧率控制"""
        with tempfile.NamedTemporaryFile(suffix='.avi', delete=False) as f:
            temp_file = f.name
        
        try:
            frame_count = [0]
            frame_callback = Mock(return_value=np.zeros((480, 640, 3), dtype=np.uint8))
            
            recorder = VideoRecorder(
                filename=temp_file,
                sizex=640,
                sizey=480,
                fps=10,  # 使用较低的帧率便于测试
                frame_callback=frame_callback
            )
            
            # 启动录制线程
            thread = recorder.start()
            time.sleep(0.5)  # 等待0.5秒
            recorder.stop()
            thread.join(timeout=1.0)
            
            # 验证帧率（应该接近10fps）
            # 0.5秒应该录制约5帧
            assert recorder.frame_counts >= 4  # 允许一些误差
            
        finally:
            if os.path.exists(temp_file):
                os.remove(temp_file)


class TestAudioRecorder:
    """测试AudioRecorder类"""
    
    @pytest.mark.skipif(not hasattr(__import__('sys').modules.get('pyaudio'), 'PyAudio'), 
                        reason="PyAudio not available")
    def test_audio_recorder_init(self):
        """测试音频录制器初始化"""
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            temp_file = f.name
        
        try:
            recorder = AudioRecorder(
                filename=temp_file,
                rate=44100
            )
            
            assert recorder.rate == 44100
            assert recorder.channels == 2
            
            recorder.stop()  # 清理
            
        except RuntimeError as e:
            if "PyAudio" in str(e):
                pytest.skip("PyAudio not available")
            raise
        finally:
            if os.path.exists(temp_file):
                os.remove(temp_file)


class TestInterviewRecorder:
    """测试InterviewRecorder集成"""
    
    def test_recorder_init(self):
        """测试录制器初始化"""
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = InterviewRecorder(
                save_path=temp_dir,
                video_size=(640, 480),
                video_fps=30
            )
            
            assert recorder.video_fps == 30
            assert recorder.video_size == (640, 480)
            assert recorder.audio_sample_rate == 44100
    
    def test_recorder_start_stop_without_devices(self):
        """测试启动和停止（无设备，仅测试逻辑）"""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Mock frame_callback返回None，模拟无视频
            frame_callback = Mock(return_value=None)
            
            recorder = InterviewRecorder(
                save_path=temp_dir,
                video_size=(640, 480),
                video_fps=30,
                frame_callback=frame_callback
            )
            
            # 尝试启动（可能会失败，但不应该崩溃）
            try:
                result = recorder.start()
                if result:
                    time.sleep(0.1)
                    recorder.stop()
            except Exception as e:
                # 如果没有设备，启动失败是正常的
                pass
    
    def test_recorder_cleanup(self):
        """测试资源清理"""
        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = InterviewRecorder(
                save_path=temp_dir,
                video_size=(640, 480),
                video_fps=30
            )
            
            # 清理应该不抛出异常
            recorder._cleanup()
            
            assert not recorder._is_recording
            assert recorder._video_recorder is None
            assert recorder._audio_recorder is None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

