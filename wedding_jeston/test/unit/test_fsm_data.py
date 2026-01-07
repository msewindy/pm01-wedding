"""
FSM 数据结构测试
"""

import pytest
import numpy as np
from wedding_interaction.fsm.wedding_fsm_data import (
    PerceptionData,
    MotionData,
    AudioData,
    WeddingFSMData,
)
from wedding_interaction.fsm.enums import WeddingEvent


class TestPerceptionData:
    """测试感知数据"""
    
    def test_default_values(self):
        """测试默认值"""
        data = PerceptionData()
        
        assert data.face_detected is False
        assert np.allclose(data.face_position, [0.5, 0.5])
        assert data.face_distance == 2.0
        assert data.gesture == "none"
        assert data.gesture_confidence == 0.0
        assert data.too_close is False
    
    def test_reset(self):
        """测试重置"""
        data = PerceptionData()
        
        # 修改数据
        data.face_detected = True
        data.gesture = "heart"
        data.gesture_confidence = 0.9
        
        # 重置
        data.reset()
        
        assert data.face_detected is False
        assert data.gesture == "none"
        assert data.gesture_confidence == 0.0
    
    def test_face_position_update(self):
        """测试人脸位置更新"""
        data = PerceptionData()
        
        data.face_position[0] = 0.3
        data.face_position[1] = 0.7
        
        assert np.allclose(data.face_position, [0.3, 0.7])


class TestMotionData:
    """测试运动数据"""
    
    def test_default_values(self):
        """测试默认值"""
        data = MotionData()
        
        assert len(data.joint_positions) == 24
        assert len(data.joint_commands) == 24
        assert data.current_pose == "neutral"
        assert data.target_pose == "neutral"
        assert np.allclose(data.look_at_target, [0.5, 0.5])
        assert data.pose_in_progress is False
    
    def test_look_at_update(self):
        """测试注视目标更新"""
        data = MotionData()
        
        data.look_at_target[0] = 0.2
        data.look_at_target[1] = 0.8
        
        assert np.allclose(data.look_at_target, [0.2, 0.8])


class TestAudioData:
    """测试音频数据"""
    
    def test_default_values(self):
        """测试默认值"""
        data = AudioData()
        
        assert data.voice_command == "none"
        assert data.is_playing is False
        assert data.current_speech == ""
        assert data.pending_speech == ""
    
    def test_pending_speech(self):
        """测试待播放语音"""
        data = AudioData()
        
        data.pending_speech = "greeting"
        assert data.pending_speech == "greeting"


class TestWeddingFSMData:
    """测试 FSM 数据"""
    
    def test_default_values(self):
        """测试默认值"""
        data = WeddingFSMData()
        
        assert isinstance(data.perception, PerceptionData)
        assert isinstance(data.motion, MotionData)
        assert isinstance(data.audio, AudioData)
        assert data.pending_command is None
        assert data.safety_triggered is False
        assert data.config == {}
    
    def test_reset_pending_command(self):
        """测试重置待处理命令"""
        data = WeddingFSMData()
        
        data.pending_command = WeddingEvent.CMD_GOTO_PHOTO
        assert data.pending_command == WeddingEvent.CMD_GOTO_PHOTO
        
        data.reset_pending_command()
        assert data.pending_command is None
    
    def test_reset_frame_data(self):
        """测试重置单帧数据"""
        data = WeddingFSMData()
        
        # 修改数据
        data.perception.face_detected = True
        data.perception.gesture = "heart"
        data.audio.pending_speech = "greeting"
        
        # 重置
        data.reset_frame_data()
        
        assert data.perception.face_detected is False
        assert data.perception.gesture == "none"
        assert data.audio.pending_speech == ""
    
    def test_config_setting(self):
        """测试配置设置"""
        config = {"param1": 1, "param2": "value"}
        data = WeddingFSMData()
        data.config = config
        
        assert data.config["param1"] == 1
        assert data.config["param2"] == "value"

