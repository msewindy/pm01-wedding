"""
枚举测试
"""

import pytest
from wedding_interaction.fsm.enums import (
    WeddingStateName,
    FSMOperatingMode,
    WeddingEvent,
)


class TestWeddingStateName:
    """测试状态名称枚举"""
    
    def test_state_names_exist(self):
        """测试所有状态名称存在"""
        assert WeddingStateName.IDLE is not None
        assert WeddingStateName.TRACKING is not None
        assert WeddingStateName.PHOTO_POSING is not None
        assert WeddingStateName.FAREWELL is not None
        assert WeddingStateName.SAFE_STOP is not None
        assert WeddingStateName.INVALID is not None
    
    def test_state_name_to_string(self):
        """测试状态名称转字符串"""
        assert str(WeddingStateName.IDLE) == "IDLE"
        assert str(WeddingStateName.TRACKING) == "TRACKING"
        assert str(WeddingStateName.PHOTO_POSING) == "PHOTO_POSING"
    
    def test_state_name_uniqueness(self):
        """测试状态名称唯一性"""
        states = [
            WeddingStateName.IDLE,
            WeddingStateName.TRACKING,
            WeddingStateName.PHOTO_POSING,
            WeddingStateName.FAREWELL,
            WeddingStateName.SAFE_STOP,
            WeddingStateName.INVALID,
        ]
        assert len(states) == len(set(states))


class TestFSMOperatingMode:
    """测试操作模式枚举"""
    
    def test_operating_modes_exist(self):
        """测试所有操作模式存在"""
        assert FSMOperatingMode.NORMAL is not None
        assert FSMOperatingMode.TRANSITIONING is not None
        assert FSMOperatingMode.SAFE_STOP is not None


class TestWeddingEvent:
    """测试事件枚举"""
    
    def test_perception_events_exist(self):
        """测试感知事件存在"""
        assert WeddingEvent.FACE_DETECTED is not None
        assert WeddingEvent.FACE_LOST is not None
        assert WeddingEvent.GESTURE_HEART is not None
        assert WeddingEvent.GESTURE_V is not None
        assert WeddingEvent.TOO_CLOSE is not None
    
    def test_command_events_exist(self):
        """测试命令事件存在"""
        assert WeddingEvent.CMD_GOTO_IDLE is not None
        assert WeddingEvent.CMD_GOTO_PHOTO is not None
        assert WeddingEvent.CMD_GOTO_TRACKING is not None
        assert WeddingEvent.CMD_STOP is not None
    
    def test_internal_events_exist(self):
        """测试内部事件存在"""
        assert WeddingEvent.TIMEOUT is not None
        assert WeddingEvent.POSE_COMPLETE is not None
        assert WeddingEvent.FAREWELL_COMPLETE is not None
    
    def test_safety_events_exist(self):
        """测试安全事件存在"""
        assert WeddingEvent.SAFETY_TRIGGER is not None
        assert WeddingEvent.SAFETY_CLEAR is not None

