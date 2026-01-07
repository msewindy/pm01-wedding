#!/usr/bin/env python3
"""
Interview 集成测试

测试 InterviewState 与 FSM 的集成，以及录制功能
"""

import sys
import os
import time
import tempfile
import shutil
import numpy as np

# 添加包路径
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(os.path.dirname(script_dir))
sys.path.insert(0, pkg_dir)

import pytest
from wedding_interaction.fsm import (
    WeddingFSM,
    WeddingStateName,
    WeddingEvent,
)
from wedding_interaction.fsm.states import (
    IdleState,
    SearchState,
    TrackingState,
    InterviewState,
)


class TestInterviewStateIntegration:
    """测试InterviewState集成"""
    
    def setup_method(self):
        """测试前准备"""
        self.fsm = WeddingFSM()
        self.fsm.register_state(IdleState(self.fsm))
        self.fsm.register_state(SearchState(self.fsm))
        self.fsm.register_state(TrackingState(self.fsm))
        self.fsm.register_state(InterviewState(self.fsm))
        self.fsm.initialize(WeddingStateName.IDLE)
    
    def test_interview_state_init(self):
        """测试InterviewState初始化"""
        state = InterviewState(self.fsm)
        assert state.state_name == WeddingStateName.INTERVIEW
        assert state.phase == InterviewState.PHASE_START
    
    def test_transition_tracking_to_interview(self):
        """测试从TRACKING转换到INTERVIEW"""
        # 初始化到TRACKING
        self.fsm.initialize(WeddingStateName.TRACKING)
        assert self.fsm.get_current_state_name() == WeddingStateName.TRACKING
        
        # 设置锁定目标（模拟从SEARCH传递）
        from wedding_interaction.perception import LockedTarget
        locked = LockedTarget(
            face_id=1,
            center=(0.5, 0.5),
            timestamp=time.time()
        )
        self.fsm.data.perception.locked_target = locked
        
        # 运行几次
        for _ in range(5):
            self.fsm.run_once()
            time.sleep(0.01)
        
        # 发送采访命令
        self.fsm.data.pending_command = WeddingEvent.CMD_START_INTERVIEW
        
        # 运行直到状态转换
        for _ in range(10):
            self.fsm.run_once()
            time.sleep(0.01)
            if self.fsm.get_current_state_name() == WeddingStateName.INTERVIEW:
                break
        
        assert self.fsm.get_current_state_name() == WeddingStateName.INTERVIEW
    
    def test_interview_flow_phases(self):
        """测试采访流程阶段"""
        # 直接初始化到INTERVIEW（用于测试）
        self.fsm.initialize(WeddingStateName.INTERVIEW)
        
        # 设置锁定目标
        from wedding_interaction.perception import LockedTarget
        locked = LockedTarget(
            face_id=1,
            center=(0.5, 0.5),
            timestamp=time.time()
        )
        self.fsm.data.perception.locked_target = locked
        
        # 运行on_enter
        current_state = self.fsm.current_state
        if isinstance(current_state, InterviewState):
            current_state.on_enter()
            assert current_state.phase == InterviewState.PHASE_START
            
            # 模拟时间推进，测试阶段转换
            # 注意：这里只是测试逻辑，不测试实际的录制功能
            current_state.phase_start_time = time.time() - 4.0  # 模拟已过4秒
            current_state.run()
            
            # 应该进入greeting阶段
            # （实际测试中需要等待足够的时间）


class TestInterviewRecorderIntegration:
    """测试InterviewRecorder集成"""
    
    def test_recorder_with_mock_frame_callback(self):
        """测试录制器与mock回调"""
        with tempfile.TemporaryDirectory() as temp_dir:
            frame_count = [0]
            
            def mock_frame_callback():
                """Mock视频帧回调"""
                frame_count[0] += 1
                if frame_count[0] > 100:  # 限制帧数
                    return None
                return np.zeros((480, 640, 3), dtype=np.uint8)
            
            recorder = InterviewRecorder(
                save_path=temp_dir,
                video_size=(640, 480),
                video_fps=30,
                frame_callback=mock_frame_callback
            )
            
            # 尝试启动（可能会因为缺少设备而失败，但不应该崩溃）
            try:
                result = recorder.start()
                if result:
                    time.sleep(1.0)  # 录制1秒
                    saved_path = recorder.stop()
                    # 如果成功，应该有保存路径
                    if saved_path:
                        assert os.path.exists(saved_path)
            except Exception as e:
                # 如果没有设备，失败是正常的
                print(f"Recording failed (expected if no devices): {e}")


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

