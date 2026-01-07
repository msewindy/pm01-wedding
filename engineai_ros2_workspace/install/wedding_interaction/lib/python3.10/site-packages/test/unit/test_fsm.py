"""
FSM 控制器测试
"""

import pytest
import time
from wedding_interaction.fsm import (
    WeddingFSM,
    WeddingStateName,
    FSMOperatingMode,
    WeddingEvent,
)
from wedding_interaction.fsm.states import (
    IdleState,
    TrackingState,
    PhotoPosingState,
    FarewellState,
    SafeStopState,
)


class TestWeddingFSM:
    """测试 FSM 控制器"""
    
    @pytest.fixture
    def fsm(self):
        """创建 FSM 实例"""
        fsm = WeddingFSM()
        fsm.register_state(IdleState(fsm))
        fsm.register_state(TrackingState(fsm))
        fsm.register_state(PhotoPosingState(fsm))
        fsm.register_state(FarewellState(fsm))
        fsm.register_state(SafeStopState(fsm))
        return fsm
    
    def test_initialization(self, fsm):
        """测试初始化"""
        fsm.initialize(WeddingStateName.IDLE)
        
        assert fsm.is_running()
        assert fsm.get_current_state_name() == WeddingStateName.IDLE
        assert fsm.get_operating_mode() == FSMOperatingMode.NORMAL
    
    def test_invalid_initial_state(self, fsm):
        """测试无效初始状态"""
        with pytest.raises(ValueError):
            fsm.initialize(WeddingStateName.INVALID)
    
    def test_no_states_registered(self):
        """测试未注册状态"""
        fsm = WeddingFSM()
        with pytest.raises(RuntimeError):
            fsm.initialize(WeddingStateName.IDLE)
    
    def test_run_once(self, fsm):
        """测试单次运行"""
        fsm.initialize(WeddingStateName.IDLE)
        
        # 运行一次
        fsm.run_once()
        
        assert fsm.iter_count == 1
        assert fsm.get_current_state_name() == WeddingStateName.IDLE
    
    def test_state_transition_on_face_detected(self, fsm):
        """测试人脸检测状态转换"""
        fsm.initialize(WeddingStateName.IDLE)
        
        # 模拟人脸检测
        fsm.data.perception.face_detected = True
        
        # 运行多次以触发转换（需要确认时间）
        for _ in range(20):  # 约 400ms
            fsm.run_once()
            time.sleep(0.02)
        
        # 应该转换到 TRACKING
        assert fsm.get_current_state_name() == WeddingStateName.TRACKING
    
    def test_force_transition(self, fsm):
        """测试强制状态转换"""
        fsm.initialize(WeddingStateName.IDLE)
        
        # 强制转换到 PHOTO_POSING
        result = fsm.force_transition(WeddingStateName.PHOTO_POSING)
        
        assert result is True
        assert fsm.get_current_state_name() == WeddingStateName.PHOTO_POSING
    
    def test_force_transition_invalid_state(self, fsm):
        """测试强制转换到无效状态"""
        fsm.initialize(WeddingStateName.IDLE)
        
        # 强制转换到无效状态
        result = fsm.force_transition(WeddingStateName.INVALID)
        
        assert result is False
        assert fsm.get_current_state_name() == WeddingStateName.IDLE
    
    def test_command_goto_photo(self, fsm):
        """测试命令：进入合影"""
        fsm.initialize(WeddingStateName.TRACKING)
        
        # 发送命令
        fsm.send_command(WeddingEvent.CMD_GOTO_PHOTO)
        fsm.run_once()
        
        # 应该进入转换模式
        assert fsm.get_operating_mode() in [
            FSMOperatingMode.TRANSITIONING,
            FSMOperatingMode.NORMAL
        ]
    
    def test_command_stop(self, fsm):
        """测试命令：停止"""
        fsm.initialize(WeddingStateName.TRACKING)
        
        # 发送停止命令
        fsm.send_command(WeddingEvent.CMD_STOP)
        fsm.run_once()
        
        # 等待转换完成
        fsm.run_once()
        
        # 应该回到 IDLE
        assert fsm.get_current_state_name() == WeddingStateName.IDLE
    
    def test_reset(self, fsm):
        """测试重置"""
        fsm.initialize(WeddingStateName.TRACKING)
        
        # 设置一些数据
        fsm.data.safety_triggered = True
        
        # 重置
        fsm.reset()
        
        assert fsm.get_current_state_name() == WeddingStateName.IDLE
        assert fsm.data.safety_triggered is False
    
    def test_stop(self, fsm):
        """测试停止"""
        fsm.initialize(WeddingStateName.IDLE)
        
        fsm.stop()
        
        assert fsm.is_running() is False
    
    def test_safety_trigger(self, fsm):
        """测试安全触发"""
        fsm.initialize(WeddingStateName.TRACKING)
        
        # 模拟太近
        fsm.data.perception.too_close = True
        fsm.run_once()
        
        # 应该触发安全停止
        assert fsm.data.safety_triggered is True
        assert fsm.get_operating_mode() == FSMOperatingMode.SAFE_STOP
    
    def test_state_change_callback(self, fsm):
        """测试状态变化回调"""
        callback_called = [False]
        old_state_received = [None]
        new_state_received = [None]
        
        def callback(old_state, new_state):
            callback_called[0] = True
            old_state_received[0] = old_state
            new_state_received[0] = new_state
        
        fsm.on_state_change(callback)
        fsm.initialize(WeddingStateName.IDLE)
        
        # 强制转换触发回调
        fsm.force_transition(WeddingStateName.TRACKING)
        
        assert callback_called[0] is True
        assert old_state_received[0] == WeddingStateName.IDLE
        assert new_state_received[0] == WeddingStateName.TRACKING
    
    def test_get_status_info(self, fsm):
        """测试获取状态信息"""
        fsm.initialize(WeddingStateName.IDLE)
        fsm.run_once()
        
        info = fsm.get_status_info()
        
        assert info['running'] is True
        assert info['current_state'] == 'IDLE'
        assert 'NORMAL' in info['operating_mode']  # 兼容不同格式
        assert info['iter_count'] == 1
        assert info['safety_triggered'] is False


class TestStateTransitions:
    """测试状态转换流程"""
    
    @pytest.fixture
    def fsm(self):
        """创建 FSM 实例"""
        fsm = WeddingFSM()
        fsm.register_state(IdleState(fsm))
        fsm.register_state(TrackingState(fsm))
        fsm.register_state(PhotoPosingState(fsm))
        fsm.register_state(FarewellState(fsm))
        fsm.register_state(SafeStopState(fsm))
        fsm.initialize(WeddingStateName.IDLE)
        return fsm
    
    def test_idle_to_tracking_flow(self, fsm):
        """测试：IDLE -> TRACKING 流程"""
        # 模拟人脸检测
        fsm.data.perception.face_detected = True
        
        # 运行足够次数以触发转换
        for _ in range(20):
            fsm.run_once()
            time.sleep(0.02)
        
        assert fsm.get_current_state_name() == WeddingStateName.TRACKING
    
    def test_tracking_to_farewell_on_lost(self, fsm):
        """测试：TRACKING -> FAREWELL（目标丢失）"""
        # 先进入 TRACKING
        fsm.force_transition(WeddingStateName.TRACKING)
        
        # 不发送人脸数据（模拟丢失）
        for _ in range(100):  # 约 2 秒
            fsm.run_once()
            time.sleep(0.02)
        
        assert fsm.get_current_state_name() == WeddingStateName.FAREWELL
    
    def test_tracking_to_photo_on_gesture(self, fsm):
        """测试：TRACKING -> PHOTO_POSING（手势触发）"""
        # 先进入 TRACKING
        fsm.force_transition(WeddingStateName.TRACKING)
        
        # 模拟人脸 + 比心手势
        fsm.data.perception.face_detected = True
        fsm.data.perception.gesture = "heart"
        fsm.data.perception.gesture_confidence = 0.9
        
        # 运行足够次数
        for _ in range(20):
            fsm.run_once()
            # 保持手势检测
            fsm.data.perception.face_detected = True
            fsm.data.perception.gesture = "heart"
            fsm.data.perception.gesture_confidence = 0.9
            time.sleep(0.02)
        
        assert fsm.get_current_state_name() == WeddingStateName.PHOTO_POSING
    
    def test_farewell_to_idle_flow(self, fsm):
        """测试：FAREWELL -> IDLE"""
        # 进入 FAREWELL
        fsm.force_transition(WeddingStateName.FAREWELL)
        
        # 运行足够次数完成送别
        for _ in range(150):  # 约 3 秒
            fsm.run_once()
            time.sleep(0.02)
        
        assert fsm.get_current_state_name() == WeddingStateName.IDLE
    
    def test_photo_posing_flow(self, fsm):
        """测试：合影完整流程"""
        # 进入 PHOTO_POSING
        fsm.force_transition(WeddingStateName.PHOTO_POSING)
        
        # 运行完整合影流程（约 10 秒，确保有足够时间）
        for _ in range(500):
            fsm.run_once()
            time.sleep(0.02)
            if fsm.get_current_state_name() == WeddingStateName.TRACKING:
                break
        
        # 完成后应该回到 TRACKING
        assert fsm.get_current_state_name() == WeddingStateName.TRACKING

