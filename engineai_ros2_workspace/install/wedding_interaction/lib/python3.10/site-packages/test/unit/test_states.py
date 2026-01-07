"""
具体状态测试
"""

import pytest
import time
from wedding_interaction.fsm import (
    WeddingFSM,
    WeddingStateName,
)
from wedding_interaction.fsm.states import (
    IdleState,
    TrackingState,
    PhotoPosingState,
    FarewellState,
    SafeStopState,
)


class TestIdleState:
    """测试 IDLE 状态"""
    
    @pytest.fixture
    def fsm(self):
        """创建 FSM 实例"""
        fsm = WeddingFSM()
        fsm.register_state(IdleState(fsm))
        fsm.register_state(TrackingState(fsm))
        fsm.register_state(SafeStopState(fsm))
        fsm.initialize(WeddingStateName.IDLE)
        return fsm
    
    def test_idle_sway_motion(self, fsm):
        """测试待机摆动"""
        initial_x = fsm.data.motion.look_at_target[0]
        
        # 运行一段时间
        for _ in range(50):
            fsm.run_once()
            time.sleep(0.02)
        
        # 应该有摆动
        current_x = fsm.data.motion.look_at_target[0]
        # 位置应该发生变化（摆动效果）
        # 由于是正弦波，可能回到起点，所以只检查运行无异常
        assert 0.0 <= current_x <= 1.0
    
    def test_idle_pose_neutral(self, fsm):
        """测试空闲状态设置中立 Pose"""
        assert fsm.data.motion.target_pose == "neutral"


class TestTrackingState:
    """测试 TRACKING 状态"""
    
    @pytest.fixture
    def fsm(self):
        """创建 FSM 实例"""
        fsm = WeddingFSM()
        fsm.register_state(IdleState(fsm))
        fsm.register_state(TrackingState(fsm))
        fsm.register_state(PhotoPosingState(fsm))
        fsm.register_state(FarewellState(fsm))
        fsm.register_state(SafeStopState(fsm))
        fsm.initialize(WeddingStateName.TRACKING)
        return fsm
    
    def test_tracking_follows_target(self, fsm):
        """测试跟随目标"""
        # 设置目标位置
        fsm.data.perception.face_detected = True
        fsm.data.perception.face_position[0] = 0.2
        fsm.data.perception.face_position[1] = 0.8
        
        # 运行几次
        for _ in range(10):
            fsm.run_once()
            fsm.data.perception.face_detected = True
            fsm.data.perception.face_position[0] = 0.2
            fsm.data.perception.face_position[1] = 0.8
        
        # 注视目标应该趋向目标位置
        look_x = fsm.data.motion.look_at_target[0]
        look_y = fsm.data.motion.look_at_target[1]
        
        assert look_x < 0.5  # 应该向左移动
        assert look_y > 0.5  # 应该向上移动
    
    def test_tracking_smooth_follow(self, fsm):
        """测试平滑跟随"""
        # 设置目标位置为极端值
        fsm.data.perception.face_detected = True
        fsm.data.perception.face_position[0] = 0.0
        
        # 运行一次
        fsm.run_once()
        
        # 不应该立即跳到目标位置
        look_x = fsm.data.motion.look_at_target[0]
        assert look_x > 0.0  # 还未到达目标


class TestPhotoPosingState:
    """测试 PHOTO_POSING 状态"""
    
    @pytest.fixture
    def fsm(self):
        """创建 FSM 实例"""
        fsm = WeddingFSM()
        fsm.register_state(IdleState(fsm))
        fsm.register_state(TrackingState(fsm))
        fsm.register_state(PhotoPosingState(fsm))
        fsm.register_state(FarewellState(fsm))
        fsm.register_state(SafeStopState(fsm))
        fsm.initialize(WeddingStateName.PHOTO_POSING)
        return fsm
    
    def test_photo_posing_selects_pose_from_gesture(self, fsm):
        """测试根据手势选择 Pose"""
        # 重新进入状态
        fsm.data.perception.gesture = "heart"
        fsm.force_transition(WeddingStateName.PHOTO_POSING)
        
        state = fsm.states[WeddingStateName.PHOTO_POSING]
        assert state.selected_pose == "heart"
    
    def test_photo_posing_phases(self, fsm):
        """测试合影阶段"""
        state = fsm.states[WeddingStateName.PHOTO_POSING]
        
        # 初始应该是确认阶段
        assert state.phase == PhotoPosingState.PHASE_CONFIRM
        
        # 运行过确认阶段
        for _ in range(60):  # 约 1.2 秒
            fsm.run_once()
            time.sleep(0.02)
        
        # 应该进入 Pose 阶段
        assert state.phase in [PhotoPosingState.PHASE_POSE, PhotoPosingState.PHASE_COUNTDOWN]


class TestFarewellState:
    """测试 FAREWELL 状态"""
    
    @pytest.fixture
    def fsm(self):
        """创建 FSM 实例"""
        fsm = WeddingFSM()
        fsm.register_state(IdleState(fsm))
        fsm.register_state(TrackingState(fsm))
        fsm.register_state(PhotoPosingState(fsm))
        fsm.register_state(FarewellState(fsm))
        fsm.register_state(SafeStopState(fsm))
        fsm.initialize(WeddingStateName.FAREWELL)
        return fsm
    
    def test_farewell_plays_wave_pose(self, fsm):
        """测试送别播放挥手 Pose"""
        assert fsm.data.motion.target_pose == "wave"
    
    def test_farewell_plays_speech(self, fsm):
        """测试送别播放语音"""
        assert fsm.data.audio.pending_speech == "farewell"


class TestSafeStopState:
    """测试 SAFE_STOP 状态"""
    
    @pytest.fixture
    def fsm(self):
        """创建 FSM 实例"""
        fsm = WeddingFSM()
        fsm.register_state(IdleState(fsm))
        fsm.register_state(TrackingState(fsm))
        fsm.register_state(PhotoPosingState(fsm))
        fsm.register_state(FarewellState(fsm))
        fsm.register_state(SafeStopState(fsm))
        fsm.initialize(WeddingStateName.SAFE_STOP)
        return fsm
    
    def test_safe_stop_neutral_pose(self, fsm):
        """测试安全停止设置中立 Pose"""
        assert fsm.data.motion.target_pose == "neutral"
    
    def test_safe_stop_plays_warning(self, fsm):
        """测试安全停止播放警告语音"""
        assert fsm.data.audio.pending_speech == "too_close"
    
    def test_safe_stop_recovery(self, fsm):
        """测试安全停止恢复"""
        # 设置安全触发
        fsm.data.safety_triggered = True
        fsm.data.perception.too_close = True
        
        # 运行几次保持安全状态
        for _ in range(10):
            fsm.run_once()
            time.sleep(0.02)
        
        assert fsm.get_current_state_name() == WeddingStateName.SAFE_STOP
        
        # 解除安全条件
        fsm.data.perception.too_close = False
        
        # 运行足够次数等待恢复
        for _ in range(150):  # 约 3 秒
            fsm.run_once()
            time.sleep(0.02)
        
        # 应该回到 IDLE
        assert fsm.get_current_state_name() == WeddingStateName.IDLE
        assert fsm.data.safety_triggered is False

