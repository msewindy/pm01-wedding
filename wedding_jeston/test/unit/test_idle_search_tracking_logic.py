#!/usr/bin/env python3
"""
IDLE → SEARCH → TRACKING 逻辑测试

测试修改后的代码逻辑：
1. IDLE 有候选时聚焦到候选
2. IDLE → SEARCH 传递候选
3. SEARCH 使用 IDLE 的候选
4. TRACKING 跟踪特定对象
"""

import unittest
import time
from unittest.mock import Mock, MagicMock

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wedding_interaction.fsm import WeddingFSM, WeddingStateName
from wedding_interaction.fsm.states import IdleState, SearchState, TrackingState
from wedding_interaction.perception import FaceInfo, LockedTarget, FaceTracker


class TestIdleStateFocus(unittest.TestCase):
    """测试 IDLE 状态聚焦逻辑"""
    
    def setUp(self):
        self.fsm = Mock(spec=WeddingFSM)
        self.fsm.data = Mock()
        self.fsm.data.config = {'enable_action': True, 'enable_speech': True}
        self.fsm.data.motion = Mock()
        self.fsm.data.motion.look_at_target = [0.5, 0.5]
        self.fsm.data.perception = Mock()
        self.fsm.data.perception.faces = []
        self.fsm.data.perception.locked_target = None
        self.fsm.data.safety_triggered = False  # 重要：设置安全标志
        self.fsm.data.pending_command = None  # 重要：设置命令
        self.fsm.get_current_time = Mock(return_value=time.time())
        
        self.idle_state = IdleState(self.fsm)
        self.idle_state.set_look_at = Mock()
        self.idle_state.set_pose = Mock()
        self.idle_state.set_speech = Mock()
        self.idle_state.log = Mock()
    
    def test_no_candidate_executes_action_pack(self):
        """测试：无候选时执行 action pack"""
        self.idle_state._candidate_face = None
        self.idle_state._current_pack = Mock()
        self.idle_state._current_pack.get_head_offset = Mock(return_value=0.1)
        self.idle_state._pack_start_time = time.time()
        
        self.idle_state.run()
        
        # 应该调用 set_look_at（来自 action pack）
        self.idle_state.set_look_at.assert_called()
    
    def test_has_candidate_focuses_on_candidate(self):
        """测试：有候选时聚焦到候选"""
        # 创建候选
        candidate = FaceInfo(
            x=0.3, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=1
        )
        self.idle_state._candidate_face = candidate
        self.fsm.data.motion.look_at_target = [0.5, 0.5]  # 当前在中心
        
        self.idle_state.run()
        
        # 应该调用 _focus_on_candidate，设置 look_at 到候选位置
        self.idle_state.set_look_at.assert_called()
        call_args = self.idle_state.set_look_at.call_args[0]
        
        # 检查是否向候选位置移动
        target_x = candidate.center_x  # 0.4
        target_y = candidate.center_y  # 0.425
        
        # 由于平滑（0.1），新位置应该在当前(0.5, 0.5)和目标(0.4, 0.425)之间
        new_x, new_y = call_args
        self.assertLess(new_x, 0.5)     # 向 0.4 移动，应该小于 0.5
        self.assertGreater(new_x, 0.4)  # 但不会立即到达 0.4
        self.assertLess(new_y, 0.5)     # 向 0.425 移动
        self.assertGreater(new_y, 0.425)  # 但不会立即到达 0.425
    
    def test_candidate_confirmed_saves_locked_target(self):
        """测试：候选确认时保存 LockedTarget"""
        candidate = FaceInfo(
            x=0.3, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=1
        )
        self.idle_state._candidate_face = candidate
        self.idle_state._candidate_start_time = time.time() - 0.4  # 超过 0.3s
        
        # 设置 faces，确保匹配成功（避免候选被重置）
        self.fsm.data.perception.faces = [candidate]
        
        next_state = self.idle_state.check_transition()
        
        # 应该进入 SEARCH
        self.assertEqual(next_state, WeddingStateName.SEARCH)
        # 应该保存 locked_target
        self.assertIsNotNone(self.fsm.data.perception.locked_target)
        self.assertEqual(self.fsm.data.perception.locked_target.face_id, 1)


class TestSearchStateUsesIdleCandidate(unittest.TestCase):
    """测试 SEARCH 状态使用 IDLE 候选"""
    
    def setUp(self):
        self.fsm = Mock(spec=WeddingFSM)
        self.fsm.data = Mock()
        self.fsm.data.perception = Mock()
        self.fsm.data.perception.locked_target = None
        self.fsm.data.safety_triggered = False
        self.fsm.data.pending_command = None
        self.fsm.get_current_time = Mock(return_value=time.time())
        
        self.search_state = SearchState(self.fsm)
        self.search_state.set_look_at = Mock()
        self.search_state.set_pose = Mock()
        self.search_state.log = Mock()
    
    def test_uses_idle_candidate(self):
        """测试：使用 IDLE 传递的候选"""
        # 创建 LockedTarget（从 IDLE 传递）
        locked_target = LockedTarget(
            face_id=1,
            bbox=(0.3, 0.3, 0.2, 0.25),
            center=(0.4, 0.425),
            lock_time=time.time()
        )
        self.fsm.data.perception.locked_target = locked_target
        
        self.search_state.on_enter()
        
        # 检查跟踪器是否设置了初始目标
        tracker = self.search_state._get_tracker()
        self.assertIsNotNone(tracker._target)
        self.assertEqual(tracker._target.face_id, 1)
        # center_x 是计算属性：x + width/2 = 0.3 + 0.2/2 = 0.4
        expected_center_x = 0.3 + 0.2 / 2
        self.assertAlmostEqual(tracker._target.center_x, expected_center_x, places=2)
    
    def test_no_candidate_resets_tracker(self):
        """测试：没有候选时重置跟踪器"""
        self.fsm.data.perception.locked_target = None
        
        self.search_state.on_enter()
        
        tracker = self.search_state._get_tracker()
        self.assertIsNone(tracker._target)


class TestFaceTrackerInitialTarget(unittest.TestCase):
    """测试 FaceTracker 初始目标设置"""
    
    def setUp(self):
        self.tracker = FaceTracker()
    
    def test_set_initial_target_from_locked_target(self):
        """测试：从 LockedTarget 设置初始目标"""
        locked_target = LockedTarget(
            face_id=1,
            bbox=(0.3, 0.3, 0.2, 0.25),
            center=(0.4, 0.425),
            lock_time=time.time()
        )
        
        self.tracker.set_initial_target(locked_target)
        
        self.assertIsNotNone(self.tracker._target)
        self.assertEqual(self.tracker._target.face_id, 1)
        # center_x 和 center_y 是计算属性：x + width/2, y + height/2
        expected_center_x = 0.3 + 0.2 / 2  # 0.4
        expected_center_y = 0.3 + 0.25 / 2  # 0.425
        self.assertAlmostEqual(self.tracker._target.center_x, expected_center_x, places=2)
        self.assertAlmostEqual(self.tracker._target.center_y, expected_center_y, places=2)
    
    def test_set_initial_target_from_face_info(self):
        """测试：从 FaceInfo 设置初始目标"""
        face = FaceInfo(
            x=0.3, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=1
        )
        
        self.tracker.set_initial_target(face)
        
        self.assertEqual(self.tracker._target, face)
        self.assertEqual(self.tracker._target.face_id, 1)
    
    def test_match_allows_side_face(self):
        """测试：匹配允许侧脸"""
        # 设置初始目标（正脸）
        target = FaceInfo(
            x=0.3, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=1
        )
        self.tracker.set_initial_target(target)
        
        # 创建侧脸（位置接近）
        side_face = FaceInfo(
            x=0.32, y=0.32, width=0.18, height=0.23,
            is_frontal=False, face_id=1,  # 同一个 face_id
            yaw=45.0  # 侧脸
        )
        
        faces = [side_face]
        matched = self.tracker._select_or_keep_target(faces)
        
        # 应该匹配到侧脸
        self.assertIsNotNone(matched)
        self.assertFalse(matched.is_frontal)  # 是侧脸
        self.assertEqual(matched.face_id, 1)


class TestTrackingStateSpecificTarget(unittest.TestCase):
    """测试 TRACKING 状态跟踪特定对象"""
    
    def setUp(self):
        self.fsm = Mock(spec=WeddingFSM)
        self.fsm.data = Mock()
        self.fsm.data.perception = Mock()
        self.fsm.data.perception.faces = []
        self.fsm.data.perception.locked_target = None
        self.fsm.data.motion = Mock()
        self.fsm.data.motion.look_at_target = [0.5, 0.5]
        self.fsm.get_current_time = Mock(return_value=time.time())
        
        self.tracking_state = TrackingState(self.fsm)
        self.tracking_state.set_look_at = Mock()
        self.tracking_state.set_speech = Mock()
        self.tracking_state.log = Mock()
    
    def test_tracks_specific_face_by_id(self):
        """测试：通过 face_id 跟踪特定对象"""
        # 设置锁定目标
        locked_target = LockedTarget(
            face_id=1,
            bbox=(0.3, 0.3, 0.2, 0.25),
            center=(0.4, 0.425),
            lock_time=time.time()
        )
        self.fsm.data.perception.locked_target = locked_target
        
        self.tracking_state.on_enter()
        
        # 创建多个人脸
        target_face = FaceInfo(
            x=0.3, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=1
        )
        other_face = FaceInfo(
            x=0.6, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=2
        )
        self.fsm.data.perception.faces = [target_face, other_face]
        
        self.tracking_state.run()
        
        # 应该跟随 face_id=1 的目标，而不是 face_id=2
        self.tracking_state.set_look_at.assert_called()
        call_args = self.tracking_state.set_look_at.call_args[0]
        new_x, new_y = call_args
        
        # 应该向 target_face 的位置移动（0.4, 0.425），而不是 other_face（0.7, 0.425）
        self.assertLess(new_x, 0.6)  # 应该更接近 0.4，而不是 0.7
    
    def test_tracks_specific_face_by_position(self):
        """测试：通过位置匹配跟踪特定对象"""
        # 设置锁定目标（face_id 可能不稳定）
        locked_target = LockedTarget(
            face_id=-1,  # 无效的 face_id
            bbox=(0.3, 0.3, 0.2, 0.25),
            center=(0.4, 0.425),
            lock_time=time.time()
        )
        self.fsm.data.perception.locked_target = locked_target
        
        self.tracking_state.on_enter()
        
        # 创建多个人脸（位置接近目标）
        target_face = FaceInfo(
            x=0.32, y=0.32, width=0.18, height=0.23,
            is_frontal=True, face_id=5  # 不同的 face_id
        )
        far_face = FaceInfo(
            x=0.7, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=6
        )
        self.fsm.data.perception.faces = [target_face, far_face]
        
        self.tracking_state.run()
        
        # 应该跟随位置接近的目标（target_face），而不是 far_face
        self.tracking_state.set_look_at.assert_called()
        call_args = self.tracking_state.set_look_at.call_args[0]
        new_x, new_y = call_args
        
        # 应该向 target_face 的位置移动（约 0.41, 0.435），而不是 far_face（0.8, 0.425）
        self.assertLess(new_x, 0.6)
    
    def test_allows_side_face_tracking(self):
        """测试：允许跟踪侧脸"""
        locked_target = LockedTarget(
            face_id=1,
            bbox=(0.3, 0.3, 0.2, 0.25),
            center=(0.4, 0.425),
            lock_time=time.time()
        )
        self.fsm.data.perception.locked_target = locked_target
        
        self.tracking_state.on_enter()
        
        # 创建侧脸（同一个 face_id）
        side_face = FaceInfo(
            x=0.32, y=0.32, width=0.18, height=0.23,
            is_frontal=False, face_id=1,  # 侧脸但 face_id 匹配
            yaw=45.0
        )
        self.fsm.data.perception.faces = [side_face]
        
        self.tracking_state.run()
        
        # 应该仍然跟随（允许侧脸）
        self.tracking_state.set_look_at.assert_called()
        self.assertEqual(self.tracking_state.lost_time, 0.0)  # 没有丢失
    
    def test_lost_when_no_match(self):
        """测试：没有匹配时目标丢失"""
        locked_target = LockedTarget(
            face_id=1,
            bbox=(0.3, 0.3, 0.2, 0.25),
            center=(0.4, 0.425),
            lock_time=time.time()
        )
        self.fsm.data.perception.locked_target = locked_target
        
        self.tracking_state.on_enter()
        
        # 创建不匹配的人脸
        other_face = FaceInfo(
            x=0.7, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=2  # 不同的 face_id 和位置
        )
        self.fsm.data.perception.faces = [other_face]
        
        initial_lost_time = self.tracking_state.lost_time
        self.tracking_state.run()
        
        # 应该累计丢失时间
        self.assertGreater(self.tracking_state.lost_time, initial_lost_time)


class TestFullFlow(unittest.TestCase):
    """测试完整流程：IDLE → SEARCH → TRACKING"""
    
    def setUp(self):
        self.fsm = Mock(spec=WeddingFSM)
        self.fsm.data = Mock()
        self.fsm.data.config = {'enable_action': True, 'enable_speech': True}
        self.fsm.data.motion = Mock()
        self.fsm.data.motion.look_at_target = [0.5, 0.5]
        self.fsm.data.perception = Mock()
        self.fsm.data.perception.faces = []
        self.fsm.data.perception.locked_target = None
        self.fsm.data.safety_triggered = False
        self.fsm.data.pending_command = None
        self.fsm.get_current_time = Mock(return_value=time.time())
    
    def test_idle_to_search_passes_candidate(self):
        """测试：IDLE → SEARCH 传递候选"""
        # IDLE 状态
        idle_state = IdleState(self.fsm)
        idle_state.set_look_at = Mock()
        idle_state.set_pose = Mock()
        idle_state.log = Mock()
        
        # 创建候选并确认
        candidate = FaceInfo(
            x=0.3, y=0.3, width=0.2, height=0.25,
            is_frontal=True, face_id=1
        )
        idle_state._candidate_face = candidate
        idle_state._candidate_start_time = time.time() - 0.4
        
        # 设置 faces，确保匹配成功（避免候选被重置）
        self.fsm.data.perception.faces = [candidate]
        
        # 检查转换
        next_state = idle_state.check_transition()
        self.assertEqual(next_state, WeddingStateName.SEARCH)
        
        # 检查 locked_target 是否保存
        self.assertIsNotNone(self.fsm.data.perception.locked_target)
        self.assertEqual(self.fsm.data.perception.locked_target.face_id, 1)
        
        # SEARCH 状态
        search_state = SearchState(self.fsm)
        search_state.set_look_at = Mock()
        search_state.set_pose = Mock()
        search_state.log = Mock()
        
        search_state.on_enter()
        
        # 检查跟踪器是否使用了 IDLE 的候选
        tracker = search_state._get_tracker()
        self.assertIsNotNone(tracker._target)
        self.assertEqual(tracker._target.face_id, 1)
    
    def test_search_to_tracking_passes_locked_target(self):
        """测试：SEARCH → TRACKING 传递锁定目标"""
        # SEARCH 状态
        search_state = SearchState(self.fsm)
        search_state.set_look_at = Mock()
        search_state.set_pose = Mock()
        search_state.log = Mock()
        
        # 设置锁定目标
        locked_target = LockedTarget(
            face_id=1,
            bbox=(0.3, 0.3, 0.2, 0.25),
            center=(0.4, 0.425),
            lock_time=time.time()
        )
        self.fsm.data.perception.locked_target = locked_target
        search_state.on_enter()
        
        # 模拟正脸持续 2s
        tracker = search_state._get_tracker()
        tracker._frontal_start_time = time.time() - 2.1
        tracker._frontal_duration = 2.1
        
        # 检查转换
        next_state = search_state.check_transition()
        self.assertEqual(next_state, WeddingStateName.TRACKING)
        
        # TRACKING 状态
        tracking_state = TrackingState(self.fsm)
        tracking_state.set_look_at = Mock()
        tracking_state.set_speech = Mock()
        tracking_state.log = Mock()
        
        tracking_state.on_enter()
        
        # 检查是否保存了跟踪信息
        self.assertEqual(tracking_state._tracked_face_id, 1)
        self.assertIsNotNone(tracking_state._tracked_position)


if __name__ == '__main__':
    unittest.main()

