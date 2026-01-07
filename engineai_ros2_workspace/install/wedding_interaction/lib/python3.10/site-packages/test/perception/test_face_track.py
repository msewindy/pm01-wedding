#!/usr/bin/env python3
"""
人脸追踪功能测试

测试 FaceIDTracker、FaceDetector、IDLE 和 SEARCH 状态的人脸追踪功能
使用图片 mp4.jpg 进行测试
"""

import unittest
import sys
import os
import time
import numpy as np
from typing import List

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

# 项目根目录（engine_ai）
engine_ai_root = os.path.dirname(project_root)  # 从 wedding_jeston 回到 engine_ai

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Warning: OpenCV not available")

from wedding_interaction.perception import (
    FaceDetector, FaceTracker, FaceIDTracker, 
    FaceInfo, LockedTarget
)
from wedding_interaction.fsm.states.idle_state import IdleState
from wedding_interaction.fsm.states.search_state import SearchState
from wedding_interaction.fsm.wedding_fsm import WeddingFSM
from wedding_interaction.fsm.wedding_fsm_data import WeddingFSMData
from wedding_interaction.fsm.enums import WeddingStateName


class TestFaceIDTracker(unittest.TestCase):
    """FaceIDTracker 单元测试"""
    
    def setUp(self):
        self.tracker = FaceIDTracker()
    
    def test_basic_id_assignment(self):
        """测试基本 ID 分配"""
        # 创建模拟人脸
        faces = [
            FaceInfo(x=0.2, y=0.2, width=0.15, height=0.2, confidence=0.9),
            FaceInfo(x=0.6, y=0.3, width=0.12, height=0.18, confidence=0.85),
        ]
        
        # 第一次更新
        result1 = self.tracker.update(faces)
        
        # 验证 ID 已分配
        self.assertGreaterEqual(result1[0].face_id, 0)
        self.assertGreaterEqual(result1[1].face_id, 0)
        self.assertNotEqual(result1[0].face_id, result1[1].face_id)
        
        # 第二次更新（相同位置）
        result2 = self.tracker.update(faces)
        
        # 验证 ID 保持一致
        self.assertEqual(result1[0].face_id, result2[0].face_id)
        self.assertEqual(result1[1].face_id, result2[1].face_id)
    
    def test_cross_frame_matching(self):
        """测试跨帧匹配"""
        # 第一帧
        faces1 = [
            FaceInfo(x=0.2, y=0.2, width=0.15, height=0.2, confidence=0.9),
        ]
        result1 = self.tracker.update(faces1)
        face_id = result1[0].face_id
        
        # 第二帧（位置稍微移动）
        faces2 = [
            FaceInfo(x=0.22, y=0.21, width=0.15, height=0.2, confidence=0.9),
        ]
        result2 = self.tracker.update(faces2)
        
        # 验证 ID 匹配
        self.assertEqual(result2[0].face_id, face_id)
    
    def test_multiple_targets(self):
        """测试多目标追踪"""
        # 第一帧：3个人脸
        faces1 = [
            FaceInfo(x=0.1, y=0.1, width=0.1, height=0.15, confidence=0.9),
            FaceInfo(x=0.4, y=0.2, width=0.12, height=0.18, confidence=0.85),
            FaceInfo(x=0.7, y=0.3, width=0.1, height=0.15, confidence=0.8),
        ]
        result1 = self.tracker.update(faces1)
        
        ids = [f.face_id for f in result1]
        # 验证 ID 不重复
        self.assertEqual(len(ids), len(set(ids)))
        
        # 第二帧：相同人脸
        result2 = self.tracker.update(faces1)
        
        # 验证 ID 保持一致
        for i in range(len(result1)):
            self.assertEqual(result1[i].face_id, result2[i].face_id)
    
    def test_track_cleanup(self):
        """测试目标丢失和清理"""
        # 第一帧
        faces1 = [
            FaceInfo(x=0.2, y=0.2, width=0.15, height=0.2, confidence=0.9),
        ]
        result1 = self.tracker.update(faces1)
        face_id = result1[0].face_id
        
        # 多帧无检测（模拟丢失），并等待足够时间
        import time
        start_time = time.time()
        for _ in range(35):  # 模拟多帧无检测
            self.tracker.update([])
            time.sleep(0.03)  # 确保时间超过 MAX_TIME_SINCE_SEEN (5.0秒)
        
        # 等待足够时间确保清理
        elapsed = time.time() - start_time
        if elapsed < 1.1:  # 确保超过 MAX_TIME_SINCE_SEEN
            time.sleep(1.1 - elapsed)
        
        # 重新出现在不同位置（避免 IoU 匹配）
        faces2 = [
            FaceInfo(x=0.7, y=0.7, width=0.15, height=0.2, confidence=0.9),
        ]
        result2 = self.tracker.update(faces2)
        
        # 应该分配新 ID（因为已清理且位置不同）
        self.assertNotEqual(result2[0].face_id, face_id,
                           f"Should get new ID after cleanup, but got same ID {face_id}")
    
    def test_tracking_stats(self):
        """测试追踪统计"""
        faces = [
            FaceInfo(x=0.2, y=0.2, width=0.15, height=0.2, confidence=0.9),
        ]
        
        # 更新多次
        for _ in range(10):
            self.tracker.update(faces)
        
        stats = self.tracker.get_stats()
        
        self.assertGreater(stats['total_detections'], 0)
        self.assertGreater(stats['total_matches'], 0)
        self.assertGreaterEqual(stats['match_rate'], 0.0)


class TestFaceDetectorWithTracking(unittest.TestCase):
    """FaceDetector 集成 ID 追踪测试"""
    
    def setUp(self):
        if not CV_AVAILABLE:
            self.skipTest("OpenCV not available")
        
        # 测试图片路径（相对于 engine_ai 根目录）
        self.test_image_path = os.path.join(
            engine_ai_root,
            "engineai_ros2_workspace/src/simulation/mujoco/assets/resource/environment/mp4.jpg"
        )
        
        if not os.path.exists(self.test_image_path):
            self.skipTest(f"Test image not found: {self.test_image_path}")
        
        self.image = cv2.imread(self.test_image_path)
        self.assertIsNotNone(self.image, "Failed to load test image")
    
    def test_id_tracking_enabled(self):
        """测试 ID 追踪启用"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        faces = detector.detect(self.image)
        
        # 验证所有人脸都有有效的 face_id
        for face in faces:
            self.assertGreaterEqual(face.face_id, 0, 
                                  f"Face should have valid ID, got {face.face_id}")
        
        detector.release()
    
    def test_cross_frame_id_stability(self):
        """测试跨帧 ID 稳定性"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        # 多次检测同一图片（模拟连续帧）
        results = []
        for _ in range(5):
            faces = detector.detect(self.image)
            results.append(faces)
        
        # 验证 ID 稳定性
        if len(results[0]) > 0:
            first_frame_ids = {f.face_id for f in results[0]}
            
            for i, faces in enumerate(results[1:], 1):
                current_ids = {f.face_id for f in faces}
                # 至少部分 ID 应该保持一致
                overlap = first_frame_ids & current_ids
                if len(first_frame_ids) > 0:
                    stability = len(overlap) / len(first_frame_ids)
                    self.assertGreater(stability, 0.5, 
                                    f"ID stability too low: {stability:.2f}")
        
        detector.release()
    
    def test_tracking_stats(self):
        """测试追踪统计"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        # 多次检测
        for _ in range(10):
            detector.detect(self.image)
        
        stats = detector.get_tracking_stats()
        
        self.assertGreater(stats['total_detections'], 0)
        self.assertGreaterEqual(stats['match_rate'], 0.0)
        
        detector.release()


class TestIdleStateFaceSelection(unittest.TestCase):
    """IDLE 状态人脸选择测试"""
    
    def setUp(self):
        if not CV_AVAILABLE:
            self.skipTest("OpenCV not available")
        
        # 创建 FSM 和 IDLE 状态
        self.fsm_data = WeddingFSMData()
        self.fsm = type('MockFSM', (), {
            'data': self.fsm_data,
            'get_current_time': lambda self=None: time.time(),  # 修复：允许 self 参数
            '_ros2_logger': None,
        })()
        
        self.idle_state = IdleState(self.fsm)
        
        # 测试图片路径（相对于 engine_ai 根目录）
        self.test_image_path = os.path.join(
            engine_ai_root,
            "engineai_ros2_workspace/src/simulation/mujoco/assets/resource/environment/mp4.jpg"
        )
        
        if not os.path.exists(self.test_image_path):
            self.skipTest(f"Test image not found: {self.test_image_path}")
        
        self.image = cv2.imread(self.test_image_path)
        self.assertIsNotNone(self.image, "Failed to load test image")
    
    def test_select_largest_frontal_face(self):
        """测试选择最大正脸（考虑位置和面积）"""
        # 创建检测器
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        # 检测人脸
        faces = detector.detect(self.image)
        
        # 过滤正脸
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) < 2:
            self.skipTest("Need at least 2 frontal faces for this test")
        
        # 使用 IDLE 状态的方法选择最佳正脸
        # 注意：select_best_face 考虑位置和面积，不是单纯按面积
        best_face = self.idle_state.select_best_face(frontal_faces, 0.5, 0.5)
        
        self.assertIsNotNone(best_face, "Should select a face")
        
        # 验证选择的是评分最高的（不是单纯面积最大）
        # 计算所有正脸的评分
        def calculate_score(face):
            area_score = min(1.0, face.area / 0.1)
            distance_to_center = abs(face.center_x - 0.5) + abs(face.center_y - 0.5)
            center_score = max(0.0, 1.0 - distance_to_center * 2)
            center_weight = self.idle_state.CENTER_WEIGHT
            return area_score * (1 - center_weight) + center_score * center_weight
        
        # 使用 face_id 作为键，因为 FaceInfo 不可哈希
        scores = {f.face_id: calculate_score(f) for f in frontal_faces}
        max_score_face = max(frontal_faces, key=lambda f: scores[f.face_id])
        
        # 验证选择的是评分最高的
        self.assertEqual(best_face.face_id, max_score_face.face_id,
                        f"Should select face with highest score, not just largest area")
        
        detector.release()
    
    def test_candidate_tracking(self):
        """测试候选追踪"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        # 检测人脸
        faces = detector.detect(self.image)
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) == 0:
            self.skipTest("No frontal faces detected")
        
        # 选择候选
        candidate = self.idle_state.select_best_face(frontal_faces, 0.5, 0.5)
        self.idle_state._candidate_face = candidate
        self.idle_state._candidate_start_time = time.time()
        
        # 模拟多帧匹配（使用相同检测结果）
        match_count = 0
        for _ in range(10):
            # 更新感知数据
            self.fsm_data.perception.faces = faces
            
            # 更新目标检测
            self.idle_state._update_target_detection()
            
            # 检查是否匹配成功
            if (self.idle_state._candidate_face is not None and 
                self.idle_state._candidate_face.face_id == candidate.face_id):
                match_count += 1
        
        # 验证匹配成功率
        match_rate = match_count / 10
        self.assertGreater(match_rate, 0.7, 
                          f"Match rate too low: {match_rate:.2f}")
        
        detector.release()
    
    def test_candidate_confirmation(self):
        """测试候选确认"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        faces = detector.detect(self.image)
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) == 0:
            self.skipTest("No frontal faces detected")
        
        # 设置候选
        candidate = self.idle_state.select_best_face(frontal_faces, 0.5, 0.5)
        self.idle_state._candidate_face = candidate
        self.idle_state._candidate_start_time = time.time() - 3.5  # 模拟已持续 3.5 秒
        
        # 验证确认
        self.assertTrue(self.idle_state._is_candidate_confirmed(),
                       "Candidate should be confirmed after 3.5s")
        
        detector.release()


class TestSearchStateTracking(unittest.TestCase):
    """SEARCH 状态追踪测试"""
    
    def setUp(self):
        if not CV_AVAILABLE:
            self.skipTest("OpenCV not available")
        
        # 创建 FSM 和 SEARCH 状态
        self.fsm_data = WeddingFSMData()
        self.fsm = type('MockFSM', (), {
            'data': self.fsm_data,
            'get_current_time': lambda self=None: time.time(),  # 修复：允许 self 参数
            '_ros2_logger': None,
        })()
        
        self.search_state = SearchState(self.fsm)
        
        # 测试图片路径（相对于 engine_ai 根目录）
        self.test_image_path = os.path.join(
            engine_ai_root,
            "engineai_ros2_workspace/src/simulation/mujoco/assets/resource/environment/mp4.jpg"
        )
        
        if not os.path.exists(self.test_image_path):
            self.skipTest(f"Test image not found: {self.test_image_path}")
        
        self.image = cv2.imread(self.test_image_path)
        self.assertIsNotNone(self.image, "Failed to load test image")
    
    def test_receive_face_id_from_idle(self):
        """测试从 IDLE 接收 face_id"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        # 检测人脸
        faces = detector.detect(self.image)
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) == 0:
            self.skipTest("No frontal faces detected")
        
        # 创建 LockedTarget（模拟 IDLE 传递）
        target_face = max(frontal_faces, key=lambda f: f.area)
        locked_target = LockedTarget.from_face(target_face)
        
        # 设置到感知数据
        self.fsm_data.perception.locked_target = locked_target
        
        # 初始化 SEARCH 状态
        self.search_state.on_enter()
        
        # 获取跟踪器
        tracker = self.search_state._get_tracker()
        
        # 验证初始目标设置
        self.assertIsNotNone(tracker._target, "Tracker should have initial target")
        self.assertEqual(tracker._target.face_id, locked_target.face_id,
                        "Face ID should match")
        
        detector.release()
    
    def test_tracking_with_face_id(self):
        """测试基于 face_id 的追踪"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        # 检测人脸
        faces = detector.detect(self.image)
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) == 0:
            self.skipTest("No frontal faces detected")
        
        # 设置初始目标
        target_face = max(frontal_faces, key=lambda f: f.area)
        locked_target = LockedTarget.from_face(target_face)
        self.fsm_data.perception.locked_target = locked_target
        
        self.search_state.on_enter()
        tracker = self.search_state._get_tracker()
        
        # 模拟多帧追踪
        tracking_success_count = 0
        for _ in range(10):
            # 更新感知数据
            self.fsm_data.perception.faces = faces
            
            # 更新跟踪器
            state = tracker.update(self.fsm_data.perception.faces)
            
            if state.has_target and state.target_face.face_id == target_face.face_id:
                tracking_success_count += 1
        
        # 验证追踪成功率
        success_rate = tracking_success_count / 10
        self.assertGreater(success_rate, 0.8, 
                          f"Tracking success rate too low: {success_rate:.2f}")
        
        detector.release()
    
    def test_target_confirmation(self):
        """测试目标确认"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        faces = detector.detect(self.image)
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) == 0:
            self.skipTest("No frontal faces detected")
        
        # 设置初始目标
        target_face = max(frontal_faces, key=lambda f: f.area)
        locked_target = LockedTarget.from_face(target_face)
        self.fsm_data.perception.locked_target = locked_target
        
        self.search_state.on_enter()
        tracker = self.search_state._get_tracker()
        
        # 模拟持续正脸（修改确认时间以便快速测试）
        original_confirm = tracker.CONFIRM_DURATION
        tracker.CONFIRM_DURATION = 0.1  # 临时改为 0.1 秒
        
        # 持续更新（正脸）
        start_time = time.time()
        while time.time() - start_time < 0.2:
            self.fsm_data.perception.faces = faces
            tracker.update(self.fsm_data.perception.faces)
            time.sleep(0.01)
        
        # 验证确认
        self.assertTrue(tracker.is_target_confirmed,
                       "Target should be confirmed after sufficient frontal duration")
        
        # 恢复原始值
        tracker.CONFIRM_DURATION = original_confirm
        
        detector.release()


class TestIdleSearchIntegration(unittest.TestCase):
    """IDLE → SEARCH 集成测试"""
    
    def setUp(self):
        if not CV_AVAILABLE:
            self.skipTest("OpenCV not available")
        
        # 创建 FSM
        self.fsm_data = WeddingFSMData()
        self.fsm = type('MockFSM', (), {
            'data': self.fsm_data,
            'get_current_time': lambda self=None: time.time(),  # 修复：允许 self 参数
            '_ros2_logger': None,
        })()
        
        self.idle_state = IdleState(self.fsm)
        self.search_state = SearchState(self.fsm)
        
        # 测试图片路径（相对于 engine_ai 根目录）
        self.test_image_path = os.path.join(
            engine_ai_root,
            "engineai_ros2_workspace/src/simulation/mujoco/assets/resource/environment/mp4.jpg"
        )
        
        if not os.path.exists(self.test_image_path):
            self.skipTest(f"Test image not found: {self.test_image_path}")
        
        self.image = cv2.imread(self.test_image_path)
        self.assertIsNotNone(self.image, "Failed to load test image")
    
    def test_complete_flow(self):
        """测试完整流程：IDLE → SEARCH"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        # 1. IDLE 状态检测
        faces = detector.detect(self.image)
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) < 2:
            self.skipTest("Need at least 2 frontal faces for this test")
        
        # 设置感知数据
        self.fsm_data.perception.faces = faces
        
        # IDLE 选择最佳正脸（考虑位置和面积）
        best_face = self.idle_state.select_best_face(frontal_faces, 0.5, 0.5)
        
        # 验证选择的是评分最高的（不是单纯面积最大）
        def calculate_score(face):
            area_score = min(1.0, face.area / 0.1)
            distance_to_center = abs(face.center_x - 0.5) + abs(face.center_y - 0.5)
            center_score = max(0.0, 1.0 - distance_to_center * 2)
            center_weight = self.idle_state.CENTER_WEIGHT
            return area_score * (1 - center_weight) + center_score * center_weight
        
        # 使用 face_id 作为键，因为 FaceInfo 不可哈希
        scores = {f.face_id: calculate_score(f) for f in frontal_faces}
        max_score_face = max(frontal_faces, key=lambda f: scores[f.face_id])
        
        self.assertEqual(best_face.face_id, max_score_face.face_id,
                        "IDLE should select face with highest score")
        
        # 设置候选并确认
        self.idle_state._candidate_face = best_face
        self.idle_state._candidate_start_time = time.time() - 3.5
        
        # 验证确认
        self.assertTrue(self.idle_state._is_candidate_confirmed())
        
        # 2. IDLE → SEARCH 转换
        next_state = self.idle_state.check_transition()
        self.assertEqual(next_state, WeddingStateName.SEARCH,
                        "Should transition to SEARCH")
        
        # 验证 LockedTarget 创建
        locked_target = self.fsm_data.perception.locked_target
        self.assertIsNotNone(locked_target, "LockedTarget should be created")
        self.assertEqual(locked_target.face_id, best_face.face_id,
                        "Face ID should be preserved")
        
        # 3. SEARCH 状态追踪
        self.search_state.on_enter()
        tracker = self.search_state._get_tracker()
        
        # 验证初始目标
        self.assertIsNotNone(tracker._target, "Tracker should have target")
        self.assertEqual(tracker._target.face_id, best_face.face_id,
                        "Face ID should match from IDLE")
        
        # 模拟追踪
        tracking_success = 0
        for _ in range(10):
            self.fsm_data.perception.faces = faces
            state = tracker.update(self.fsm_data.perception.faces)
            
            if (state.has_target and 
                state.target_face.face_id == best_face.face_id):
                tracking_success += 1
        
        # 验证追踪成功
        success_rate = tracking_success / 10
        self.assertGreater(success_rate, 0.8,
                          f"Tracking success rate too low: {success_rate:.2f}")
        
        detector.release()
    
    def test_face_id_consistency(self):
        """测试 face_id 一致性"""
        detector = FaceDetector(enable_tracking=True, use_face_mesh=True)
        
        faces = detector.detect(self.image)
        frontal_faces = [f for f in faces if f.is_frontal]
        
        if len(frontal_faces) == 0:
            self.skipTest("No frontal faces detected")
        
        # IDLE 选择
        best_face = self.idle_state.select_best_face(frontal_faces, 0.5, 0.5)
        idle_face_id = best_face.face_id
        
        # 创建 LockedTarget
        locked_target = LockedTarget.from_face(best_face)
        self.assertEqual(locked_target.face_id, idle_face_id,
                        "LockedTarget should preserve face_id")
        
        # SEARCH 接收
        self.fsm_data.perception.locked_target = locked_target
        self.search_state.on_enter()
        
        tracker = self.search_state._get_tracker()
        search_face_id = tracker._target.face_id
        
        # 验证一致性
        self.assertEqual(search_face_id, idle_face_id,
                        "Face ID should be consistent from IDLE to SEARCH")
        
        detector.release()


def run_tests():
    """运行所有测试"""
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加测试类
    suite.addTests(loader.loadTestsFromTestCase(TestFaceIDTracker))
    suite.addTests(loader.loadTestsFromTestCase(TestFaceDetectorWithTracking))
    suite.addTests(loader.loadTestsFromTestCase(TestIdleStateFaceSelection))
    suite.addTests(loader.loadTestsFromTestCase(TestSearchStateTracking))
    suite.addTests(loader.loadTestsFromTestCase(TestIdleSearchIntegration))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 打印摘要
    print("\n" + "="*50)
    print("测试摘要")
    print("="*50)
    print(f"总测试数: {result.testsRun}")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    
    if result.failures:
        print("\n失败的测试:")
        for test, traceback in result.failures:
            print(f"  - {test}")
    
    if result.errors:
        print("\n错误的测试:")
        for test, traceback in result.errors:
            print(f"  - {test}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)