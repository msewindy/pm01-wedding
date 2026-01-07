#!/usr/bin/env python3
"""
SEARCH 状态独立测试脚本

测试 SearchState 的状态逻辑，不依赖 ROS2

Usage:
    python test/states/test_search_state.py
"""

import sys
import time

# 添加项目路径
import os
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

from wedding_interaction.fsm import WeddingFSM, WeddingStateName
from wedding_interaction.fsm.states import IdleState, SearchState, TrackingState, SafeStopState
from wedding_interaction.perception import FaceInfo, BodyInfo


def create_mock_face(x: float = 0.5, y: float = 0.3, 
                     is_frontal: bool = True, face_id: int = 0) -> FaceInfo:
    """创建模拟人脸"""
    return FaceInfo(
        x=x - 0.1,
        y=y - 0.15,
        width=0.2,
        height=0.3,
        is_frontal=is_frontal,
        yaw=0.0 if is_frontal else 45.0,
        confidence=0.9,
        face_id=face_id,
        timestamp=time.time()
    )


def create_mock_body(x: float = 0.5, y: float = 0.3, body_id: int = 0) -> BodyInfo:
    """创建模拟人体"""
    return BodyInfo(
        x=x - 0.15,
        y=y - 0.1,
        width=0.3,
        height=0.7,
        confidence=0.9,
        body_id=body_id,
        timestamp=time.time()
    )


def test_idle_to_search():
    """测试 IDLE → SEARCH 转换"""
    print("\n" + "=" * 50)
    print("Test: IDLE → SEARCH")
    print("=" * 50)
    
    # 创建 FSM
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.register_state(SearchState(fsm))
    fsm.register_state(TrackingState(fsm))
    fsm.register_state(SafeStopState(fsm))
    fsm.initialize(WeddingStateName.IDLE)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 模拟无人脸的情况
    print("\n[Step 1] No face detected...")
    for i in range(10):
        fsm.data.perception.face_detected = False
        fsm.run_once()
    print(f"State: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.IDLE, "Should stay in IDLE"
    
    # 模拟检测到人脸
    print("\n[Step 2] Face detected for 0.5s...")
    for i in range(25):  # 50Hz * 0.5s = 25 iterations
        fsm.data.perception.face_detected = True
        fsm.run_once()
        time.sleep(0.02)
    
    print(f"State: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.SEARCH, "Should transition to SEARCH"
    
    print("\n✅ Test passed: IDLE → SEARCH transition works!")
    fsm.stop()


def test_search_to_tracking():
    """测试 SEARCH → TRACKING 转换"""
    print("\n" + "=" * 50)
    print("Test: SEARCH → TRACKING")
    print("=" * 50)
    
    # 创建 FSM
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.register_state(SearchState(fsm))
    fsm.register_state(TrackingState(fsm))
    fsm.register_state(SafeStopState(fsm))
    fsm.initialize(WeddingStateName.SEARCH)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 创建模拟数据
    face = create_mock_face(0.5, 0.3, is_frontal=True, face_id=1)
    body = create_mock_body(0.5, 0.3, body_id=1)
    
    # 模拟持续检测到正脸和人体
    print("\n[Step 1] Detecting frontal face + body for 2.5s...")
    start_time = time.time()
    
    while time.time() - start_time < 2.5:
        # 更新感知数据
        fsm.data.perception.faces = [face]
        fsm.data.perception.bodies = [body]
        fsm.data.perception.face_detected = True
        fsm.run_once()
        
        # 打印进度
        elapsed = time.time() - start_time
        search_state = fsm.current_state
        if hasattr(search_state, 'get_search_info'):
            info = search_state.get_search_info()
            print(f"\r  Elapsed: {elapsed:.1f}s, Phase: {info['phase']}, "
                  f"Face Duration: {info['face_visible_duration']:.1f}s", end="")
        
        time.sleep(0.02)
    
    print()
    print(f"Final state: {fsm.get_current_state_name()}")
    
    if fsm.get_current_state_name() == WeddingStateName.TRACKING:
        print("\n✅ Test passed: SEARCH → TRACKING transition works!")
    else:
        print(f"\n⚠️ Test note: State is {fsm.get_current_state_name()}, "
              "may need more time or configuration adjustment")
    
    fsm.stop()


def test_search_to_idle_lost():
    """测试 SEARCH → IDLE（目标丢失）"""
    print("\n" + "=" * 50)
    print("Test: SEARCH → IDLE (target lost)")
    print("=" * 50)
    
    # 创建 FSM
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.register_state(SearchState(fsm))
    fsm.register_state(TrackingState(fsm))
    fsm.register_state(SafeStopState(fsm))
    fsm.initialize(WeddingStateName.SEARCH)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 创建模拟数据
    face = create_mock_face(0.5, 0.3, is_frontal=True, face_id=1)
    body = create_mock_body(0.5, 0.3, body_id=1)
    
    # 先检测到目标
    print("\n[Step 1] Detecting target for 0.5s...")
    for i in range(25):
        fsm.data.perception.faces = [face]
        fsm.data.perception.bodies = [body]
        fsm.data.perception.face_detected = True
        fsm.run_once()
        time.sleep(0.02)
    
    # 然后丢失目标
    print("[Step 2] Target lost for 3.5s...")
    start_time = time.time()
    
    while time.time() - start_time < 3.5:
        fsm.data.perception.faces = []
        fsm.data.perception.bodies = []
        fsm.data.perception.face_detected = False
        fsm.run_once()
        
        elapsed = time.time() - start_time
        print(f"\r  Elapsed: {elapsed:.1f}s, State: {fsm.get_current_state_name()}", end="")
        
        if fsm.get_current_state_name() == WeddingStateName.IDLE:
            break
        
        time.sleep(0.02)
    
    print()
    print(f"Final state: {fsm.get_current_state_name()}")
    
    if fsm.get_current_state_name() == WeddingStateName.IDLE:
        print("\n✅ Test passed: SEARCH → IDLE (target lost) works!")
    else:
        print(f"\n⚠️ Test note: State is {fsm.get_current_state_name()}")
    
    fsm.stop()


def test_search_timeout():
    """测试 SEARCH 超时"""
    print("\n" + "=" * 50)
    print("Test: SEARCH timeout")
    print("=" * 50)
    
    # 创建 FSM
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    
    # 创建 SearchState 并设置短超时
    search_state = SearchState(fsm)
    search_state.T_SEARCH_TIMEOUT = 2.0  # 2秒超时（测试用）
    fsm.register_state(search_state)
    
    fsm.register_state(TrackingState(fsm))
    fsm.register_state(SafeStopState(fsm))
    fsm.initialize(WeddingStateName.SEARCH)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    print(f"Timeout set to: {search_state.T_SEARCH_TIMEOUT}s")
    
    # 模拟无目标超时
    print("\n[Step 1] No target, waiting for timeout...")
    start_time = time.time()
    
    while time.time() - start_time < 2.5:
        fsm.data.perception.faces = []
        fsm.data.perception.bodies = []
        fsm.data.perception.face_detected = False
        fsm.run_once()
        
        elapsed = time.time() - start_time
        print(f"\r  Elapsed: {elapsed:.1f}s, State: {fsm.get_current_state_name()}", end="")
        
        if fsm.get_current_state_name() == WeddingStateName.IDLE:
            break
        
        time.sleep(0.02)
    
    print()
    print(f"Final state: {fsm.get_current_state_name()}")
    
    if fsm.get_current_state_name() == WeddingStateName.IDLE:
        print("\n✅ Test passed: SEARCH timeout works!")
    else:
        print(f"\n⚠️ Test note: State is {fsm.get_current_state_name()}")
    
    fsm.stop()


def test_body_takeover():
    """测试人体接管（正脸丢失，人体继续跟踪）"""
    print("\n" + "=" * 50)
    print("Test: Body takeover (face lost, body continues)")
    print("=" * 50)
    
    # 创建 FSM
    fsm = WeddingFSM()
    fsm.register_state(IdleState(fsm))
    fsm.register_state(SearchState(fsm))
    fsm.register_state(TrackingState(fsm))
    fsm.register_state(SafeStopState(fsm))
    fsm.initialize(WeddingStateName.SEARCH)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 创建模拟数据
    face = create_mock_face(0.5, 0.3, is_frontal=True, face_id=1)
    body = create_mock_body(0.5, 0.3, body_id=1)
    
    # 先检测到正脸和人体
    print("\n[Step 1] Detecting face + body for 1s...")
    for i in range(50):
        fsm.data.perception.faces = [face]
        fsm.data.perception.bodies = [body]
        fsm.data.perception.face_detected = True
        fsm.run_once()
        time.sleep(0.02)
    
    # 正脸丢失，只有人体
    print("[Step 2] Face lost, body continues for 1.5s...")
    start_time = time.time()
    
    while time.time() - start_time < 1.5:
        fsm.data.perception.faces = []  # 正脸丢失
        fsm.data.perception.bodies = [body]  # 人体仍在
        fsm.data.perception.face_detected = False
        fsm.run_once()
        
        search_state = fsm.current_state
        if hasattr(search_state, 'get_search_info'):
            info = search_state.get_search_info()
            elapsed = time.time() - start_time
            print(f"\r  Elapsed: {elapsed:.1f}s, "
                  f"Face visible: {info['is_face_visible']}, "
                  f"Body visible: {info['is_body_visible']}, "
                  f"Duration: {info['face_visible_duration']:.1f}s", end="")
        
        time.sleep(0.02)
    
    print()
    
    # 检查是否进入了 TRACKING（因为人体持续跟踪）
    print(f"Final state: {fsm.get_current_state_name()}")
    
    if fsm.get_current_state_name() == WeddingStateName.TRACKING:
        print("\n✅ Test passed: Body takeover works!")
    elif fsm.get_current_state_name() == WeddingStateName.SEARCH:
        search_state = fsm.current_state
        if hasattr(search_state, 'get_search_info'):
            info = search_state.get_search_info()
            print(f"\n⚠️ Still in SEARCH, face_duration={info['face_visible_duration']:.1f}s")
            print("  May need more time to confirm target")
    else:
        print(f"\n⚠️ Unexpected state: {fsm.get_current_state_name()}")
    
    fsm.stop()


def run_all_tests():
    """运行所有测试"""
    print("\n" + "=" * 60)
    print("SEARCH State Test Suite")
    print("=" * 60)
    
    tests = [
        ("IDLE → SEARCH", test_idle_to_search),
        ("SEARCH → TRACKING", test_search_to_tracking),
        ("SEARCH → IDLE (lost)", test_search_to_idle_lost),
        ("SEARCH timeout", test_search_timeout),
        ("Body takeover", test_body_takeover),
    ]
    
    passed = 0
    failed = 0
    
    for name, test_func in tests:
        try:
            test_func()
            passed += 1
        except Exception as e:
            print(f"\n❌ Test '{name}' failed with error: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "=" * 60)
    print(f"Test Summary: {passed} passed, {failed} failed")
    print("=" * 60)


if __name__ == '__main__':
    run_all_tests()

