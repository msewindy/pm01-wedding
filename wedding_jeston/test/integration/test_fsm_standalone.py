#!/usr/bin/env python3
"""
独立 FSM 测试脚本（无需 ROS2）

用于验证 FSM 逻辑
"""

import sys
import os
import time
import logging

# 添加包路径
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(script_dir)
sys.path.insert(0, pkg_dir)

from wedding_interaction.fsm import (
    WeddingFSM,
    WeddingStateName,
    WeddingEvent,
)
from wedding_interaction.fsm.states import (
    IdleState,
    TrackingState,
    PhotoPosingState,
    FarewellState,
    SafeStopState,
)


def setup_logging():
    """配置日志"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        datefmt='%H:%M:%S'
    )


def create_fsm() -> WeddingFSM:
    """创建并初始化 FSM"""
    fsm = WeddingFSM()
    
    # 注册状态
    fsm.register_state(IdleState(fsm))
    fsm.register_state(TrackingState(fsm))
    fsm.register_state(PhotoPosingState(fsm))
    fsm.register_state(FarewellState(fsm))
    fsm.register_state(SafeStopState(fsm))
    
    return fsm


def test_idle_to_tracking():
    """测试：IDLE -> TRACKING"""
    print("\n" + "="*50)
    print("Test: IDLE -> TRACKING (face detection)")
    print("="*50)
    
    fsm = create_fsm()
    fsm.initialize(WeddingStateName.IDLE)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 运行 IDLE 几次
    for i in range(5):
        fsm.run_once()
        time.sleep(0.02)
    
    # 模拟人脸检测
    print("Simulating face detection...")
    fsm.data.perception.face_detected = True
    fsm.data.perception.face_position[0] = 0.6
    fsm.data.perception.face_position[1] = 0.4
    
    # 运行直到状态转换
    for i in range(30):
        fsm.run_once()
        fsm.data.perception.face_detected = True  # 保持检测
        time.sleep(0.02)
        if fsm.get_current_state_name() == WeddingStateName.TRACKING:
            break
    
    print(f"Final state: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.TRACKING
    print("✓ Test passed!")


def test_tracking_to_photo():
    """测试：TRACKING -> PHOTO_POSING (手势)"""
    print("\n" + "="*50)
    print("Test: TRACKING -> PHOTO_POSING (gesture)")
    print("="*50)
    
    fsm = create_fsm()
    fsm.initialize(WeddingStateName.TRACKING)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 模拟人脸 + 比心手势
    print("Simulating face + heart gesture...")
    for i in range(30):
        fsm.data.perception.face_detected = True
        fsm.data.perception.gesture = "heart"
        fsm.data.perception.gesture_confidence = 0.9
        fsm.run_once()
        time.sleep(0.02)
        if fsm.get_current_state_name() == WeddingStateName.PHOTO_POSING:
            break
    
    print(f"Final state: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.PHOTO_POSING
    print("✓ Test passed!")


def test_photo_flow():
    """测试：合影完整流程"""
    print("\n" + "="*50)
    print("Test: Photo posing complete flow")
    print("="*50)
    
    fsm = create_fsm()
    fsm.initialize(WeddingStateName.PHOTO_POSING)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    state = fsm.states[WeddingStateName.PHOTO_POSING]
    
    # 运行合影流程
    last_phase = ""
    for i in range(450):  # 增加迭代次数，确保转换完成
        fsm.run_once()
        time.sleep(0.02)
        
        current_state = fsm.get_current_state_name()
        if current_state == WeddingStateName.PHOTO_POSING:
            if state.phase != last_phase:
                print(f"  Phase: {state.phase}")
                last_phase = state.phase
        
        if current_state == WeddingStateName.TRACKING:
            print(f"  Transitioned to TRACKING!")
            break
    
    print(f"Final state: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.TRACKING
    print("✓ Test passed!")


def test_safety_trigger():
    """测试：安全触发"""
    print("\n" + "="*50)
    print("Test: Safety trigger (too close)")
    print("="*50)
    
    fsm = create_fsm()
    fsm.initialize(WeddingStateName.TRACKING)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 模拟太近
    print("Simulating too close...")
    fsm.data.perception.too_close = True
    fsm.run_once()
    
    print(f"After safety trigger: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.SAFE_STOP
    
    # 解除安全条件
    print("Clearing safety condition...")
    fsm.data.perception.too_close = False
    
    # 需要等待恢复时间（默认 2 秒）+ 额外缓冲
    for i in range(200):  # 增加到 4 秒
        fsm.run_once()
        time.sleep(0.02)
        current_state = fsm.get_current_state_name()
        if current_state == WeddingStateName.IDLE:
            print(f"  Recovered after {i * 0.02:.1f}s")
            break
    
    print(f"After recovery: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.IDLE
    print("✓ Test passed!")


def test_command_control():
    """测试：命令控制"""
    print("\n" + "="*50)
    print("Test: Command control")
    print("="*50)
    
    fsm = create_fsm()
    fsm.initialize(WeddingStateName.IDLE)
    
    print(f"Initial state: {fsm.get_current_state_name()}")
    
    # 命令进入合影
    print("Sending CMD_GOTO_PHOTO...")
    fsm.send_command(WeddingEvent.CMD_GOTO_PHOTO)
    fsm.run_once()
    fsm.run_once()  # 完成转换
    
    print(f"After CMD_GOTO_PHOTO: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.PHOTO_POSING
    
    # 命令停止
    print("Sending CMD_STOP...")
    fsm.send_command(WeddingEvent.CMD_STOP)
    fsm.run_once()
    fsm.run_once()
    
    print(f"After CMD_STOP: {fsm.get_current_state_name()}")
    assert fsm.get_current_state_name() == WeddingStateName.IDLE
    print("✓ Test passed!")


def main():
    """主函数"""
    setup_logging()
    
    print("\n" + "#"*60)
    print("# Wedding FSM Standalone Test")
    print("#"*60)
    
    try:
        test_idle_to_tracking()
        test_tracking_to_photo()
        test_photo_flow()
        test_safety_trigger()
        test_command_control()
        
        print("\n" + "="*50)
        print("All tests passed! ✓")
        print("="*50 + "\n")
        
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

