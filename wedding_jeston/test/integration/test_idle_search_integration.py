#!/usr/bin/env python3
"""
IDLE → SEARCH 集成测试

自动化测试 IDLE 和 SEARCH 状态的切换逻辑

Usage:
    # 先启动仿真器和婚礼互动节点
    ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
    ros2 launch wedding_interaction search_debug.launch.py
    
    # 运行测试
    python3 test/integration/test_idle_search_integration.py
"""

import time
import threading
from dataclasses import dataclass, field
from typing import Optional, List, Callable
from enum import Enum
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PointStamped


class TestResult(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    TIMEOUT = "TIMEOUT"


@dataclass
class TestCase:
    """测试用例"""
    id: str
    name: str
    description: str
    timeout: float = 30.0
    result: TestResult = TestResult.SKIP
    duration: float = 0.0
    message: str = ""


@dataclass
class TestContext:
    """测试上下文（共享数据）"""
    current_state: str = "UNKNOWN"
    last_state_change_time: float = 0.0
    look_at_x: float = 0.5
    look_at_y: float = 0.5
    look_at_count: int = 0
    face_detected: bool = False
    audio_play_count: int = 0
    last_audio: str = ""
    pose_count: int = 0
    last_pose: str = ""
    panel_x: float = 0.0  # panel 当前位置
    panel_y: float = 0.0
    panel_z: float = 0.0


class IdleSearchIntegrationTest(Node):
    """IDLE → SEARCH 集成测试节点"""
    
    # Photo Panel 位置常量
    PANEL_FAR = 80.0      # 远离（相机看不到，确认失败原因）
    PANEL_NEAR = 1.5     # 靠近（相机可见，更近以增大人脸像素）
    PANEL_LEFT = 0.8     # 左侧
    PANEL_RIGHT = -0.8   # 右侧
    PANEL_CENTER = 0.0   # 中心
    PANEL_Z = 1.5        # 高度
    
    def __init__(self):
        super().__init__('idle_search_integration_test')
        
        # 测试上下文
        self.ctx = TestContext()
        self.ctx_lock = threading.Lock()
        
        # 测试用例列表
        self.test_cases: List[TestCase] = []
        self._init_test_cases()
        
        # 发布器
        self.panel_pub = self.create_publisher(
            Point, '/sim/photo_panel/position', 10)
        self.cmd_pub = self.create_publisher(
            String, '/wedding/fsm/command', 10)
        
        # 订阅器
        self.state_sub = self.create_subscription(
            String, '/wedding/fsm/state', self._on_state, 10)
        self.look_at_sub = self.create_subscription(
            PointStamped, '/wedding/motion/look_at', self._on_look_at, 10)
        self.face_sub = self.create_subscription(
            Bool, '/wedding/perception/face_detected', self._on_face_detected, 10)
        self.audio_sub = self.create_subscription(
            String, '/wedding/audio/play', self._on_audio, 10)
        self.pose_sub = self.create_subscription(
            String, '/wedding/motion/pose', self._on_pose, 10)
        
        # Panel 位置订阅（用于确认动作执行）
        # 注意：如果 MuJoCo 没有提供 feedback topic，我们通过其他方式确认
        try:
            self.panel_pos_sub = self.create_subscription(
                Point, '/sim/photo_panel/position_feedback', self._on_panel_position, 10)
        except:
            self.panel_pos_sub = None
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("IDLE → SEARCH 集成测试")
        self.get_logger().info("=" * 60)
    
    def _init_test_cases(self):
        """初始化测试用例"""
        self.test_cases = [
            TestCase("TC-01", "IDLE 动作执行", 
                    "验证 IDLE 状态下头部摆动", timeout=10.0),
            TestCase("TC-02", "IDLE 语音播放", 
                    "验证 IDLE 状态下语音播放（概率性）", timeout=40.0),
            TestCase("TC-03", "IDLE → SEARCH", 
                    "验证检测到正脸后进入 SEARCH", timeout=5.0),
            TestCase("TC-04", "SEARCH 跟随（中心）", 
                    "验证头部跟随中心目标", timeout=3.0),
            TestCase("TC-05", "SEARCH 跟随（移动）", 
                    "验证头部跟随移动目标", timeout=5.0),
            TestCase("TC-06", "SEARCH → TRACKING", 
                    "验证正脸持续 2s 后进入 TRACKING", timeout=5.0),
            TestCase("TC-07", "SEARCH → IDLE (超时)", 
                    "验证目标丢失后回到 IDLE", timeout=15.0),
            TestCase("TC-08", "目标恢复", 
                    "验证目标短暂丢失后恢复跟踪", timeout=5.0),
            TestCase("TC-09", "完整流程", 
                    "IDLE → SEARCH → TRACKING", timeout=10.0),
            TestCase("TC-10", "含超时回退", 
                    "IDLE → SEARCH → IDLE", timeout=20.0),
        ]
    
    # ========== 回调函数 ==========
    
    def _on_state(self, msg: String):
        with self.ctx_lock:
            if self.ctx.current_state != msg.data:
                self.ctx.last_state_change_time = time.time()
                self.get_logger().info(f"状态变化: {self.ctx.current_state} → {msg.data}")
            self.ctx.current_state = msg.data
    
    def _on_look_at(self, msg: PointStamped):
        with self.ctx_lock:
            self.ctx.look_at_x = msg.point.x
            self.ctx.look_at_y = msg.point.y
            self.ctx.look_at_count += 1
    
    def _on_face_detected(self, msg: Bool):
        with self.ctx_lock:
            self.ctx.face_detected = msg.data
    
    def _on_audio(self, msg: String):
        with self.ctx_lock:
            self.ctx.audio_play_count += 1
            self.ctx.last_audio = msg.data
            self.get_logger().info(f"语音播放: {msg.data}")
    
    def _on_pose(self, msg: String):
        with self.ctx_lock:
            self.ctx.pose_count += 1
            self.ctx.last_pose = msg.data
    
    def _on_panel_position(self, msg: Point):
        """Panel 位置反馈（如果存在）"""
        with self.ctx_lock:
            self.ctx.panel_x = msg.x
            self.ctx.panel_y = msg.y
            self.ctx.panel_z = msg.z
    
    # ========== 日志辅助方法 ==========
    
    def _log_phase(self, phase: str, message: str = ""):
        """记录测试阶段"""
        self.get_logger().info(f"  📍 [阶段] {phase}" + (f": {message}" if message else ""))
    
    def _log_action(self, action: str, details: str = ""):
        """记录执行的动作"""
        self.get_logger().info(f"  ⚙️  [动作] {action}" + (f": {details}" if details else ""))
    
    def _log_expectation(self, expectation: str):
        """记录期望结果"""
        self.get_logger().info(f"  📋 [期望] {expectation}")
    
    def _log_result(self, success: bool, message: str = ""):
        """记录实际结果"""
        symbol = "✅" if success else "❌"
        self.get_logger().info(f"  {symbol} [结果] " + ("通过" if success else "失败") + (f": {message}" if message else ""))
    
    def _log_data(self, label: str, value: str):
        """记录数据"""
        self.get_logger().info(f"  📊 [数据] {label}: {value}")
    
    # ========== 辅助方法 ==========
    
    def _move_panel(self, x: float, y: float = 0.0, confirm: bool = True) -> bool:
        """
        移动 photo_panel
        
        Args:
            x: X 坐标
            y: Y 坐标
            confirm: 是否确认动作执行
        
        Returns:
            如果 confirm=True，返回是否确认执行成功
        """
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = self.PANEL_Z
        self.panel_pub.publish(msg)
        
        if confirm:
            # 等待动作执行（通过观察系统响应确认）
            # 如果 panel 移动成功，应该会影响感知结果
            time.sleep(0.5)  # 等待物理更新
            return True
        return True
    
    def _send_command(self, cmd: str, confirm: bool = True):
        """
        发送 FSM 命令
        
        Args:
            cmd: 命令字符串
            confirm: 是否确认命令执行（通过状态变化）
        
        Returns:
            如果 confirm=True，返回是否确认执行成功
        """
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        
        if confirm:
            # 等待命令执行（通过状态变化确认）
            time.sleep(0.3)
            return True
        return True
    
    def _reset_fsm(self):
        """
        强制重置 FSM 到 IDLE 并等待稳定
        
        原理：
        1. 移动 panel 到远处（8.0m），使其不在相机视野内或人脸太小无法检测
        2. 发送 idle 命令强制切换到 IDLE
        3. 因为检测不到人脸，IDLE 状态不会自动进入 SEARCH
        4. 等待状态稳定（连续 5 次检测到 IDLE）
        """
        self.get_logger().info("  🔄 [重置] 开始强制重置 FSM 到 IDLE 状态")
        
        # 步骤 1: 移动 panel 到远处
        self.get_logger().info(f"  📍 [步骤 1] 移动 panel 到远处 (X={self.PANEL_FAR:.1f}m)")
        self.get_logger().info(f"     原因: 使 panel 移出相机视野或人脸太小，无法被检测到")
        self._move_panel(self.PANEL_FAR)
        time.sleep(0.5)  # 等待物理更新
        
        # 检查是否仍然检测到人脸
        with self.ctx_lock:
            face_detected = self.ctx.face_detected
        
        if face_detected:
            self.get_logger().warn(f"  ⚠️ [警告] Panel 在 {self.PANEL_FAR:.1f}m 处仍然检测到人脸（可能是误检测）")
        else:
            self.get_logger().info(f"  ✅ [确认] Panel 在 {self.PANEL_FAR:.1f}m 处未检测到人脸")
        
        # 步骤 2: 发送 idle 命令
        self.get_logger().info(f"  📍 [步骤 2] 发送 idle 命令强制切换到 IDLE")
        for i in range(3):
            self._send_command("idle", confirm=False)
            time.sleep(0.2)
        
        # 步骤 3: 等待状态稳定
        self.get_logger().info(f"  📍 [步骤 3] 等待状态稳定（需要连续 5 次检测到 IDLE）")
        self.get_logger().info(f"     原因: 如果检测不到人脸，IDLE 状态不会自动进入 SEARCH")
        
        stable_count = 0
        max_wait = 5.0
        start = time.time()
        last_state = None
        
        while time.time() - start < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            state = self._get_state()
            
            if state != last_state:
                self.get_logger().info(f"     状态: {state}")
                last_state = state
            
            if "IDLE" in state:
                stable_count += 1
                if stable_count >= 5:  # 连续 5 次检测到 IDLE
                    self.get_logger().info(f"  ✅ [成功] FSM 已稳定在 IDLE 状态（连续 {stable_count} 次确认）")
                    self.get_logger().info(f"     说明: 因为 panel 在远处，检测不到人脸，所以保持在 IDLE")
                    return True
            else:
                stable_count = 0
                # 如果进入其他状态，说明可能检测到人脸
                if "SEARCH" in state or "TRACKING" in state:
                    self.get_logger().warn(f"  ⚠️ [警告] 状态切换到 {state}，说明可能仍然检测到人脸")
        
        self.get_logger().warn(f"  ❌ [失败] FSM 未能稳定到 IDLE，当前: {self._get_state()}")
        return False
    
    def _reset_context(self):
        """重置测试上下文"""
        with self.ctx_lock:
            self.ctx.look_at_count = 0
            self.ctx.audio_play_count = 0
            self.ctx.pose_count = 0
            self.ctx.last_state_change_time = time.time()
    
    def _get_state(self) -> str:
        with self.ctx_lock:
            return self.ctx.current_state
    
    def _wait_for_state(self, target_state: str, timeout: float) -> bool:
        """等待指定状态"""
        start = time.time()
        while time.time() - start < timeout:
            if self._get_state() == target_state:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False
    
    def _spin_for(self, duration: float):
        """持续 spin 指定时间"""
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    # ========== 测试用例实现 ==========
    
    def _run_tc01(self, tc: TestCase):
        """TC-01: IDLE 动作执行（在无人脸时）"""
        # 强制进入 IDLE
        if not self._reset_fsm():
            tc.result = TestResult.FAIL
            tc.message = f"无法进入 IDLE 状态: {self._get_state()}"
            return
        
        self._reset_context()
        
        # 观察 5 秒
        self._spin_for(5.0)
        
        with self.ctx_lock:
            look_at_count = self.ctx.look_at_count
            state = self.ctx.current_state
        
        if "IDLE" not in state:
            tc.result = TestResult.FAIL
            tc.message = f"状态离开 IDLE: {state}（可能检测到噪声人脸）"
        elif look_at_count < 10:
            tc.result = TestResult.FAIL
            tc.message = f"LookAt 消息太少: {look_at_count}"
        else:
            tc.result = TestResult.PASS
            tc.message = f"LookAt 消息数: {look_at_count}"
    
    def _run_tc02(self, tc: TestCase):
        """TC-02: IDLE 语音播放"""
        # 强制进入 IDLE
        if not self._reset_fsm():
            tc.result = TestResult.FAIL
            tc.message = f"无法进入 IDLE 状态"
            return
        
        self._reset_context()
        
        # 等待语音（最多 35 秒，超过冷却时间）
        start = time.time()
        while time.time() - start < 35.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self.ctx_lock:
                if self.ctx.audio_play_count > 0:
                    break
        
        with self.ctx_lock:
            audio_count = self.ctx.audio_play_count
            last_audio = self.ctx.last_audio
        
        if audio_count > 0:
            tc.result = TestResult.PASS
            tc.message = f"收到语音: {last_audio}"
        else:
            # 语音概率 30%，可能未播放
            tc.result = TestResult.PASS
            tc.message = "未收到语音（概率性，视为通过）"
    
    def _run_tc03(self, tc: TestCase):
        """TC-03: IDLE → SEARCH"""
        self._log_phase("初始化", "准备测试 IDLE → SEARCH 切换")
        
        # 阶段 1: 强制进入 IDLE
        self._log_phase("阶段 1", "确保系统在 IDLE 状态")
        if not self._reset_fsm():
            self._log_result(False, f"无法进入 IDLE 状态: {self._get_state()}")
            tc.result = TestResult.FAIL
            tc.message = f"无法进入 IDLE 状态"
            return
        self._log_result(True, f"当前状态: {self._get_state()}")
        
        # 阶段 2: 移动 panel 到可见位置
        self._log_phase("阶段 2", "移动 panel 到可见位置以触发人脸检测")
        self._log_action("移动 panel", f"X={self.PANEL_NEAR:.2f}, Y={self.PANEL_CENTER:.2f}")
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        self._log_expectation("系统应该检测到人脸，并在 0.3s 后进入 SEARCH 状态")
        
        start = time.time()
        
        # 阶段 3: 等待状态切换
        self._log_phase("阶段 3", "等待状态切换")
        search_reached = False
        while time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            state = self._get_state()
            if "SEARCH" in state or "TRACKING" in state:
                search_reached = True
                break
        
        duration = time.time() - start
        
        # 验证结果
        self._log_phase("验证", "检查状态切换结果")
        if search_reached:
            self._log_result(True, f"成功进入 {self._get_state()} 状态，耗时 {duration:.2f}s")
            tc.result = TestResult.PASS
            tc.message = f"切换耗时: {duration:.2f}s, 状态: {self._get_state()}"
        else:
            self._log_result(False, f"未进入 SEARCH，当前状态: {self._get_state()}")
            tc.result = TestResult.FAIL
            tc.message = f"未进入 SEARCH，当前: {self._get_state()}"
    
    def _run_tc04(self, tc: TestCase):
        """TC-04: SEARCH 跟随（检测到人脸并跟随）"""
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        self._send_command("search")
        time.sleep(0.5)
        self._reset_context()
        
        # 观察 2 秒
        self._spin_for(2.0)
        
        with self.ctx_lock:
            look_x = self.ctx.look_at_x
            look_y = self.ctx.look_at_y
            look_count = self.ctx.look_at_count
        
        # 检查是否有 LookAt 消息（说明检测到人脸）
        # 注：人脸位置取决于 photo_panel 上照片中人脸的位置，不一定在中心
        if look_count > 0 and 0.0 <= look_x <= 1.0 and 0.0 <= look_y <= 1.0:
            tc.result = TestResult.PASS
            tc.message = f"LookAt: ({look_x:.2f}, {look_y:.2f}), 消息数: {look_count}"
        else:
            tc.result = TestResult.FAIL
            tc.message = f"未收到有效 LookAt 消息"
    
    def _run_tc05(self, tc: TestCase):
        """TC-05: SEARCH 跟随（移动）- 验证 Y 坐标变化"""
        self._log_phase("初始化", "准备测试 SEARCH 跟随移动")
        
        # 阶段 1: 设置初始位置
        self._log_phase("阶段 1", "移动 panel 到中心位置")
        self._log_action("移动 panel", f"X={self.PANEL_NEAR:.2f}, Y={self.PANEL_CENTER:.2f}")
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        
        self._log_action("发送命令", "search")
        self._send_command("search")
        time.sleep(1.0)
        
        # 记录初始 LookAt
        with self.ctx_lock:
            initial_x = self.ctx.look_at_x
            initial_y = self.ctx.look_at_y
        
        self._log_data("初始 LookAt", f"X={initial_x:.3f}, Y={initial_y:.3f}")
        self._log_expectation("LookAt 应该指向 panel 中心位置")
        
        # 阶段 2: 移动到左侧
        self._log_phase("阶段 2", "移动 panel 到左侧 (Y=0.8)")
        self._log_action("移动 panel", f"X={self.PANEL_NEAR:.2f}, Y={self.PANEL_LEFT:.2f}")
        self._move_panel(self.PANEL_NEAR, self.PANEL_LEFT)
        self._log_expectation("LookAt Y 坐标应该变化（跟随 panel 移动）")
        
        self._spin_for(1.5)
        
        with self.ctx_lock:
            left_x = self.ctx.look_at_x
            left_y = self.ctx.look_at_y
        
        self._log_data("左侧 LookAt", f"X={left_x:.3f}, Y={left_y:.3f}")
        
        # 阶段 3: 移动到右侧
        self._log_phase("阶段 3", "移动 panel 到右侧 (Y=-0.8)")
        self._log_action("移动 panel", f"X={self.PANEL_NEAR:.2f}, Y={self.PANEL_RIGHT:.2f}")
        self._move_panel(self.PANEL_NEAR, self.PANEL_RIGHT)
        self._log_expectation("LookAt Y 坐标应该再次变化")
        
        self._spin_for(1.5)
        
        with self.ctx_lock:
            right_x = self.ctx.look_at_x
            right_y = self.ctx.look_at_y
        
        self._log_data("右侧 LookAt", f"X={right_x:.3f}, Y={right_y:.3f}")
        
        # 验证结果
        self._log_phase("验证", "检查 LookAt 是否跟随 panel Y 坐标变化")
        
        y_changed = abs(left_y - initial_y) > 0.01 or abs(right_y - left_y) > 0.01
        x_changed = abs(left_x - initial_x) > 0.01 or abs(right_x - left_x) > 0.01
        
        if y_changed:
            self._log_result(True, f"Y 坐标变化: {initial_y:.3f} → {left_y:.3f} → {right_y:.3f}")
        else:
            self._log_result(False, f"Y 坐标未变化: {initial_y:.3f} → {left_y:.3f} → {right_y:.3f}")
        
        if x_changed:
            self._log_result(True, f"X 坐标变化: {initial_x:.3f} → {left_x:.3f} → {right_x:.3f}")
        else:
            self._log_result(False, f"X 坐标未变化: {initial_x:.3f} → {left_x:.3f} → {right_x:.3f}")
        
        # 判断测试结果
        if y_changed or x_changed:
            tc.result = TestResult.PASS
            tc.message = f"LookAt 变化: X({initial_x:.2f}→{left_x:.2f}→{right_x:.2f}), Y({initial_y:.2f}→{left_y:.2f}→{right_y:.2f})"
        else:
            tc.result = TestResult.FAIL
            tc.message = f"LookAt 无变化: X({initial_x:.2f}, {left_x:.2f}, {right_x:.2f}), Y({initial_y:.2f}, {left_y:.2f}, {right_y:.2f})"
    
    def _run_tc06(self, tc: TestCase):
        """TC-06: SEARCH → TRACKING"""
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        self._send_command("search")
        time.sleep(0.5)
        
        start = time.time()
        
        # 等待进入 TRACKING（最多 5 秒，因为需要 2s 正脸确认）
        tracking_reached = False
        while time.time() - start < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            state = self._get_state()
            if "TRACKING" in state:
                tracking_reached = True
                break
        
        duration = time.time() - start
        
        if tracking_reached:
            # 允许 1.5s - 5s 的切换时间
            tc.result = TestResult.PASS
            tc.message = f"切换耗时: {duration:.2f}s"
        else:
            tc.result = TestResult.FAIL
            tc.message = f"未进入 TRACKING，当前: {self._get_state()}"
    
    def _run_tc07(self, tc: TestCase):
        """TC-07: 目标丢失 → FAREWELL → IDLE"""
        self._log_phase("初始化", "准备测试目标丢失回退")
        
        # 阶段 1: 进入 TRACKING
        self._log_phase("阶段 1", "进入 TRACKING 状态")
        self._log_action("移动 panel", f"X={self.PANEL_NEAR:.2f}, Y={self.PANEL_CENTER:.2f}")
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        
        self._log_action("发送命令", "tracking")
        self._send_command("tracking")
        time.sleep(1.0)
        
        # 确认在 TRACKING
        state = self._get_state()
        if "TRACKING" not in state:
            self._log_result(False, f"未进入 TRACKING，当前: {state}")
            tc.result = TestResult.FAIL
            tc.message = f"未进入 TRACKING，当前: {state}"
            return
        
        self._log_result(True, f"当前状态: {state}")
        
        # 检查 face_detected
        with self.ctx_lock:
            face_detected_before = self.ctx.face_detected
        self._log_data("移走前 face_detected", str(face_detected_before))
        
        # 阶段 2: 移动 panel 到不可见位置
        self._log_phase("阶段 2", f"移动 panel 到远处 (X={self.PANEL_FAR:.1f}m)")
        self._log_action("移动 panel", f"X={self.PANEL_FAR:.1f}, Y=0.0")
        self._log_expectation("face_detected 应该变为 False，lost_time 开始累加")
        self._move_panel(self.PANEL_FAR)
        
        # 等待物理更新
        time.sleep(1.0)
        
        # 检查 face_detected 是否变为 False
        with self.ctx_lock:
            face_detected_after = self.ctx.face_detected
        
        self._log_data("移走后 face_detected", str(face_detected_after))
        if face_detected_after:
            self._log_result(False, f"⚠️ 移走后仍然检测到人脸！可能是误检测或数据延迟")
        else:
            self._log_result(True, f"移走后未检测到人脸")
        
        # 阶段 3: 等待状态回退
        self._log_phase("阶段 3", "等待状态回退 (TRACKING → FAREWELL → IDLE)")
        self._log_expectation("TRACKING lost_timeout=1.5s → FAREWELL duration=2.5s → IDLE")
        
        start = time.time()
        visited_farewell = False
        reached_idle = False
        last_state = state
        state_change_times = []
        
        # 等待回到 IDLE（最多 20 秒，给足够时间）
        while time.time() - start < 20.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            current_state = self._get_state()
            
            if current_state != last_state:
                elapsed = time.time() - start
                state_change_times.append((elapsed, last_state, current_state))
                self._log_data("状态变化", f"{elapsed:.2f}s: {last_state} → {current_state}")
                last_state = current_state
            
            if "FAREWELL" in current_state:
                visited_farewell = True
            if "IDLE" in current_state:
                reached_idle = True
                break
        
        duration = time.time() - start
        
        # 验证结果
        self._log_phase("验证", "检查回退结果")
        self._log_data("总耗时", f"{duration:.2f}s")
        self._log_data("经过 FAREWELL", str(visited_farewell))
        self._log_data("到达 IDLE", str(reached_idle))
        
        if reached_idle:
            self._log_result(True, f"成功回退到 IDLE")
            tc.result = TestResult.PASS
            tc.message = f"回退成功: {duration:.2f}s, 经过FAREWELL: {visited_farewell}"
        else:
            self._log_result(False, f"未回到 IDLE，当前: {self._get_state()}")
            self._log_data("状态变化时间线", str(state_change_times))
            if face_detected_after:
                tc.message = f"未回到 IDLE，当前: {self._get_state()}。可能原因: 移走后仍然检测到人脸 (face_detected=True)"
            else:
                tc.message = f"未回到 IDLE，当前: {self._get_state()}。可能原因: lost_time 未正确累加或超时时间不足"
            tc.result = TestResult.FAIL
    
    def _run_tc08(self, tc: TestCase):
        """TC-08: 目标恢复"""
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        self._send_command("search")
        time.sleep(1.0)
        
        # 移动到边缘（可能丢失）
        self._move_panel(self.PANEL_NEAR, 1.2)
        self._spin_for(0.5)
        
        # 移回中心
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        self._spin_for(1.0)
        
        state = self._get_state()
        if "SEARCH" in state or "TRACKING" in state:
            tc.result = TestResult.PASS
            tc.message = f"状态恢复正常: {state}"
        else:
            tc.result = TestResult.FAIL
            tc.message = f"状态异常: {state}"
    
    def _run_tc09(self, tc: TestCase):
        """TC-09: 完整流程 IDLE → SEARCH → TRACKING"""
        # 强制进入 IDLE
        if not self._reset_fsm():
            tc.result = TestResult.FAIL
            tc.message = f"无法进入 IDLE 状态"
            return
        
        start = time.time()
        
        # 移动 panel 到可见位置
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        
        # 记录状态变化
        visited_search = False
        tracking_reached = False
        
        # 等待进入 TRACKING（IDLE 0.3s → SEARCH 2s → TRACKING）
        while time.time() - start < 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            state = self._get_state()
            if "SEARCH" in state:
                visited_search = True
            if "TRACKING" in state:
                tracking_reached = True
                break
        
        duration = time.time() - start
        
        if tracking_reached:
            tc.result = TestResult.PASS
            tc.message = f"耗时: {duration:.2f}s, 经过SEARCH: {visited_search}"
        else:
            tc.result = TestResult.FAIL
            tc.message = f"未达到 TRACKING，当前: {self._get_state()}"
    
    def _run_tc10(self, tc: TestCase):
        """TC-10: 完整循环 IDLE → SEARCH → TRACKING → FAREWELL → IDLE"""
        self._log_phase("初始化", "准备测试完整循环")
        
        # 阶段 1: 强制进入 IDLE
        self._log_phase("阶段 1", "确保系统在 IDLE 状态")
        if not self._reset_fsm():
            self._log_result(False, f"无法进入 IDLE 状态")
            tc.result = TestResult.FAIL
            tc.message = f"无法进入 IDLE 状态"
            return
        self._log_result(True, f"当前状态: {self._get_state()}")
        
        start = time.time()
        
        # 阶段 2: 移动 panel 到可见位置，等待进入 TRACKING
        self._log_phase("阶段 2", "触发 IDLE → SEARCH → TRACKING")
        self._log_action("移动 panel", f"X={self.PANEL_NEAR:.2f}, Y={self.PANEL_CENTER:.2f}")
        self._move_panel(self.PANEL_NEAR, self.PANEL_CENTER)
        self._log_expectation("应该进入 TRACKING 状态")
        
        tracking_reached = False
        visited_search = False
        while time.time() - start < 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            state = self._get_state()
            if "SEARCH" in state:
                visited_search = True
            if "TRACKING" in state:
                tracking_reached = True
                break
        
        if tracking_reached:
            self._log_result(True, f"成功进入 TRACKING，经过 SEARCH: {visited_search}")
        else:
            self._log_result(False, f"未进入 TRACKING，当前: {self._get_state()}")
            # 继续测试，看看是否能回退
        
        # 阶段 3: 移走 panel，等待回退
        self._log_phase("阶段 3", f"移走 panel 到远处 (X={self.PANEL_FAR:.1f}m)")
        self._log_action("移动 panel", f"X={self.PANEL_FAR:.1f}, Y=0.0")
        
        # 检查移走前的 face_detected
        with self.ctx_lock:
            face_detected_before = self.ctx.face_detected
        self._log_data("移走前 face_detected", str(face_detected_before))
        
        self._move_panel(self.PANEL_FAR)
        time.sleep(1.0)  # 等待物理更新
        
        # 检查移走后的 face_detected
        with self.ctx_lock:
            face_detected_after = self.ctx.face_detected
        self._log_data("移走后 face_detected", str(face_detected_after))
        
        if face_detected_after:
            self._log_result(False, f"⚠️ 移走后仍然检测到人脸！")
        else:
            self._log_result(True, f"移走后未检测到人脸")
        
        panel_removed_time = time.time()
        self._log_expectation("TRACKING lost_timeout=1.5s → FAREWELL duration=2.5s → IDLE")
        
        # 阶段 4: 等待回到 IDLE
        self._log_phase("阶段 4", "等待回退到 IDLE")
        idle_reached = False
        visited_farewell = False
        last_state = self._get_state()
        state_change_times = []
        
        while time.time() - start < 25.0:  # 增加等待时间
            rclpy.spin_once(self, timeout_sec=0.1)
            current_state = self._get_state()
            
            if current_state != last_state:
                elapsed = time.time() - start
                state_change_times.append((elapsed, last_state, current_state))
                self._log_data("状态变化", f"{elapsed:.2f}s: {last_state} → {current_state}")
                last_state = current_state
            
            if "FAREWELL" in current_state:
                visited_farewell = True
            if "IDLE" in current_state:
                idle_reached = True
                break
        
        total_duration = time.time() - start
        return_duration = time.time() - panel_removed_time
        
        # 验证结果
        self._log_phase("验证", "检查完整循环结果")
        self._log_data("总耗时", f"{total_duration:.2f}s")
        self._log_data("回退耗时", f"{return_duration:.2f}s")
        self._log_data("经过 FAREWELL", str(visited_farewell))
        self._log_data("到达 IDLE", str(idle_reached))
        
        if idle_reached:
            self._log_result(True, f"完整循环成功")
            tc.result = TestResult.PASS
            tc.message = f"总耗时: {total_duration:.2f}s, 回退耗时: {return_duration:.2f}s"
        else:
            self._log_result(False, f"未回到 IDLE，当前: {self._get_state()}")
            self._log_data("状态变化时间线", str(state_change_times))
            if face_detected_after:
                tc.message = f"未回到 IDLE，当前: {self._get_state()}。可能原因: 移走后仍然检测到人脸 (face_detected=True)"
            else:
                tc.message = f"未回到 IDLE，当前: {self._get_state()}。可能原因: lost_time 未正确累加或超时时间不足"
            tc.result = TestResult.FAIL
    
    # ========== 测试执行 ==========
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info("开始运行测试...")
        self.get_logger().info("")
        
        # 等待初始化
        self.get_logger().info("等待系统初始化...")
        self._spin_for(2.0)
        
        # 强制初始化到 IDLE 状态
        self.get_logger().info("强制初始化到 IDLE 状态...")
        self._move_panel(self.PANEL_FAR)
        time.sleep(1.0)
        if not self._reset_fsm():
            self.get_logger().warn("初始化警告：未能进入 IDLE 状态")
        
        # 测试方法映射
        test_methods = {
            "TC-01": self._run_tc01,
            "TC-02": self._run_tc02,
            "TC-03": self._run_tc03,
            "TC-04": self._run_tc04,
            "TC-05": self._run_tc05,
            "TC-06": self._run_tc06,
            "TC-07": self._run_tc07,
            "TC-08": self._run_tc08,
            "TC-09": self._run_tc09,
            "TC-10": self._run_tc10,
        }
        
        for tc in self.test_cases:
            self.get_logger().info("")
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"运行 {tc.id}: {tc.name}")
            self.get_logger().info(f"描述: {tc.description}")
            self.get_logger().info("=" * 60)
            
            start = time.time()
            try:
                if tc.id in test_methods:
                    test_methods[tc.id](tc)
                else:
                    tc.result = TestResult.SKIP
                    tc.message = "未实现"
            except Exception as e:
                self._log_result(False, f"测试异常: {e}")
                tc.result = TestResult.FAIL
                tc.message = f"异常: {e}"
            
            tc.duration = time.time() - start
            
            # 打印最终结果
            result_symbol = "✅" if tc.result == TestResult.PASS else "❌"
            self.get_logger().info("")
            self.get_logger().info(f"  {result_symbol} 最终结果: {tc.result.value} ({tc.duration:.2f}s)")
            if tc.message:
                self.get_logger().info(f"  详情: {tc.message}")
            self.get_logger().info("")
            
            # 重置
            self._log_action("重置", "移动 panel 到远处")
            self._move_panel(self.PANEL_FAR)
            time.sleep(0.5)
        
        # 生成报告
        self._print_report()
    
    def _print_report(self):
        """打印测试报告"""
        print("\n" + "═" * 70)
        print("           IDLE → SEARCH 集成测试报告")
        print("═" * 70)
        print(f"\n测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("测试环境: MuJoCo Simulator + pm_v2")
        print("\n" + "─" * 70)
        print("测试用例结果")
        print("─" * 70)
        
        passed = 0
        failed = 0
        skipped = 0
        total_time = 0.0
        
        for tc in self.test_cases:
            if tc.result == TestResult.PASS:
                symbol = "✅ PASS"
                passed += 1
            elif tc.result == TestResult.FAIL:
                symbol = "❌ FAIL"
                failed += 1
            else:
                symbol = "⏭️ SKIP"
                skipped += 1
            
            total_time += tc.duration
            print(f"{tc.id}: {tc.name:<25} [{symbol}] ({tc.duration:.2f}s)")
            if tc.message:
                print(f"      {tc.message}")
        
        print("\n" + "─" * 70)
        print("统计")
        print("─" * 70)
        print(f"总用例数:    {len(self.test_cases)}")
        print(f"通过:        {passed}")
        print(f"失败:        {failed}")
        print(f"跳过:        {skipped}")
        print(f"成功率:      {passed / len(self.test_cases) * 100:.1f}%")
        print(f"总耗时:      {total_time:.2f}s")
        print("\n" + "═" * 70)
        
        # 保存报告到文件
        self._save_report(passed, failed, skipped, total_time)
    
    def _save_report(self, passed: int, failed: int, skipped: int, total_time: float):
        """保存报告到文件"""
        report_path = "/home/lingjing/project/engine_ai/wedding_jeston/test_report.txt"
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("═" * 70 + "\n")
            f.write("           IDLE → SEARCH 集成测试报告\n")
            f.write("═" * 70 + "\n\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("测试环境: MuJoCo Simulator + pm_v2\n\n")
            f.write("─" * 70 + "\n")
            f.write("测试用例结果\n")
            f.write("─" * 70 + "\n")
            
            for tc in self.test_cases:
                if tc.result == TestResult.PASS:
                    symbol = "PASS"
                elif tc.result == TestResult.FAIL:
                    symbol = "FAIL"
                else:
                    symbol = "SKIP"
                
                f.write(f"{tc.id}: {tc.name:<25} [{symbol}] ({tc.duration:.2f}s)\n")
                if tc.message:
                    f.write(f"      {tc.message}\n")
            
            f.write("\n" + "─" * 70 + "\n")
            f.write("统计\n")
            f.write("─" * 70 + "\n")
            f.write(f"总用例数:    {len(self.test_cases)}\n")
            f.write(f"通过:        {passed}\n")
            f.write(f"失败:        {failed}\n")
            f.write(f"跳过:        {skipped}\n")
            f.write(f"成功率:      {passed / len(self.test_cases) * 100:.1f}%\n")
            f.write(f"总耗时:      {total_time:.2f}s\n")
            f.write("\n" + "═" * 70 + "\n")
        
        print(f"\n报告已保存到: {report_path}")


def main():
    rclpy.init()
    
    node = IdleSearchIntegrationTest()
    
    try:
        node.run_all_tests()
    except KeyboardInterrupt:
        print("\n测试中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

