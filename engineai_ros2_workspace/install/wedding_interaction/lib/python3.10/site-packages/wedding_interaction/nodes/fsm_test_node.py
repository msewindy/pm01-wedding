"""
FSM 测试节点

用于测试 FSM 逻辑，模拟感知数据输入
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
import math
import time


class FSMTestNode(Node):
    """
    FSM 测试节点
    
    模拟感知数据，用于测试 FSM 逻辑
    """
    
    def __init__(self):
        super().__init__('fsm_test_node')
        
        # 发布者
        self.target_pub = self.create_publisher(
            PointStamped,
            '/wedding/perception/target',
            10
        )
        
        self.gesture_pub = self.create_publisher(
            String,
            '/wedding/perception/gesture',
            10
        )
        
        self.too_close_pub = self.create_publisher(
            Bool,
            '/wedding/perception/too_close',
            10
        )
        
        self.command_pub = self.create_publisher(
            String,
            '/wedding/fsm/command',
            10
        )
        
        # 订阅者
        self.state_sub = self.create_subscription(
            String,
            '/wedding/fsm/state',
            self._on_state,
            10
        )
        
        self.pose_sub = self.create_subscription(
            String,
            '/wedding/motion/pose',
            self._on_pose,
            10
        )
        
        self.speech_sub = self.create_subscription(
            String,
            '/wedding/audio/play',
            self._on_speech,
            10
        )
        
        # 当前状态
        self.current_state = ""
        
        # 测试场景
        self.test_scenarios = [
            ("idle", 3.0),           # 空闲 3 秒
            ("face_detect", 5.0),    # 模拟人脸检测 5 秒
            ("gesture_heart", 3.0),  # 模拟比心手势 3 秒
            ("wait", 8.0),           # 等待合影完成
            ("face_lost", 3.0),      # 人脸丢失
            ("wait", 4.0),           # 等待送别完成
        ]
        self.current_scenario_idx = 0
        self.scenario_start_time = time.time()
        
        # 定时器
        self.timer = self.create_timer(0.1, self._run_test)  # 10 Hz
        
        self.get_logger().info("FSM Test Node started")
        self.get_logger().info("Test scenarios: " + str([s[0] for s in self.test_scenarios]))
    
    def _on_state(self, msg: String) -> None:
        """状态回调"""
        if msg.data != self.current_state:
            self.current_state = msg.data
            self.get_logger().info(f"[STATE] {self.current_state}")
    
    def _on_pose(self, msg: String) -> None:
        """Pose 回调"""
        self.get_logger().info(f"[POSE] {msg.data}")
    
    def _on_speech(self, msg: String) -> None:
        """语音回调"""
        self.get_logger().info(f"[SPEECH] {msg.data}")
    
    def _run_test(self) -> None:
        """运行测试"""
        if self.current_scenario_idx >= len(self.test_scenarios):
            self.get_logger().info("All test scenarios completed!")
            self.timer.cancel()
            return
        
        scenario, duration = self.test_scenarios[self.current_scenario_idx]
        elapsed = time.time() - self.scenario_start_time
        
        if elapsed > duration:
            # 进入下一个场景
            self.current_scenario_idx += 1
            self.scenario_start_time = time.time()
            if self.current_scenario_idx < len(self.test_scenarios):
                next_scenario = self.test_scenarios[self.current_scenario_idx][0]
                self.get_logger().info(f"\n>>> Entering scenario: {next_scenario}")
            return
        
        # 执行当前场景
        if scenario == "idle":
            # 不发布任何数据
            pass
        
        elif scenario == "face_detect":
            # 发布人脸位置（模拟移动的人脸）
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = 0.5 + 0.1 * math.sin(elapsed * 2)  # 左右移动
            msg.point.y = 0.5
            msg.point.z = 1.5  # 距离 1.5m
            self.target_pub.publish(msg)
        
        elif scenario == "gesture_heart":
            # 继续发布人脸
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = 0.5
            msg.point.y = 0.5
            msg.point.z = 1.5
            self.target_pub.publish(msg)
            
            # 发布比心手势
            gesture_msg = String()
            gesture_msg.data = "heart"
            self.gesture_pub.publish(gesture_msg)
        
        elif scenario == "face_lost":
            # 不发布人脸（模拟丢失）
            pass
        
        elif scenario == "wait":
            # 不发布任何数据，等待状态变化
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FSMTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

