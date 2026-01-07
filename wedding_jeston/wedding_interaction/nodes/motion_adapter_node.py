"""
运动适配器节点

将 FSM 的注视目标转换为实际的关节命令
用于仿真环境调试

关键：必须控制所有 24 个关节，保持高刚度以维持稳定
参考：engineai_ros2_workspace/src/interface_example/src/hold_joint_status.cc
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import numpy as np
import math
import threading


class MotionAdapterNode(Node):
    """
    运动适配器节点
    
    功能：
    - 订阅 FSM 的 look_at 目标
    - 保持所有关节稳定（高刚度）
    - 头部/腰部执行小幅注视运动
    
    关节索引（PM01 V2, 24 DOF）：
    - J00-J05: 左腿 (6 DOF)
    - J06-J11: 右腿 (6 DOF)  
    - J12: 腰部 Yaw (1 DOF)
    - J13-J17: 左臂 (5 DOF)
    - J18-J22: 右臂 (5 DOF)
    - J23: 头部 Yaw (1 DOF)
    """
    
    # 关节配置
    NUM_JOINTS = 24
    WAIST_YAW_IDX = 12
    HEAD_YAW_IDX = 23
    
    # 腿部关节索引（需要高刚度保持稳定）
    LEG_JOINT_IDX = list(range(0, 12))  # J00-J11
    
    # 手臂关节索引
    ARM_JOINT_IDX = list(range(13, 23))  # J13-J22
    
    # 关节限位（弧度）- 增大运动范围
    WAIST_YAW_LIMIT = 1.0472  # ±0.4 rad ≈ ±30° (增大)
    HEAD_YAW_LIMIT = 0.5236   # ±0.6 rad ≈ ±60° (增大)
    
    # PD 参数 - 参考 hold_joint_status.cc
    LEG_KP = 400.0   # 腿部高刚度保持稳定
    LEG_KD = 5.0
    BODY_KP = 200.0  # 腰部/头部中等刚度
    BODY_KD = 3.0
    ARM_KP = 100.0   # 手臂低刚度
    ARM_KD = 2.0
    
    def __init__(self):
        super().__init__('motion_adapter_node')
        
        # 声明参数
        self.declare_parameter('control_rate', 500.0)  # 500Hz 与示例一致
        self.declare_parameter('smooth_factor', 1.0)  # 更小的平滑系数，更平滑
        
        control_rate = self.get_parameter('control_rate').value
        self.smooth_factor = self.get_parameter('smooth_factor').value
        
        # 状态标志
        self._initialized = False
        self._received_joint_state = False
        self._lock = threading.Lock()
        
        # 初始关节位置（等待从仿真器获取）
        self.initial_positions = np.zeros(self.NUM_JOINTS)
        self.current_positions = np.zeros(self.NUM_JOINTS)
        
        # 目标偏移角度（相对于初始位置）
        self.target_waist_offset = 0.0
        self.target_head_offset = 0.0
        
        # 当前偏移角度（平滑后）
        self.current_waist_offset = 0.0
        self.current_head_offset = 0.0
        
        # QoS 配置 - 必须与仿真器/示例代码一致！
        # 参考: message_handler.cc 和 ros_interface.cc
        # 使用 best_effort + volatile，与仿真器订阅端匹配
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3  # 与 message_handler.cc 一致
        )
        
        # ========== 动态导入 interface_protocol ==========
        try:
            from interface_protocol.msg import JointState, JointCommand
            self._JointState = JointState
            self._JointCommand = JointCommand
            self._use_interface_protocol = True
            
            # 订阅关节状态反馈 - 使用与仿真器一致的 QoS
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/hardware/joint_state',
                self._on_joint_state,
                qos_sensor
            )
            
            # 发布关节命令 - 必须使用 best_effort，与仿真器订阅端匹配！
            self.joint_cmd_pub = self.create_publisher(
                JointCommand,
                '/hardware/joint_command',
                qos_sensor
            )
            
            self.get_logger().info("Using interface_protocol messages")
            
        except ImportError:
            self._use_interface_protocol = False
            self.get_logger().error(
                "interface_protocol not found! Cannot control robot."
            )
            return
        
        # ========== 订阅 FSM look_at 目标 ==========
        # 内部通信可以使用 reliable
        qos_internal = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.look_at_sub = self.create_subscription(
            PointStamped,
            '/wedding/motion/look_at',
            self._on_look_at,
            qos_internal
        )
        
        # ========== 定时器 ==========
        # 等待初始化完成后再开始控制
        self.init_timer = self.create_timer(0.1, self._wait_for_joint_state)
        
        # 控制定时器（初始化完成后启动）
        control_period = 1.0 / control_rate
        self.control_timer = None
        self._control_period = control_period
        
        self.get_logger().info(f"MotionAdapterNode created, waiting for joint state...")
        self.get_logger().info(f"Control rate: {control_rate} Hz")
    
    def _wait_for_joint_state(self) -> None:
        """等待获取初始关节状态"""
        if not self._use_interface_protocol:
            return
        
        if self._received_joint_state:
            # 已接收到关节状态，初始化完成
            self.init_timer.cancel()
            
            # 启动控制定时器
            self.control_timer = self.create_timer(
                self._control_period, 
                self._control_loop
            )
            
            self._initialized = True
            self.get_logger().info("Initialization complete, starting control loop")
            self.get_logger().info(f"Initial positions received: {self.NUM_JOINTS} joints")
        else:
            # ROS2 Python 没有 info_throttle，使用计数器节流
            if not hasattr(self, '_wait_log_count'):
                self._wait_log_count = 0
            self._wait_log_count += 1
            if self._wait_log_count % 10 == 1:  # 每 1 秒打印一次 (0.1s * 10)
                self.get_logger().info(
                    "Waiting for first joint state from simulator..."
                )
    
    def _on_joint_state(self, msg) -> None:
        """处理关节状态反馈"""
        with self._lock:
            if len(msg.position) >= self.NUM_JOINTS:
                positions = np.array(msg.position[:self.NUM_JOINTS])
                
                if not self._received_joint_state:
                    # 首次接收，保存为初始位置
                    self.initial_positions = positions.copy()
                    self._received_joint_state = True
                    self.get_logger().info("First joint state received!")
                    # 打印初始位置用于调试
                    self.get_logger().info(f"Initial positions (first 6 legs L): {positions[0:6]}")
                    self.get_logger().info(f"Initial positions (6-12 legs R): {positions[6:12]}")
                    self.get_logger().info(f"Initial positions (12 waist): {positions[12]}")
                    self.get_logger().info(f"Initial positions (13-18 arm L): {positions[13:18]}")
                    self.get_logger().info(f"Initial positions (18-23 arm R): {positions[18:23]}")
                    self.get_logger().info(f"Initial positions (23 head): {positions[23]}")
                
                self.current_positions = positions
    
    def _on_look_at(self, msg: PointStamped) -> None:
        """
        处理 look_at 目标
        
        输入：归一化坐标 (0, 1)，中心为 (0.5, 0.5)
        输出：目标关节偏移角度
        """
        if not self._initialized:
            return
        
        # 归一化坐标转换为角度偏移
        # 图像坐标系：x=0为左侧，x=1为右侧（标准OpenCV/MediaPipe）
        # 映射关系：
        #   x=0（图像左侧）→ 机器人左转 → 看左侧
        #   x=0.5（图像中心）→ 机器人中心 → 看中心
        #   x=1（图像右侧）→ 机器人右转 → 看右侧
        x = msg.point.x  # 0-1（0=左侧，1=右侧）
        
        # 转换为 -1 到 1 范围
        # 标准映射：x=0（左侧）→ x_offset=-1, x=1（右侧）→ x_offset=1
        x_offset = (x - 0.5) * 2.0  # -1 (左) 到 1 (右)
        
        # 转换为偏移角度（弧度）
        self.target_head_offset = x_offset * self.HEAD_YAW_LIMIT  # 不反转符号
        self.target_waist_offset = x_offset * self.WAIST_YAW_LIMIT   # 不反转符号

        # self.get_logger().info(
        #     f"Head offset mismatch: current={self.current_head_offset:.6f}rad, "
        #     f"target={self.target_head_offset:.6f}rad, "
        #     f"diff={abs(self.current_head_offset - self.target_head_offset):.6f}rad"
        # )

        # self.get_logger().info(
        #     f"Waist offset mismatch: current={self.current_waist_offset:.6f}rad, "
        #     f"target={self.target_waist_offset:.6f}rad, "
        #     f"diff={abs(self.current_waist_offset - self.target_waist_offset):.6f}rad"
        # )
    
    def _control_loop(self) -> None:
        """
        控制循环 - 500Hz
        
        关键：必须控制所有关节，保持腿部高刚度
        """
        if not self._initialized or not self._use_interface_protocol:
            return
        
        with self._lock:
            # 平滑过渡偏移角度
            self.current_head_offset += self.smooth_factor * (
                self.target_head_offset - self.current_head_offset
            )
            self.current_waist_offset += self.smooth_factor * (
                self.target_waist_offset - self.current_waist_offset
            )

            
            # 构建关节命令
            self._publish_joint_command()
    
    def _publish_joint_command(self) -> None:
        """
        发布关节命令
        
        策略：
        - 腿部：保持初始位置，高刚度
        - 腰部/头部：初始位置 + 偏移，中等刚度
        - 手臂：保持初始位置，低刚度
        """
        msg = self._JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 初始化数组 - 全部基于初始位置
        msg.position = self.initial_positions.tolist()
        msg.velocity = [0.0] * self.NUM_JOINTS
        msg.feed_forward_torque = [0.0] * self.NUM_JOINTS
        msg.torque = [0.0] * self.NUM_JOINTS
        msg.stiffness = [0.0] * self.NUM_JOINTS
        msg.damping = [0.0] * self.NUM_JOINTS
        
        # 设置各关节 PD 参数
        for i in range(self.NUM_JOINTS):
            if i in self.LEG_JOINT_IDX:
                # 腿部：高刚度保持稳定
                msg.stiffness[i] = self.LEG_KP
                msg.damping[i] = self.LEG_KD
            elif i in self.ARM_JOINT_IDX:
                # 手臂：低刚度
                msg.stiffness[i] = self.ARM_KP
                msg.damping[i] = self.ARM_KD
            else:
                # 腰部、头部：中等刚度
                msg.stiffness[i] = self.BODY_KP
                msg.damping[i] = self.BODY_KD
        
        # 设置腰部位置 = 初始位置 + 偏移
        msg.position[self.WAIST_YAW_IDX] = (
            self.initial_positions[self.WAIST_YAW_IDX] + self.current_waist_offset
        )
        
        # 设置头部位置 = 初始位置 + 偏移
        msg.position[self.HEAD_YAW_IDX] = (
            self.initial_positions[self.HEAD_YAW_IDX] + self.current_head_offset
        )
        
        # parallel_parser_type: 0 = 不使用并行解析
        msg.parallel_parser_type = 0
        
        self.joint_cmd_pub.publish(msg)
        
        # 调试日志（每 500 次打印一次，即每秒一次 @ 500Hz）
        if not hasattr(self, '_pub_count'):
            self._pub_count = 0
        self._pub_count += 1
        # 调试日志已移除，如需调试请使用 debug 级别
        # if self._pub_count % 500 == 1:
        #     self.get_logger().debug(...)


def main(args=None):
    rclpy.init(args=args)
    node = MotionAdapterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
