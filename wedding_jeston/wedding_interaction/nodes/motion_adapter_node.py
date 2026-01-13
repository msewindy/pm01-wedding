"""
运动适配器节点

功能：
- 订阅 FSM 的指令 (set_pose, set_motion)
- 保持所有关节稳定（高刚度）
- 内置波形发生器 (Sway, Wave, Scan) 处理持续运动
- 接收直接控制指令 (LookAt)

设计原则：
- 数据驱动：参数从 config 加载
- 策略模式：根据 current_mode 决定 output
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import numpy as np
import math
import threading
import json

from ..config import robot_config
from ..config import motion_resources


class SmoothControl:
    """梯形速度规划平滑器"""
    def __init__(self, max_vel: float, max_acc: float, dt: float):
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.dt = dt
        self.current_pos = 0.0
        self.current_vel = 0.0
    
    def reset(self, initial_pos: float):
        self.current_pos = initial_pos
        self.current_vel = 0.0
        
    def update(self, target_pos: float) -> float:
        pos_error = target_pos - self.current_pos
        if abs(pos_error) < 1e-4 and abs(self.current_vel) < 1e-4:
            self.current_vel = 0.0
            self.current_pos = target_pos
            return self.current_pos
            
        direction = 1.0 if pos_error > 0 else -1.0
        v_limit = math.sqrt(2.0 * self.max_acc * abs(pos_error))
        target_vel = min(v_limit, self.max_vel) * direction
        
        max_dv = self.max_acc * self.dt
        vel_diff = target_vel - self.current_vel
        
        if abs(vel_diff) > max_dv:
            self.current_vel += max_dv if vel_diff > 0 else -max_dv
        else:
            self.current_vel = target_vel
            
        self.current_pos += self.current_vel * self.dt
        return self.current_pos


class MotionAdapterNode(Node):
    
    def __init__(self):
        super().__init__('motion_adapter_node')
        
        self.declare_parameter('control_rate', 500.0)
        control_rate = self.get_parameter('control_rate').value
        self._control_period = 1.0 / control_rate
        
        # 状态标志
        self._initialized = False
        self._received_joint_state = False
        self._lock = threading.Lock()
        
        # 关节位置
        self.initial_positions = np.zeros(robot_config.NUM_JOINTS)
        self.current_positions = np.zeros(robot_config.NUM_JOINTS)
        
        # 运动模式与参数
        self._motion_mode = "idle" # idle, wave, scan, track
        self._motion_params = {}
        self._motion_start_time = 0.0
        
        # 目标值
        self.target_waist_offset = 0.0
        self.target_head_offset = 0.0
        self.target_arm_angles = motion_resources.POSES['neutral'].copy()
        
        # 平滑控制器
        # 从配置读取参数
        h_params = robot_config.DEFAULT_SMOOTH_PARAMS['HEAD']
        w_params = robot_config.DEFAULT_SMOOTH_PARAMS['WAIST']
        a_params = robot_config.DEFAULT_SMOOTH_PARAMS['ARM']
        
        self.head_smoother = SmoothControl(h_params['vel'], h_params['acc'], self._control_period)
        self.waist_smoother = SmoothControl(w_params['vel'], w_params['acc'], self._control_period)
        
        self.arm_smoothers = {}
        for idx in robot_config.JOINTS_ARM_RIGHT:
            self.arm_smoothers[idx] = SmoothControl(a_params['vel'], a_params['acc'], self._control_period)
            
        # 实际输出值 (平滑后)
        self.current_waist_offset = 0.0
        self.current_head_offset = 0.0
        self.current_arm_angles = motion_resources.POSES['neutral'].copy()

        # QoS 配置
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 动态导入 interface_protocol
        try:
            from interface_protocol.msg import JointState, JointCommand
            self._JointState = JointState
            self._JointCommand = JointCommand
            self._use_interface_protocol = True
            
            self.joint_state_sub = self.create_subscription(
                JointState, '/hardware/joint_state', self._on_joint_state, qos_sensor
            )
            self.joint_cmd_pub = self.create_publisher(
                JointCommand, '/hardware/joint_command', qos_cmd
            )
            self.get_logger().info("Using interface_protocol messages")
        except ImportError:
            self._use_interface_protocol = False
            self.get_logger().error("interface_protocol not found! Cannot control robot.")
            return

        # 订阅控制话题
        self.msg_sub_pose = self.create_subscription(String, '/wedding/motion/set_pose', self._on_set_pose, 10)
        self.msg_sub_motion = self.create_subscription(String, '/wedding/motion/set_motion', self._on_set_motion, 10)
        
        # 订阅 LookAt (仅在 track 模式下生效)
        qos_internal = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.look_at_sub = self.create_subscription(PointStamped, '/wedding/motion/look_at', self._on_look_at, qos_internal)
        
        # 初始化定时器
        self.init_timer = self.create_timer(0.1, self._wait_for_joint_state)
        self.control_timer = None
        
        self.get_logger().info(f"MotionAdapterNode initialized. Rate: {control_rate}Hz")

    def _wait_for_joint_state(self):
        if not self._use_interface_protocol: return
        if self._received_joint_state:
            self.init_timer.cancel()
            self.control_timer = self.create_timer(self._control_period, self._control_loop)
            self._initialized = True
            self.get_logger().info("Control loop started.")
            # Reset smoothers
            self.head_smoother.reset(0.0)
            self.waist_smoother.reset(0.0)
            for idx in robot_config.JOINTS_ARM_RIGHT:
                self.arm_smoothers[idx].reset(0.0)
                
    def _on_joint_state(self, msg):
        with self._lock:
            if len(msg.position) >= robot_config.NUM_JOINTS:
                positions = np.array(msg.position[:robot_config.NUM_JOINTS])
                if not self._received_joint_state:
                    self.initial_positions = positions.copy()
                    # 强制 Head/Waist reference 为 0
                    self.initial_positions[robot_config.WAIST_YAW_IDX] = 0.0
                    self.initial_positions[robot_config.HEAD_YAW_IDX] = 0.0
                    self._received_joint_state = True
                self.current_positions = positions

    def _on_set_pose(self, msg: String):
        """处理姿态切换"""
        pose_name = msg.data
        if pose_name in motion_resources.POSES:
            self.target_arm_angles = motion_resources.POSES[pose_name].copy()
            self.get_logger().info(f"Set pose: {pose_name}")
        else:
            self.get_logger().warn(f"Unknown pose: {pose_name}")

    def _on_set_motion(self, msg: String):
        """处理动态切换"""
        motion_name = msg.data
        if motion_name in motion_resources.MOTIONS:
            params = motion_resources.MOTIONS[motion_name]
            self._motion_mode = params['type']
            self._motion_params = params
            self._motion_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Set motion: {motion_name} ({self._motion_mode})")
        elif motion_name == "stop" or motion_name == "None" or not motion_name:
             self._motion_mode = "idle"
             self._motion_params = {}
             self.get_logger().info("Motion stopped")
        else:
            self.get_logger().warn(f"Unknown motion: {motion_name}")

    def _on_look_at(self, msg: PointStamped):
        """仅在 track 模式下处理 LookAt"""
        if self._motion_mode != "track":
            return
            
        x = msg.point.x # 0~1
        x_offset = (x - 0.5) * 2.0 # -1~1
        
        # 映射到限位
        self.target_head_offset = x_offset * robot_config.LIMITS['HEAD_YAW']
        self.target_waist_offset = x_offset * robot_config.LIMITS['WAIST_YAW']

    def _generate_sway(self, t: float):
        """生成摆动波形"""
        params = self._motion_params
        period = params.get('period', 3.0)
        head_amp = params.get('head_amp', 0.1)
        waist_amp = params.get('waist_amp', 0.05)
        phase_diff = params.get('phase_diff', 0.0)
        
        w = 2 * math.pi / period
        self.target_head_offset = head_amp * math.sin(w * t)
        self.target_waist_offset = waist_amp * math.sin(w * t + phase_diff)

    def _generate_wave(self, t: float):
        """生成挥手动作"""
        # 挥手主要通过叠加额外的 arm 角度实现
        # 这里简化：修改 arm_angles 中的某个关节
        params = self._motion_params
        period = params.get('period', 1.0)
        amp = params.get('arm_amp', 0.3)
        idx = params.get('joint_idx', 14) 
        
        w = 2 * math.pi / period
        wave_val = amp * math.sin(w * t)
        
        # 叠加波形到目标姿态上
        if idx in self.target_arm_angles:
             # 注意：这里会修改 self.target_arm_angles 中的值
             # 为了避免累积误差，应该基于 base pose 叠加 (TODO: 改进)
             # 简单实现：我们假设 target_arm_angles 已经是 Base Pose
             # 但 _control_loop 是高频调用的，所以不能直接 += 
             # 更好的做法：在 control_loop 里拿到 base，然后 + wave
             pass 
        return idx, wave_val

    def _generate_scan(self, t: float):
        """生成扫描动作"""
        params = self._motion_params
        period = params.get('period', 8.0)
        rng = params.get('range', 0.8)
        
        w = 2 * math.pi / period
        # 简单的 Sine 扫描
        val = rng * math.sin(w * t)
        
        self.target_head_offset = val * robot_config.LIMITS['HEAD_YAW']
        self.target_waist_offset = val * 0.3 * robot_config.LIMITS['WAIST_YAW'] # 腰部动少点

    def _control_loop(self):
        if not self._initialized or not self._use_interface_protocol: return
        
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._motion_start_time
        
        with self._lock:
            # 1. 根据模式生成目标值
            wave_offset = 0.0
            wave_joint = -1
            
            if self._motion_mode == "sway":
                self._generate_sway(dt)
            elif self._motion_mode == "wave":
                wave_joint, wave_offset = self._generate_wave(dt)
                # 挥手时保持头部不动或微动
                self.target_head_offset = 0.0
                self.target_waist_offset = 0.0
            elif self._motion_mode == "scan":
                self._generate_scan(dt)
            elif self._motion_mode == "track":
                # target 由 _on_look_at 更新
                pass
            else:
                # Idle/Stop: 回中
                self.target_head_offset = 0.0
                self.target_waist_offset = 0.0
                
            # 2. 平滑处理
            self.current_head_offset = self.head_smoother.update(self.target_head_offset)
            self.current_waist_offset = self.waist_smoother.update(self.target_waist_offset)
            
            for idx in robot_config.JOINTS_ARM_RIGHT:
                base_angle = self.target_arm_angles.get(idx, 0.0)
                # 叠加 Wave 效果
                if idx == wave_joint:
                    base_angle += wave_offset
                
                self.current_arm_angles[idx] = self.arm_smoothers[idx].update(base_angle)
            
            # 3. 发布命令
            self._publish_joint_command()

    def _publish_joint_command(self):
        msg = self._JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.position = self.initial_positions.tolist()
        msg.velocity = [0.0] * robot_config.NUM_JOINTS
        msg.feed_forward_torque = [0.0] * robot_config.NUM_JOINTS
        msg.torque = [0.0] * robot_config.NUM_JOINTS
        msg.stiffness = [0.0] * robot_config.NUM_JOINTS
        msg.damping = [0.0] * robot_config.NUM_JOINTS
        
        # 填充 PD 参数
        for i in range(robot_config.NUM_JOINTS):
            if i in robot_config.JOINTS_LEG:
                p = robot_config.DEFAULT_PD_PARAMS['LEG']
            elif i in robot_config.JOINTS_ARM_ALL: # 包含腰部
                # 细分：腰部用 BODY，手臂用 ARM
                if i == robot_config.WAIST_YAW_IDX:
                     p = robot_config.DEFAULT_PD_PARAMS['BODY']
                else:
                     p = robot_config.DEFAULT_PD_PARAMS['ARM']
            else: # 头部
                p = robot_config.DEFAULT_PD_PARAMS['BODY']
                
            msg.stiffness[i] = p['kp']
            msg.damping[i] = p['kd']
            
        # 应用偏移
        msg.position[robot_config.WAIST_YAW_IDX] += self.current_waist_offset
        msg.position[robot_config.HEAD_YAW_IDX] += self.current_head_offset
        
        for idx in robot_config.JOINTS_ARM_RIGHT:
            msg.position[idx] += self.current_arm_angles[idx]
            
        msg.parallel_parser_type = 0
        self.joint_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

