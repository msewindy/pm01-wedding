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


class SmoothControl:
    """
    梯形速度规划平滑器 (Velocity and Acceleration Limited Smoother)
    
    保证运动在最大速度和最大加速度限制内，避免急启急停
    """
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
        # 1. 计算位置误差
        pos_error = target_pos - self.current_pos
        
        # 2. 如果误差很小，直接停止
        if abs(pos_error) < 1e-4 and abs(self.current_vel) < 1e-4:
            self.current_vel = 0.0
            self.current_pos = target_pos
            return self.current_pos
            
        # 3. 计算期望速度（带符号）
        # 如果当前以最大减速度减速，能否在目标点停下？
        # stop_dist = v^2 / (2*a)
        stop_dist = (self.current_vel ** 2) / (2.0 * self.max_acc)
        
        # 确定运动方向
        direction = 1.0 if pos_error > 0 else -1.0
        
        # 检查是否需要开始减速
        # 即使方向一致，如果剩余距离小于刹车距离，也要减速（反向加速）
        # 注意：这里是一个简化的二阶数控系统逻辑
        
        # 简单实现：P控制速度，然后限制加速度
        # V_cmd = Kp * error (但我们要限制V_max)
        # 更物理的实现：
        
        # 计算在最大加速度下，要想在目标点速度为0，当前允许的最大速度
        # v_limit = sqrt(2 * a * dist)
        v_limit = math.sqrt(2.0 * self.max_acc * abs(pos_error))
        
        # 目标速度受限于 v_limit 和 max_vel
        target_vel = min(v_limit, self.max_vel) * direction
        
        # 4. 速度控制：从 current_vel 平滑过渡到 target_vel
        # 限制每一步的速度变化量 (dv = a * dt)
        max_dv = self.max_acc * self.dt
        
        vel_diff = target_vel - self.current_vel
        
        if abs(vel_diff) > max_dv:
            # 限制加速度
            self.current_vel += max_dv if vel_diff > 0 else -max_dv
        else:
            # 可以达到目标速度
            self.current_vel = target_vel
            
        # 5. 更新位置
        self.current_pos += self.current_vel * self.dt
        
        return self.current_pos


class MotionAdapterNode(Node):
    """
    运动适配器节点
    
    功能：
    - 订阅 FSM 的 look_at 目标
    - 保持所有关节稳定（高刚度）
    - 头部/腰部执行小幅注视运动 (平滑处理)
    """
    
    # 关节配置
    NUM_JOINTS = 24
    WAIST_YAW_IDX = 12
    HEAD_YAW_IDX = 23
    
    # 腿部关节索引（需要高刚度保持稳定）
    LEG_JOINT_IDX = list(range(0, 12))  # J00-J11
    
    # 手臂关节索引
    ARM_JOINT_IDX = list(range(13, 23))  # J13-J22
    
    # 右臂关节索引 (J13=R_SHOULDER_PITCH, J14=R_SHOULDER_ROLL, J15=R_ELBOW_YAW, J16=R_ELBOW_PITCH)
    # 假设 J13-J17 是右臂主要关节
    RIGHT_ARM_INDICES = [13, 14, 15, 16]
    
    # Pose 定义 (关节角度)
    # Neutral: 垂下
    POSE_NEUTRAL_ANGLES = {
        13: 0.0,  # R_SHOULDER_PITCH
        14: 0.0,  # R_SHOULDER_ROLL
        15: 0.0,  # R_ELBOW_YAW
        16: 0.0   # R_ELBOW_PITCH
    }
    
    # Mic Hold: 举起右手
    # Shoulder Pitch: 抬起 (-0.5?)
    # Elbow Pitch: 弯曲 (-1.5?)
    # 具体角度需要根据实际机器人结构调整
    POSE_MIC_HOLD_ANGLES = {
        13: -0.5, # R_SHOULDER_PITCH (向前抬起)
        14: -0.2, # R_SHOULDER_ROLL (稍微外展)
        15: 0.0,  # R_ELBOW_YAW
        16: -1.2  # R_ELBOW_PITCH (弯曲手肘)
    }
    
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
        self.declare_parameter('control_rate', 500.0)  # 500Hz
        
        # 运动平滑参数（最大速度 rad/s，最大加速度 rad/s^2）
        self.declare_parameter('head_max_vel', 1.5)    # 头部较快
        self.declare_parameter('head_max_acc', 5.0)    # 提高加速度以减少滞后 (原 3.0)
        self.declare_parameter('waist_max_vel', 1.0)   # 腰部提速
        self.declare_parameter('waist_max_acc', 4.0)   # 提高加速度匹配头部 (原 2.0)
        self.declare_parameter('arm_max_vel', 1.0)     # 手臂速度
        self.declare_parameter('arm_max_acc', 2.0)     # 手臂加速度
        
        control_rate = self.get_parameter('control_rate').value
        
        head_max_vel = self.get_parameter('head_max_vel').value
        head_max_acc = self.get_parameter('head_max_acc').value
        waist_max_vel = self.get_parameter('waist_max_vel').value
        waist_max_acc = self.get_parameter('waist_max_acc').value
        arm_max_vel = self.get_parameter('arm_max_vel').value
        arm_max_acc = self.get_parameter('arm_max_acc').value
        
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
        
        # 手臂目标状态
        self.target_arm_angles = self.POSE_NEUTRAL_ANGLES.copy()
        
        # 控制周期
        self._control_period = 1.0 / control_rate
        
        # 初始化平滑器
        self.head_smoother = SmoothControl(head_max_vel, head_max_acc, self._control_period)
        self.waist_smoother = SmoothControl(waist_max_vel, waist_max_acc, self._control_period)
        
        # 手臂平滑器 (为每个关注的关节创建一个平滑器)
        self.arm_smoothers = {}
        for idx in self.RIGHT_ARM_INDICES:
            self.arm_smoothers[idx] = SmoothControl(arm_max_vel, arm_max_acc, self._control_period)
        
        # 当前偏移角度/手臂角度（平滑后）
        self.current_waist_offset = 0.0
        self.current_head_offset = 0.0
        self.current_arm_angles = self.POSE_NEUTRAL_ANGLES.copy()
        
        # QoS 配置 - 必须与仿真器/示例代码一致！
        # 参考: message_handler.cc 和 ros_interface.cc
        # 使用 best_effort + volatile，与仿真器订阅端匹配
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
        
        # ========== 动态导入 interface_protocol ==========
        try:
            from interface_protocol.msg import JointState, JointCommand
            self._JointState = JointState
            self._JointCommand = JointCommand
            self._use_interface_protocol = True
            
            # 订阅关节状态反馈
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/hardware/joint_state',
                self._on_joint_state,
                qos_sensor
            )
            
            # 发布关节命令
            self.joint_cmd_pub = self.create_publisher(
                JointCommand,
                '/hardware/joint_command',
                qos_cmd
            )
            
            self.get_logger().info("Using interface_protocol messages")
            
        except ImportError:
            self._use_interface_protocol = False
            self.get_logger().error(
                "interface_protocol not found! Cannot control robot."
            )
            return
        
        # ========== 订阅 FSM look_at 目标 ==========
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
        
        # ========== 订阅 FSM Pose 目标 ==========
        self.pose_sub = self.create_subscription(
            String,
            '/wedding/motion/pose',
            self._on_pose,
            10
        )
        
        # ========== 定时器 ==========
        # 等待初始化完成后再开始控制
        self.init_timer = self.create_timer(0.1, self._wait_for_joint_state)
        
        # 控制定时器在 wait_for_joint_state 中启动
        self.control_timer = None
        
        self.get_logger().info(f"MotionAdapterNode created, waiting for joint state...")
        self.get_logger().info(f"Control rate: {control_rate} Hz")
        self.get_logger().info(f"Head Limit: vel={head_max_vel}, acc={head_max_acc}")
        self.get_logger().info(f"Waist Limit: vel={waist_max_vel}, acc={waist_max_acc}")
    
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
            
            # 重置平滑器到当前状态（假设初始偏移为0）
            self.head_smoother.reset(0.0)
            self.waist_smoother.reset(0.0)
            
            # 重置手臂平滑器到初始位置 (通常是0，或者当前实际位置如果不同)
            for idx in self.RIGHT_ARM_INDICES:
                # 初始目标是从0开始(相对于initial_positions的偏移)
                # 假设 initial_positions 就是 neutral
                self.arm_smoothers[idx].reset(0.0)
            
        else:
            if not hasattr(self, '_wait_log_count'):
                self._wait_log_count = 0
            self._wait_log_count += 1
            if self._wait_log_count % 10 == 1:
                self.get_logger().info("Waiting for first joint state from simulator...")
    
    def _on_joint_state(self, msg) -> None:
        """处理关节状态反馈"""
        with self._lock:
            if len(msg.position) >= self.NUM_JOINTS:
                positions = np.array(msg.position[:self.NUM_JOINTS])
                
                if not self._received_joint_state:
                    # 首次接收，保存为初始位置
                    self.initial_positions = positions.copy()
                    
                    # 强制将 Head 和 Waist 的初始位置设为 0，以确保 look_at=0.5 (Offset=0) 对应绝对正中 (Motor Zero)
                    # 解决启动时如果头/腰歪了，会导致后续控制一直有偏差的问题
                    self.initial_positions[self.WAIST_YAW_IDX] = 0.0
                    self.initial_positions[self.HEAD_YAW_IDX] = 0.0
                    
                    self._received_joint_state = True
                    self.get_logger().info(f"First joint state received! Reset Head/Waist reference to 0.0")
                
                self.current_positions = positions
    
    def _on_look_at(self, msg: PointStamped) -> None:
        """
        处理 look_at 目标
        """
        if not self._initialized:
            return
        
        # 归一化坐标(0~1)转换为偏移角度
        x = msg.point.x  # 0-1（0=左侧，1=右侧）
        x_offset = (x - 0.5) * 2.0  # -1 (左) 到 1 (右)
        
        # 转换为偏移角度（弧度）
        self.target_head_offset = x_offset * self.HEAD_YAW_LIMIT
        self.target_waist_offset = x_offset * self.WAIST_YAW_LIMIT
    
    def _on_pose(self, msg: String) -> None:
        """处理 Pose 目标"""
        pose_name = msg.data
        if pose_name == "interview_mic_hold":
            self.target_arm_angles = self.POSE_MIC_HOLD_ANGLES.copy()
            self.get_logger().info("Switching to Pose: MIC HOLD")
        elif pose_name == "neutral":
            self.target_arm_angles = self.POSE_NEUTRAL_ANGLES.copy()
            self.get_logger().info("Switching to Pose: NEUTRAL")
        else:
            # 默认 Neutral
            self.target_arm_angles = self.POSE_NEUTRAL_ANGLES.copy()
    
    def _control_loop(self) -> None:
        """
        控制循环 - 500Hz
        """
        if not self._initialized or not self._use_interface_protocol:
            return
        
        with self._lock:
            # 使用梯形速度平滑器更新偏移
            self.current_head_offset = self.head_smoother.update(self.target_head_offset)
            self.current_waist_offset = self.waist_smoother.update(self.target_waist_offset)
            
            # 更新手臂关节
            for idx in self.RIGHT_ARM_INDICES:
                 target = self.target_arm_angles.get(idx, 0.0)
                 self.current_arm_angles[idx] = self.arm_smoothers[idx].update(target)
            
            # 构建关节命令
            self._publish_joint_command()
    
    def _publish_joint_command(self) -> None:
        """
        发布关节命令
        
        策略（叠加控制 Superposition Control）：
        1. 腿部 (Legs): 保持初始位置，高刚度以维持站立稳定。
        2. 头部/腰部 (Head/Waist): 
           - 由 FSM 的 look_at 目标控制跟随 (Tracking)。
           - 位置 = 初始位置 + current_head/waist_offset。
        3. 手臂 (Arms):
           - 由 FSM 的 set_pose 目标控制姿态 (Pose)。
           - 位置 = 初始位置 + current_arm_angles (独立平滑)。
           
        通过分别计算不同关节组的目标位置，并将它们合并到同一个 JointCommand 消息中，
        实现了"一边跟随目标(动头/腰)，一边保持特定姿态(如举手)"的叠加效果。
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
                # 手臂：低刚度 (除了正在动作的右臂？为了保持动作，可能需要提高刚度)
                if i in self.RIGHT_ARM_INDICES:
                    msg.stiffness[i] = self.BODY_KP # 提高刚度以保持姿势
                    msg.damping[i] = self.BODY_KD
                else:
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
        
        # 设置右臂位置
        for idx in self.RIGHT_ARM_INDICES:
            # 假设 POSE 中定义的是相对于 0 位的绝对角度，还是相对于 Initial 的偏移？
            # 这里的 MotionAdapter 逻辑是基于 Initial Position 的。
            # 如果 Initial Position 是 垂下 (Neutral)，那么 Pose 定义的就是相对于垂下的偏移。
            # 这样比较安全。
            msg.position[idx] = self.initial_positions[idx] + self.current_arm_angles[idx]
        
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

