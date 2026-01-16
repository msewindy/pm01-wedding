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
from std_msgs.msg import String, Bool
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
        self.initial_torques = np.zeros(robot_config.NUM_JOINTS)
        self.current_positions = np.zeros(robot_config.NUM_JOINTS)
        self.current_positions = np.zeros(robot_config.NUM_JOINTS)
        
        # 运动模式与参数
        self._motion_mode = "idle" # idle, wave, scan, track
        self._motion_params = {}
        self._motion_start_time = 0.0
        
        # 目标值
        self.target_waist_offset = 0.0
        self.target_head_offset = 0.0
        self.target_arm_angles = motion_resources.POSES['neutral'].copy()
        
        # 声明此节点用的平滑参数
        # 声明此节点用的平滑参数 (进一步调优)
        self.declare_parameter('motion_head_vel', 0.5) # prev 0.8
        self.declare_parameter('motion_head_acc', 1.0) # prev 2.0
        self.declare_parameter('motion_waist_vel', 0.3) # prev 0.5
        self.declare_parameter('motion_waist_acc', 0.8) # prev 1.5
        self.declare_parameter('motion_arm_vel', 0.3) # prev 0.6 (Slowed down for stability)
        self.declare_parameter('motion_arm_acc', 0.5) # prev 1.0 (Smoother buildup)
        
        # 声明关节 PD 参数
        self.declare_parameter('pd_leg_kp', 60.0)
        self.declare_parameter('pd_leg_kd', 3.0)
        self.declare_parameter('pd_body_kp', 60.0)
        self.declare_parameter('pd_body_kd', 3.0)
        self.declare_parameter('pd_arm_kp', 20.0)
        self.declare_parameter('pd_arm_kd', 1.0)
        
        # 加载 PD 参数
        self.pd_params = {
            'LEG': {'kp': self.get_parameter('pd_leg_kp').value, 'kd': self.get_parameter('pd_leg_kd').value},
            'BODY': {'kp': self.get_parameter('pd_body_kp').value, 'kd': self.get_parameter('pd_body_kd').value},
            'ARM': {'kp': self.get_parameter('pd_arm_kp').value, 'kd': self.get_parameter('pd_arm_kd').value},
        }
        
        head_vel = self.get_parameter('motion_head_vel').value
        head_acc = self.get_parameter('motion_head_acc').value
        waist_vel = self.get_parameter('motion_waist_vel').value
        waist_acc = self.get_parameter('motion_waist_acc').value
        arm_vel = self.get_parameter('motion_arm_vel').value
        arm_acc = self.get_parameter('motion_arm_acc').value
        
        self.head_smoother = SmoothControl(head_vel, head_acc, self._control_period)
        self.waist_smoother = SmoothControl(waist_vel, waist_acc, self._control_period)
        
        self.arm_smoothers = {}
        self.arm_smoothers = {}
        # 初始化所有手臂关节的平滑器 (Left + Right)
        active_arm_joints = robot_config.JOINTS_ARM_LEFT + robot_config.JOINTS_ARM_RIGHT
        for idx in active_arm_joints:
            self.arm_smoothers[idx] = SmoothControl(arm_vel, arm_acc, self._control_period)
            
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

        # 系统就绪状态发布 (Latching QoS to ensure late subscribers get it)
        qos_latching = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, # Latching behavior
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.ready_pub = self.create_publisher(Bool, '/wedding/system/ready', qos_latching)

        
        self.init_timer = self.create_timer(0.1, self._wait_for_joint_state)
        self.control_timer = None
        
        self.get_logger().info(f"MotionAdapterNode initialized. Rate: {control_rate}Hz")
        self.get_logger().info(f"Loaded PD Params - LEG KP: {self.pd_params['LEG']['kp']}, KD: {self.pd_params['LEG']['kd']}")
        self.get_logger().info(f"Loaded PD Params - BODY KP: {self.pd_params['BODY']['kp']}, KD: {self.pd_params['BODY']['kd']}")
        self.get_logger().info(f"Interface Protocol: {self._use_interface_protocol}")

    def _wait_for_joint_state(self):
        if not self._use_interface_protocol: return
        if self._received_joint_state:
            self.init_timer.cancel()
            self.control_timer = self.create_timer(self._control_period, self._control_loop)
            self._initialized = True
            self.get_logger().info("Control loop started.")
            
            # 发布系统就绪信号
            msg = Bool()
            msg.data = True
            self.ready_pub.publish(msg)
            self.get_logger().info("System Ready: Initial joint state received. /wedding/system/ready -> True")

            # Reset smoothers
            self.head_smoother.reset(0.0)
            self.waist_smoother.reset(0.0)
            active_arm_joints = robot_config.JOINTS_ARM_LEFT + robot_config.JOINTS_ARM_RIGHT
            for idx in active_arm_joints:
                self.arm_smoothers[idx].reset(0.0)
                
    def _on_joint_state(self, msg):
        with self._lock:
            if len(msg.position) >= robot_config.NUM_JOINTS:
                positions = np.array(msg.position[:robot_config.NUM_JOINTS])
                # 在控制回路启动前，持续更新初始姿态 (Follow Mode)
                # 这样可以防止 "Capture" 和 "Act" 之间的时间差导致机器人试图回到过去的姿态 (而该姿态可能已经是倒下的过程)
                if not self._initialized:
                    self.initial_positions = positions.copy()
                    
                    # LOG POSITIONS (Restored for Debug)
                    self.get_logger().info(f"Initial Positions Captured: {self.initial_positions[:6]}")
                    if len(self.initial_positions) > 18:
                         self.get_logger().debug(f"L_SHOULDER_PITCH (13): {self.initial_positions[13]}")
                         self.get_logger().debug(f"R_SHOULDER_PITCH (18): {self.initial_positions[18]}")
                         
                         limit_l = robot_config.LIMITS.get('L_SHOULDER_PITCH_MIN', -999)
                         self.get_logger().debug(f"Limit L_SHOULDER_PITCH_MIN: {limit_l}")

                    # 强制 Head/Waist reference 为 0 (保持中立)
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
             if self._motion_mode == "sway":
                 # 触发柔和停止 (Sway Decay)
                 self._motion_mode = "sway_stopping"
                 self._stop_start_time = self.get_clock().now().nanoseconds / 1e9
                 self.get_logger().info("Motion stopping (sway decay)...")
             else:
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
        
        # Debug Log (Every ~10 messages)
        # using getattr to avoid init issues
        if not hasattr(self, '_look_at_log_count'): self._look_at_log_count = 0
        self._look_at_log_count += 1
        if self._look_at_log_count % 10 == 0:
             self.get_logger().info(f"LookAt received: x={x:.3f} -> head_off={self.target_head_offset:.3f}")

    def _generate_sway(self, t: float):
        """生成摆动波形"""
        params = self._motion_params
        period = params.get('period', 3.0)
        head_amp = params.get('head_amp', 0.1)
        waist_amp = params.get('waist_amp', 0.05)
        phase_diff = params.get('phase_diff', 0.0)
        
        w = 2 * math.pi / period
        
        # 增加 Ramp-up: 前 1.5 秒振幅线性增加
        ramp_duration = 1.5
        ramp_factor = min(1.0, t / ramp_duration)
        
        # 使用 smooth step 函数 (3x^2 - 2x^3) 让 fade-in 更柔和
        # ramp_factor = ramp_factor * ramp_factor * (3 - 2 * ramp_factor)
        
        self.target_head_offset = head_amp * math.sin(w * t) * ramp_factor
        self.target_waist_offset = waist_amp * math.sin(w * t + phase_diff) * ramp_factor

    def _generate_wave(self, t: float):
        """生成挥手动作"""
        # 挥手主要通过叠加额外的 arm 角度实现
        # 这里简化：修改 arm_angles 中的某个关节
        params = self._motion_params
        period = params.get('period', 1.0)
        amp = params.get('arm_amp', 0.3)
        idx = params.get('joint_idx', 14) 
        
        w = 2 * math.pi / period
        
        # 增加 Ramp-up 逻辑：前 1.0 秒振幅线性增加
        # 这确保了如果从手臂下垂状态开始挥手，不会在提升过程中剧烈摆动
        ramp_duration = 1.0
        ramp_factor = min(1.0, t / ramp_duration)
        
        wave_val = amp * math.sin(w * t) * ramp_factor
        
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

    def _generate_multi_sine(self, t: float):
        """生成多关节正弦波 (通用)"""
        params = self._motion_params
        joints_config = params.get('joints', [])
        
        offsets = {}
        
        for cfg in joints_config:
            idx = cfg.get('id')
            amp = cfg.get('amp', 0.1)
            period = cfg.get('period', 1.0)
            phase = cfg.get('phase', 0.0)
            
            w = 2 * math.pi / period
            
            # 使用 Ramp-up 防止突变
            ramp_duration = 1.0
            ramp_factor = min(1.0, t / ramp_duration)
            
            val = amp * math.sin(w * t + phase) * ramp_factor
            offsets[idx] = val
            
        return offsets

    def _control_loop(self):
        if not self._initialized or not self._use_interface_protocol: return
        
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._motion_start_time
        
        with self._lock:
            # 1. 根据模式生成目标值
            wave_offset = 0.0
            wave_joint = -1
            multi_sine_offsets = {}
            
            if self._motion_mode == "sway":
                self._generate_sway(dt)
            elif self._motion_mode == "wave":
                wave_joint, wave_offset = self._generate_wave(dt)
                # 挥手时保持头部不动或微动
                self.target_head_offset = 0.0
                self.target_waist_offset = 0.0
            elif self._motion_mode == "multi_sine":
                 multi_sine_offsets = self._generate_multi_sine(dt)
                 # 可以在此模式下保持头部/腰部不动，或者允许它们如果被配置在 multi_sine 里的话
                 # 目前 multi_sine 只针对 offsets 字典，我们在后面应用
                 pass
            elif self._motion_mode == "scan":
                self._generate_scan(dt)
            elif self._motion_mode == "track":
                # target 由 _on_look_at 更新
                pass
                # Debug Log for Control Loop (Every ~500 cycles = 1s)
                if not hasattr(self, '_ctrl_debug_count'): self._ctrl_debug_count = 0
                self._ctrl_debug_count += 1
                if self._ctrl_debug_count % 500 == 0:
                    self.get_logger().info(f"Control[TRACK]: target_head={self.target_head_offset:.3f}, curr_head={self.current_head_offset:.3f}")
            elif self._motion_mode == "sway_stopping":
                # Sway 柔和停止逻辑
                start_dt = now - self._motion_start_time
                self._generate_sway(start_dt) # 继续生成目标值 (保持相位连续)
                
                # 计算衰减
                DECAY_DURATION = 1.5
                stop_dt = now - self._stop_start_time
                alpha = 1.0 - (stop_dt / DECAY_DURATION)
                
                if alpha <= 0:
                    # 衰减结束，彻底停止
                    self._motion_mode = "idle"
                    self.target_head_offset = 0.0
                    self.target_waist_offset = 0.0
                    self._motion_params = {}
                    self.get_logger().info("Motion fully stopped (decay complete)")
                else:
                    # 应用衰减系数
                    self.target_head_offset *= alpha
                    self.target_waist_offset *= alpha
                
            else:
                # Idle/Stop: 回中
                self.target_head_offset = 0.0
                self.target_waist_offset = 0.0
                
            # 2. 平滑处理
            self.current_head_offset = self.head_smoother.update(self.target_head_offset)
            self.current_waist_offset = self.waist_smoother.update(self.target_waist_offset)
            
            active_arm_joints = robot_config.JOINTS_ARM_LEFT + robot_config.JOINTS_ARM_RIGHT
            for idx in active_arm_joints:
                base_angle = self.target_arm_angles.get(idx, 0.0)
                
                # 叠加 Wave 效果 (Legacy)
                if idx == wave_joint:
                    base_angle += wave_offset
                
                # 叠加 Multi-Sine 效果 (New)
                if idx in multi_sine_offsets:
                    base_angle += multi_sine_offsets[idx]
                
                self.current_arm_angles[idx] = self.arm_smoothers[idx].update(base_angle)
            
            # 3. 发布命令
            self._publish_joint_command()

    def _publish_joint_command(self):
        msg = self._JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.position = self.initial_positions.tolist() # 初始化为 Initial
        msg.velocity = [0.0] * robot_config.NUM_JOINTS
        # 恢复旧逻辑：不发送 Feed Forward Torque (设为0)
        msg.feed_forward_torque = [0.0] * robot_config.NUM_JOINTS
        msg.torque = [0.0] * robot_config.NUM_JOINTS
        msg.stiffness = [0.0] * robot_config.NUM_JOINTS
        msg.damping = [0.0] * robot_config.NUM_JOINTS
        
        # 填充 PD 参数
        for i in range(robot_config.NUM_JOINTS):
            if i in robot_config.JOINTS_LEG:
                p = self.pd_params['LEG']
            elif i in robot_config.JOINTS_ARM_ALL: # 包含腰部
                # 细分：腰部用 BODY，手臂用 ARM
                if i == robot_config.WAIST_YAW_IDX:
                     p = self.pd_params['BODY']
                else:
                     p = self.pd_params['ARM']
            else: # 头部
                p = self.pd_params['BODY']
                
            msg.stiffness[i] = p['kp']
            msg.damping[i] = p['kd']
            
        # 叠加控制 (Superposition Control)
        # 逻辑：Final = Initial + Offset
        
        # Helper: Clamp value with Min/Max support
        def clamp(val, key):
             if key in robot_config.LIMITS:
                 limit = robot_config.LIMITS[key]
                 return max(-limit, min(limit, val))
             
             # Check for Asymmetric Limits
             min_key = key + "_MIN"
             max_key = key + "_MAX"
             if min_key in robot_config.LIMITS and max_key in robot_config.LIMITS:
                 l_min = robot_config.LIMITS[min_key]
                 l_max = robot_config.LIMITS[max_key]
                 return max(l_min, min(l_max, val))
                 
             return val

        # Apply Waist Offset
        # msg.position[WAIST] = Initial[WAIST] + Offset
        final_waist = self.initial_positions[robot_config.WAIST_YAW_IDX] + self.current_waist_offset
        # 注意：Clamp的是最终绝对位置吗？还是Offset？
        # 通常限位是针对绝对位置的。
        # 但这里的 Clamp 函数是针对 Offset 写的 (如果是 symmetric limit)。
        # 如果 LIMITS 定义的是绝对范围 (如 -3.14 ~ 2.8)，我们需要 Clamp(Final)。
        # 之前的代码 Clamp(Offset) 是因为假设 Initial=0。
        # 现在恢复 Initial != 0，必须 Clamp(Final)。
        
        # 重新定义 Clamp 适配绝对值
        # 这里直接用前面定义的 clamp 函数，传入 绝对值
        
        msg.position[robot_config.WAIST_YAW_IDX] = clamp(final_waist, robot_config.JOINT_LIMIT_MAP[robot_config.WAIST_YAW_IDX])
        
        # Apply Head Offset
        final_head = self.initial_positions[robot_config.HEAD_YAW_IDX] + self.current_head_offset
        msg.position[robot_config.HEAD_YAW_IDX] = clamp(final_head, robot_config.JOINT_LIMIT_MAP[robot_config.HEAD_YAW_IDX])
        
        # Apply Arm Offsets
        active_arm_joints = robot_config.JOINTS_ARM_LEFT + robot_config.JOINTS_ARM_RIGHT
        for idx in active_arm_joints:
            # Offset comes from smoothers
            offset = self.current_arm_angles[idx]
            final_pos = self.initial_positions[idx] + offset
            
            # Clamp Absolute Position
            if idx in robot_config.JOINT_LIMIT_MAP:
                 key = robot_config.JOINT_LIMIT_MAP[idx]
                 final_pos = clamp(final_pos, key)
            
            msg.position[idx] = final_pos
            
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

