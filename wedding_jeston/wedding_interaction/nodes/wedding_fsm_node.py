"""
婚礼互动 FSM ROS2 节点

职责：
- 封装 WeddingFSM
- 订阅感知数据（人脸检测）
- 发布状态和动作指令
- 提供服务接口
"""

import json
import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
import numpy as np
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo, Image
import numpy as np

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

from ..fsm import (
    WeddingFSM,
    WeddingStateName,
    WeddingEvent,
)
from ..fsm.states import (
    IdleState,
    SearchState,
    TrackingState,
    InterviewState,
)
from ..action_pack.action_manager import ActionManager


class WeddingFSMNode(Node):
    """
    婚礼互动 FSM ROS2 节点
    
    订阅：
    - /wedding/perception/gesture: 手势
    - /wedding/perception/too_close: 太近标志
    - /wedding/perception/faces_json: 人脸检测结果（JSON）
    - /wedding/fsm/command: 外部命令
    
    发布：
    - /wedding/fsm/state: 当前状态
    - /wedding/motion/pose: 目标 Pose
    - /wedding/motion/look_at: 注视目标
    - /wedding/audio/play: 语音资源名
    
    服务：
    - /wedding/fsm/reset: 重置 FSM
    - /wedding/fsm/stop: 停止 FSM
    """
    
    def __init__(self):
        super().__init__('wedding_fsm_node')
        
        # 配置 Python logging 以输出到终端（用于状态类的日志）
        # 这样状态类使用 logging.getLogger() 的日志也能显示出来
        # 获取根 logger 并配置
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)
        
        # 如果还没有 handler，添加一个 StreamHandler
        if not root_logger.handlers:
            handler = logging.StreamHandler()
            handler.setLevel(logging.INFO)
            formatter = logging.Formatter('[%(name)s] %(levelname)s: %(message)s')
            handler.setFormatter(formatter)
            root_logger.addHandler(handler)
        else:
            # 如果已有 handler，更新格式和级别
            for handler in root_logger.handlers:
                handler.setLevel(logging.INFO)
                handler.setFormatter(logging.Formatter('[%(name)s] %(levelname)s: %(message)s'))
        
        # 确保 State.* 的 logger 也输出
        state_logger = logging.getLogger('State')
        state_logger.setLevel(logging.INFO)
        if not state_logger.handlers:
            handler = logging.StreamHandler()
            handler.setLevel(logging.INFO)
            formatter = logging.Formatter('[%(name)s] %(levelname)s: %(message)s')
            handler.setFormatter(formatter)
            state_logger.addHandler(handler)
        
        # 声明参数
        self.declare_parameter('fsm_rate', 25.0)  # FSM 运行频率 Hz
        # FSM Rate
        self.declare_parameter('state_pub_rate', 5.0)
        
        # Global Config
        self.declare_parameter('enable_action', True)
        self.declare_parameter('enable_speech', True)
        self.declare_parameter('audio_dir', '')

        # Idle Params
        self.declare_parameter('idle_switch_interval_min', 10.0)
        self.declare_parameter('idle_switch_interval_max', 20.0)
        self.declare_parameter('idle_greeting_probability', 0.3)
        self.declare_parameter('idle_observe_duration_min', 3.0)
        self.declare_parameter('idle_observe_duration_max', 5.0)
        self.declare_parameter('idle_face_confirm_time', 0.5)
        self.declare_parameter('idle_face_lost_tolerance', 0.2)
        
        # Tracking Params
        # Tracking Params
        self.declare_parameter('tracking_lost_timeout', 5.0)
        # REMOVED: self.declare_parameter('tracking_match_distance_threshold', 0.2)
        self.declare_parameter('tracking_stable_duration', 2.0)
        self.declare_parameter('tracking_auto_interview_duration', 1.5)
        self.declare_parameter('tracking_interview_trigger_min_lost', 0.5)
        self.declare_parameter('tracking_interview_trigger_max_lost', 1.5)
        # REMOVED: self.declare_parameter('tracking_idle_trigger_lost_time', 2.0)
        
        # Search Params
        self.declare_parameter('search_face_confirm_time', 2.0)
        self.declare_parameter('search_timeout', 10.0)
        self.declare_parameter('search_no_target_timeout', 5.0)

        # Interview Params
        self.declare_parameter('interview_greeting_duration', 3.0)
        self.declare_parameter('interview_question_duration', 2.0)
        self.declare_parameter('interview_listening_timeout', 8.0)
        self.declare_parameter('interview_silence_threshold', 1.5)
        self.declare_parameter('interview_ending_duration', 2.0)
        self.declare_parameter('interview_done_duration', 1.0)
        self.declare_parameter('interview_voice_threshold', 0.02)
        # REMOVED: self.declare_parameter('interview_match_distance_threshold', 0.2)
        
        # PID Control
        self.declare_parameter('pid_kp', 0.03)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.002)
        self.declare_parameter('pid_max_angle_change', 0.02)
        self.declare_parameter('pid_approach_threshold', 0.10)
        self.declare_parameter('pid_approach_damping', 0.3)
        
        # Face Tracker Params
        self.declare_parameter('face_tracker_pos_match_threshold', 0.3)
        
        # Load local variables for timer init
        fsm_rate = self.get_parameter('fsm_rate').value
        state_pub_rate = self.get_parameter('state_pub_rate').value

        # Load all into config dict
        fsm_config = {
            'enable_action': self.get_parameter('enable_action').value,
            'enable_speech': self.get_parameter('enable_speech').value,
            'audio_dir': self.get_parameter('audio_dir').value,
            
            # Idle
            'idle_switch_interval_min': self.get_parameter('idle_switch_interval_min').value,
            'idle_switch_interval_max': self.get_parameter('idle_switch_interval_max').value,
            'idle_greeting_probability': self.get_parameter('idle_greeting_probability').value,
            'idle_observe_duration_min': self.get_parameter('idle_observe_duration_min').value,
            'idle_observe_duration_max': self.get_parameter('idle_observe_duration_max').value,
            'idle_face_confirm_time': self.get_parameter('idle_face_confirm_time').value,
            'idle_face_lost_tolerance': self.get_parameter('idle_face_lost_tolerance').value,
            
            # Tracking
            # Tracking
            'tracking_lost_timeout': self.get_parameter('tracking_lost_timeout').value,
            # 'tracking_match_distance_threshold': self.get_parameter('tracking_match_distance_threshold').value,
            'tracking_stable_duration': self.get_parameter('tracking_stable_duration').value,
            'tracking_auto_interview_duration': self.get_parameter('tracking_auto_interview_duration').value,
            'tracking_interview_trigger_min_lost': self.get_parameter('tracking_interview_trigger_min_lost').value,
            'tracking_interview_trigger_max_lost': self.get_parameter('tracking_interview_trigger_max_lost').value,
            # 'tracking_idle_trigger_lost_time': self.get_parameter('tracking_idle_trigger_lost_time').value,
            
            # Search
            'search_face_confirm_time': self.get_parameter('search_face_confirm_time').value,
            'search_timeout': self.get_parameter('search_timeout').value,
            'search_no_target_timeout': self.get_parameter('search_no_target_timeout').value,
            
            # Interview
            'interview_greeting_duration': self.get_parameter('interview_greeting_duration').value,
            'interview_question_duration': self.get_parameter('interview_question_duration').value,
            'interview_listening_timeout': self.get_parameter('interview_listening_timeout').value,
            'interview_silence_threshold': self.get_parameter('interview_silence_threshold').value,
            'interview_ending_duration': self.get_parameter('interview_ending_duration').value,
            'interview_done_duration': self.get_parameter('interview_done_duration').value,
            'interview_voice_threshold': self.get_parameter('interview_voice_threshold').value,
            # 'interview_match_distance_threshold': self.get_parameter('interview_match_distance_threshold').value,
            
            # PID
            'pid_kp': self.get_parameter('pid_kp').value,
            'pid_ki': self.get_parameter('pid_ki').value,
            'pid_kd': self.get_parameter('pid_kd').value,
            'pid_max_angle_change': self.get_parameter('pid_max_angle_change').value,
            'pid_approach_threshold': self.get_parameter('pid_approach_threshold').value,
            'pid_approach_damping': self.get_parameter('pid_approach_damping').value,
            
            # Face Tracker
            'face_tracker_pos_match_threshold': self.get_parameter('face_tracker_pos_match_threshold').value,
        }
        
        self.get_logger().info(f"FSM Config Loaded: {fsm_config}")
        
        # 初始化 ActionManager
        self.action_manager = ActionManager(self)
        
        # 创建 FSM（传入配置、ROS2 logger 和 ActionManager）
        # 传递 ROS2 logger 给 FSM，这样状态类可以使用 ROS2 logger
        ros2_logger = self.get_logger()
        self.fsm = WeddingFSM(config=fsm_config, ros2_logger=ros2_logger, action_manager=self.action_manager)
        
        # 注册状态
        self.fsm.register_state(IdleState(self.fsm))
        self.fsm.register_state(SearchState(self.fsm))
        self.fsm.register_state(TrackingState(self.fsm))
        self.fsm.register_state(InterviewState(self.fsm))
        
        # 初始化 FSM
        self.fsm.initialize(WeddingStateName.IDLE)
        
        # QoS 配置（与 perception_node 匹配）
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        
        # System Ready Flag
        self._system_ready = False
        
        # ========== 订阅者 ==========
        
        # 监听系统就绪信号 (Transient Local to catch latched message)
        qos_latching = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.ready_sub = self.create_subscription(
            Bool,
            '/wedding/system/ready',
            self._on_system_ready,
            qos_latching
        )
        
        # 感知数据：手势
        self.gesture_sub = self.create_subscription(
            String,
            '/wedding/perception/gesture',
            self._on_gesture,
            10
        )
        
        # 感知数据：太近检测
        self.too_close_sub = self.create_subscription(
            Bool,
            '/wedding/perception/too_close',
            self._on_too_close,
            10
        )
        
        # 外部命令
        self.command_sub = self.create_subscription(
            String,
            '/wedding/fsm/command',
            self._on_command,
            10
        )
        
        # 人脸检测结果（JSON）
        # 注意：必须使用 BEST_EFFORT QoS 与 perception_node 匹配
        self.faces_sub = self.create_subscription(
            String,
            '/wedding/perception/faces_json',
            self._on_faces,
            qos_best_effort
        )
        self.get_logger().info("Subscribing to /wedding/perception/faces_json (BEST_EFFORT QoS)")
        
        # 相机信息（用于 PID 跟随）
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/head/camera_info',
            self._on_camera_info,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribing to /camera/head/camera_info for PID following")
        
        # 外部设置 look_at（用于调试等场景）
        self.set_look_at_sub = self.create_subscription(
            PointStamped,
            '/wedding/fsm/set_look_at',
            self._on_set_look_at,
            10
        )
        
        # 相机图像（用于录制等功能）
        # 注意：必须使用 BEST_EFFORT QoS 与仿真器匹配
        self.image_sub = self.create_subscription(
            Image,
            '/camera/head/rgb/image_raw',
            self._on_image,
            qos_best_effort
        )
        self.get_logger().info("Subscribing to /camera/head/rgb/image_raw for recording (BEST_EFFORT QoS)")
        
        # ========== 发布者 ==========
        
        # FSM 状态
        self.state_pub = self.create_publisher(
            String,
            '/wedding/fsm/state',
            10
        )
        
        # 目标 face_id（用于可视化）
        self.target_face_id_pub = self.create_publisher(
            String,
            '/wedding/fsm/target_face_id',
            10
        )
        
        # 动作指令：Pose
        self.pose_pub = self.create_publisher(
            String,
            '/wedding/motion/set_pose',
            10
        )
        
        # 动作指令：注视目标
        self.look_at_pub = self.create_publisher(
            PointStamped,
            '/wedding/motion/look_at',
            10
        )
        
        # 语音指令
        self.speech_pub = self.create_publisher(
            String,
            '/wedding/audio/play',
            10
        )
        
        # 调试日志发布器（用于状态类的调试日志）
        self.debug_log_pub = self.create_publisher(
            String,
            '/wedding/fsm/debug_log',
            10
        )
        
        # 将调试发布器传递给 FSM
        self.fsm._debug_pub = self.debug_log_pub
        
        # ========== 服务 ==========
        
        self.reset_srv = self.create_service(
            Trigger,
            '/wedding/fsm/reset',
            self._on_reset
        )
        
        self.stop_srv = self.create_service(
            Trigger,
            '/wedding/fsm/stop',
            self._on_stop
        )
        
        # ========== 定时器 ==========
        
        # FSM 主循环
        fsm_period = 1.0 / fsm_rate
        self.fsm_timer = self.create_timer(fsm_period, self._run_fsm)
        
        # 状态发布
        state_period = 1.0 / state_pub_rate
        self.state_timer = self.create_timer(state_period, self._publish_state)
        
        # 注册状态变化回调
        self.fsm.on_state_change(self._on_state_change)
        
        # 记录上一次的 Pose，避免重复发布
        self._last_pose = ""
        
        self.get_logger().info("WeddingFSMNode initialized")
        self.get_logger().info(f"FSM rate: {fsm_rate} Hz, State pub rate: {state_pub_rate} Hz")
    
    def _on_system_ready(self, msg: Bool) -> None:
        """从 MotionAdapter 接收系统就绪信号"""
        if msg.data:
            self._system_ready = True
            self.get_logger().info("System READY signal received. FSM logic enabled.")
    
    # ========== 感知回调 ==========
    
    
    def _on_gesture(self, msg: String) -> None:
        """处理手势"""
        gesture = msg.data.lower()
        self.fsm.data.perception.gesture = gesture
        if gesture not in ["none", ""]:
            self.fsm.data.perception.gesture_confidence = 0.8
        else:
            self.fsm.data.perception.gesture_confidence = 0.0
    
    def _on_too_close(self, msg: Bool) -> None:
        """处理太近检测"""
        self.fsm.data.perception.too_close = msg.data
    
    def _on_faces(self, msg: String) -> None:
        """处理人脸检测结果（JSON）"""
        try:
            data = json.loads(msg.data)
            faces_data = data.get('faces', [])
            
            # 转换为 FaceInfo 对象
            import time
            from ..perception import FaceInfo
            faces = []
            for f in faces_data:
                # 获取时间戳，如果没有则使用当前时间
                timestamp = f.get('timestamp', time.time())
                face = FaceInfo(
                    x=f.get('x', 0.0),
                    y=f.get('y', 0.0),
                    width=f.get('width', 0.0),
                    height=f.get('height', 0.0),
                    is_frontal=f.get('is_frontal', False),
                    yaw=f.get('yaw', 0.0),
                    pitch=f.get('pitch', 0.0),
                    roll=f.get('roll', 0.0),
                    confidence=f.get('confidence', 0.0),
                    face_id=f.get('face_id', -1),
                    timestamp=timestamp,  # 保留时间戳用于延时补偿
                )
                faces.append(face)
            
            self.fsm.data.perception.faces = faces
            self.fsm.data.perception.face_count = len(faces)
            
            # 记录接收到的face_id信息（用于调试face_id变化）
            if faces:
                face_ids = [f.face_id for f in faces]
                self.get_logger().debug(f"[FSMNode] 接收到 {len(faces)} 个人脸, IDs={face_ids}:")
                for i, face in enumerate(faces):
                    self.get_logger().debug(
                        f"  face[{i}]: id={face.face_id}, "
                        f"pos=({face.center_x:.4f}, {face.center_y:.4f}), "
                        f"frontal={face.is_frontal}, "
                        f"bbox=({face.x:.3f}, {face.y:.3f}, {face.width:.3f}, {face.height:.3f})"
                    )
            
            # 同步 face_detected 标志
            frontal_faces = [f for f in faces if f.is_frontal]
            if frontal_faces:
                self.fsm.data.perception.face_detected = True
                # 使用最大正脸的位置
                best = max(frontal_faces, key=lambda f: f.area)
                self.fsm.data.perception.face_position[0] = best.center_x
                self.fsm.data.perception.face_position[1] = best.center_y
            else:
                # 重要：如果没有正脸，必须设置为 False
                # 否则 face_detected 会保持旧值，导致 TRACKING 状态无法检测到目标丢失
                self.fsm.data.perception.face_detected = False
            
            # 打印更新后的感知数据
            perception = self.fsm.data.perception
            # self.get_logger().info(
            #     f"[感知数据更新] face_detected={perception.face_detected}, "
            #     f"face_count={perception.face_count}, "
            #     f"face_position=({perception.face_position[0]:.4f}, {perception.face_position[1]:.4f}), "
            #     f"has_target={perception.has_target}, "
            #     f"target_confirmed={perception.target_confirmed}, "
            #     f"target_position=({perception.target_position[0]:.4f}, {perception.target_position[1]:.4f}), "
            #     f"is_face_visible={perception.is_face_visible}, "
            #     f"face_visible_duration={perception.face_visible_duration:.2f}s, "
            #     f"faces_count={len(perception.faces)}"
            # )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse faces JSON: {e}")
    
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """处理相机信息"""
        self.fsm.data.camera.update_from_camera_info(msg)
        # 只在首次接收时记录日志
        # 定期日志 (每3秒)
        now = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_cam_log'): self._last_cam_log = 0.0
        if now - self._last_cam_log > 3.0:
            self.get_logger().info(
                f"Camera info received: fx={self.fsm.data.camera.fx:.1f}, "
                f"size={self.fsm.data.camera.width}x{self.fsm.data.camera.height}"
            )
            self._last_cam_log = now
            self._camera_info_logged = True
    
    def _on_image(self, msg: Image) -> None:
        """处理相机图像（用于录制等功能）"""
        if not CV_AVAILABLE:
            return
        
        try:
            # 获取图像尺寸
            height = msg.height
            width = msg.width
            encoding = msg.encoding
            
            # 根据编码格式转换为OpenCV格式（BGR）
            if encoding == 'rgb8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
                cv_image = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            elif encoding == 'bgr8':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            elif encoding == 'rgba8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
                cv_image = cv2.cvtColor(img_array, cv2.COLOR_RGBA2BGR)
            elif encoding == 'bgra8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
                cv_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)
            elif encoding == 'mono8':
                img_gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
                cv_image = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
            else:
                # 不支持的编码格式，跳过
                return
            
            # 确保图像是连续的
            cv_image = np.ascontiguousarray(cv_image)
            
            # 存储到FSM数据中
            self.fsm.data.perception.current_frame = cv_image
            
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def _on_set_look_at(self, msg: PointStamped) -> None:
        """处理外部设置的 look_at 值"""
        # 直接修改 FSM 数据中的 look_at_target
        # 这样 _publish_motion 会定时发布这个值
        self.fsm.data.motion.look_at_target[0] = float(msg.point.x)
        self.fsm.data.motion.look_at_target[1] = float(msg.point.y)
        self.get_logger().debug(
            f"External look_at set: x={msg.point.x:.3f}, y={msg.point.y:.3f}"
        )
    
    def _on_command(self, msg: String) -> None:
        """处理外部命令"""
        cmd = msg.data.lower().strip()
        self.get_logger().info(f"Received command: {cmd}")
        
        if cmd == "stop":
            self.fsm.send_command(WeddingEvent.CMD_STOP)
        elif cmd == "photo":
            self.fsm.send_command(WeddingEvent.CMD_GOTO_PHOTO)
        elif cmd == "idle":
            self.fsm.send_command(WeddingEvent.CMD_GOTO_IDLE)
        elif cmd == "search":
            self.fsm.send_command(WeddingEvent.CMD_GOTO_SEARCH)
        elif cmd == "tracking":
            self.fsm.send_command(WeddingEvent.CMD_GOTO_TRACKING)
        elif cmd == "interview":
            self.fsm.send_command(WeddingEvent.CMD_START_INTERVIEW)
        elif cmd == "reset":
            self.fsm.reset()
        else:
            self.get_logger().warning(f"Unknown command: {cmd}")
    
    # ========== 服务回调 ==========
    
    def _on_reset(self, request, response) -> Trigger.Response:
        """重置 FSM"""
        self.fsm.reset()
        response.success = True
        response.message = "FSM reset to IDLE"
        self.get_logger().info("FSM reset via service")
        return response
    
    def _on_stop(self, request, response) -> Trigger.Response:
        """停止 FSM"""
        self.fsm.send_command(WeddingEvent.CMD_STOP)
        response.success = True
        response.message = "FSM stop command sent"
        return response
    
    # ========== 主循环 ==========
    
    def _run_fsm(self) -> None:
        """FSM 主循环"""
        if not self._system_ready:
            # 等待 MotionAdapter 初始化完毕
            return

        # 运行 FSM
        self.fsm.run_once()
        
        # 动作管理器更新 (必须周期性调用)
        if self.fsm.action_manager:
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.fsm.action_manager.update(current_time)
        
        # 发布动作指令 (Legacy: 如果ActionManager已接管Pose发布，这里可能需要调整)
        # 目前保留 LookAt 发布，Pose 由 ActionManager 发布
        self._publish_motion()
        
        # 发布语音指令 (Legacy: FSM.data.audio.pending_speech)
        # ActionManager 也会独立发布语音，两者并行不冲突
        self._publish_speech()
        
        # 重置单帧感知数据
        self.fsm.data.perception.reset()
    
    def _publish_state(self) -> None:
        """发布 FSM 状态"""
        msg = String()
        msg.data = str(self.fsm.get_current_state_name())
        self.state_pub.publish(msg)
        
        # 发布目标 face_id（用于可视化）
        target_face_id = self.fsm.data.perception.target_face_id
        target_msg = String()
        if target_face_id is not None:
            target_msg.data = str(target_face_id)
        else:
            target_msg.data = "-1"  # -1 表示无目标
        self.target_face_id_pub.publish(target_msg)
    
    def _publish_motion(self) -> None:
        """发布动作指令"""
        motion = self.fsm.data.motion
        
        # 发布 Pose（仅当变化时）
        # 注意：现在 ActionManager 也会发布 /set_pose
        # 如果 FSM 通过 execute_action 设置了 Action，ActionManager 会负责发布 Pose
        # 如果 FSM 通过 set_pose() 设置了 target_pose，这里负责发布
        # 为了避免冲突，我们检查 target_pose 是否非空且变化
        # 并统一 Topic 名称为 /wedding/motion/set_pose (Adapter 监听的)
        if motion.target_pose != self._last_pose:
            msg = String()
            msg.data = motion.target_pose
            self.pose_pub.publish(msg)
            self._last_pose = motion.target_pose
            self.get_logger().debug(f"Published pose: {motion.target_pose}")
        
        # 发布 LookAt
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.point.x = float(motion.look_at_target[0])
        msg.point.y = float(motion.look_at_target[1])
        msg.point.z = 0.0
        self.look_at_pub.publish(msg)
    
    def _publish_speech(self) -> None:
        """发布语音指令"""
        audio = self.fsm.data.audio
        if audio.pending_speech:
            msg = String()
            msg.data = audio.pending_speech
            self.speech_pub.publish(msg)
            self.get_logger().info(f"Published speech: {audio.pending_speech}")
            audio.pending_speech = ""  # 清空，只发一次
    
    def _on_state_change(self, old_state: WeddingStateName, new_state: WeddingStateName) -> None:
        """状态变化回调"""
        self.get_logger().info(f"State changed: {old_state} -> {new_state}")
        
        # 立即发布状态变化
        msg = String()
        msg.data = str(new_state)
        self.state_pub.publish(msg)
    
    def destroy_node(self):
        """销毁节点"""
        self.fsm.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WeddingFSMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
