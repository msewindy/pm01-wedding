"""
感知可视化节点

在输入图像上标记人脸检测结果，并根据 FSM 状态使用不同颜色
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

import numpy as np
import json
import time
import threading
from collections import deque

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Warning: cv2 not available")

from ..perception import FaceInfo


class PerceptionVisualizerNode(Node):
    """
    感知可视化节点
    
    功能：
    1. 订阅相机图像
    2. 订阅人脸检测结果
    3. 订阅 FSM 状态
    4. 在图像上绘制检测框和状态信息
    5. 发布可视化后的图像
    
    Topics:
    - 订阅: /camera/head/rgb/image_raw (Image)
    - 订阅: /wedding/perception/faces_json (String)
    - 订阅: /wedding/fsm/state (String)
    - 发布: /wedding/perception/visualization (Image)
    """
    
    # 颜色定义 (BGR)
    COLOR_IDLE_ALL = (255, 255, 255)          # 白色 - IDLE 状态未锁定前显示所有人脸
    COLOR_IDLE_CANDIDATE = (0, 255, 255)      # 黄色 - IDLE 状态锁定后的候选
    COLOR_SEARCH_CONFIRM = (0, 165, 255)      # 橙色 - SEARCH 状态确认的正脸
    COLOR_TRACKING_TARGET = (0, 255, 0)       # 绿色 - TRACKING 状态跟随的目标
    COLOR_SIDE_FACE = (128, 128, 128)         # 灰色 - 侧脸
    COLOR_TEXT_BG = (0, 0, 0)                 # 黑色背景
    COLOR_TEXT = (255, 255, 255)              # 白色文字
    
    def __init__(self):
        super().__init__('perception_visualizer_node')
        
        # 参数
        self.declare_parameter('image_topic', '/camera/head/rgb/image_raw')
        self.declare_parameter('output_topic', '/wedding/perception/visualization')
        self.declare_parameter('publish_rate', 10.0)
        
        self._image_topic = self.get_parameter('image_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._publish_rate = self.get_parameter('publish_rate').value
        
        # 数据缓存
        self._image_buffer = deque(maxlen=30)  # 存储最近30帧 (约1秒)
        self._latest_state = "UNKNOWN"
        self._latest_face_detected = False
        self._target_face_id = None  # 当前跟踪的目标 face_id
        self._data_lock = threading.Lock()
        
        # QoS
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅器
        self.image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, qos_sensor)
        
        self.faces_sub = self.create_subscription(
            String, '/wedding/perception/faces_json', self._on_faces, qos_sensor)
        
        self.state_sub = self.create_subscription(
            String, '/wedding/fsm/state', self._on_state, qos_reliable)
        
        self.face_detected_sub = self.create_subscription(
            Bool, '/wedding/perception/face_detected', self._on_face_detected, qos_reliable)
        
        self.target_face_id_sub = self.create_subscription(
            String, '/wedding/fsm/target_face_id', self._on_target_face_id, qos_reliable)
        
        # 发布器
        self.viz_pub = self.create_publisher(
            Image, self._output_topic, qos_sensor)
        
        self.get_logger().info(f"PerceptionVisualizerNode initialized (Sync Mode)")
        self.get_logger().info(f"  Image topic: {self._image_topic}")
        self.get_logger().info(f"  Output topic: {self._output_topic}")

        # 调试统计
        self._img_recv_count = 0
        self._json_recv_count = 0
        self._pub_count = 0
        self._last_log_time = time.time()
        self.create_timer(2.0, self._print_debug_status)

    def _print_debug_status(self):
        """定期打印调试状态"""
        now = time.time()
        if now - self._last_log_time > 5.0:
            with self._data_lock:
                 buf_size = len(self._image_buffer)
                 latest_ts = self._image_buffer[-1][1] if self._image_buffer else 0.0
            
            self.get_logger().info(
                f"[VizStatus] ImgRecv: {self._img_recv_count}, JSONRecv: {self._json_recv_count}, "
                f"Pub: {self._pub_count}, BufSize: {buf_size}, LastImgTS: {latest_ts:.2f}"
            )
            self._last_log_time = now
    
    def _on_image(self, msg: Image):
        """处理相机图像"""
        self._img_recv_count += 1
        if not CV_AVAILABLE:
            return
        
        try:
            height = msg.height
            width = msg.width
            encoding = msg.encoding
            
            # 根据编码格式转换
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
                return
            
            cv_image = np.ascontiguousarray(cv_image)
            
            # 获取时间戳
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # [Fix for Sim] 如果 timestamp 为 0 (仿真环境可能未设置时间)，使用当前时间
            # 这样才能与 PerceptionNode (同样fallback到当前时间) 进行匹配
            if timestamp == 0.0:
                timestamp = time.time()
            
            with self._data_lock:
                self._image_buffer.append((cv_image, timestamp))
                
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def _on_faces(self, msg: String):
        """处理人脸检测结果"""
        self._json_recv_count += 1
        try:
            data = json.loads(msg.data)
            faces_data = data.get('faces', [])
            
            try:
                faces = []
                for f in faces_data:
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
                    )
                    faces.append(face)
            except Exception as e:
                self.get_logger().error(f"Error creating FaceInfo objects: {e}")
                return
            
            # 获取结果时间戳
            faces_timestamp = data.get('timestamp', 0)
            
            matched_image = None
            
            with self._data_lock:
                # 检查 buffer 状态
                if not self._image_buffer:
                    if self._json_recv_count % 30 == 0:
                        self.get_logger().warn("[Viz] Image buffer is empty! Cannot visualize.")
                    return

                # 在 buffer 中查找最接近的时间戳
                best_diff = 0.1  # 最大允许误差 100ms
                
                # 倒序查找（优先匹配最新的）
                for img, ts in reversed(self._image_buffer):
                    diff = abs(ts - faces_timestamp)
                    if diff < best_diff:
                        best_diff = diff
                        matched_image = img
                        # 找到最佳匹配后可以停止吗？
                        # 因为是倒序，越早遍历到的越新。
                        # 如果 faces_timestamp 是旧的，我们应该找旧图。
                        # 如果 buffer 是 [t1, t2, t3, t4]，faces 是 t3。
                        # 倒序遍历：t4(diff大), t3(diff小), t2(diff大)...
                        # 我们可以遍历整个 buffer 找最小值。
                
                # 如果没有精确的时间戳（旧版 perception_node），或者没匹配到
                if matched_image is None and faces_timestamp <= 0 and self._image_buffer:
                     # 使用最新的一帧
                     matched_image = self._image_buffer[-1][0]
                
                # 如果仍未匹配到且 buffer 非空，但误差都太大？
                # 这种情况下，为了避免无画面，可以使用最新帧，虽然会滞后。
                if matched_image is None and self._image_buffer:
                    matched_image = self._image_buffer[-1][0]

            if matched_image is not None:
                self._publish_visualization(matched_image, faces)
            else:
                 self.get_logger().warn("[Viz] Failed to match image for visualization")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse faces JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in _on_faces: {e}")
    
    def _on_state(self, msg: String):
        """处理 FSM 状态"""
        with self._data_lock:
            self._latest_state = msg.data
    
    def _on_face_detected(self, msg: Bool):
        """处理 face_detected 标志"""
        with self._data_lock:
            self._latest_face_detected = msg.data
    
    def _on_target_face_id(self, msg: String):
        """处理目标 face_id"""
        try:
            face_id = int(msg.data)
            with self._data_lock:
                self._target_face_id = face_id if face_id >= 0 else None
        except ValueError:
            with self._data_lock:
                self._target_face_id = None
    
    def _publish_visualization(self, image, faces):
        """发布可视化图像 (事件驱动)"""
        if not CV_AVAILABLE:
            return
        
        self._pub_count += 1
        
        with self._data_lock:
            # 状态信息仍然是最新的
            state = self._latest_state
            face_detected = self._latest_face_detected
            target_face_id = self._target_face_id
        
        if image is None:
            return
        
        # 复制图像用于绘制
        viz_image = image.copy()
        h, w = viz_image.shape[:2]
        
        # 调试日志（每30帧记录一次，避免日志过多）
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 30 == 0:
            self.get_logger().info(
                f"[可视化调试] state='{state}', target_face_id={target_face_id}, "
                f"faces_count={len(faces)}, face_ids={[f.face_id for f in faces]}"
            )
        
        # 根据状态决定显示哪些人脸
        faces_to_draw = []
        
        # 状态字符串可能是 "IDLE", "SEARCH", "TRACKING" 等
        # 使用 upper() 确保大小写不敏感匹配
        state_upper = state.upper() if state else ""
        
        if "IDLE" in state_upper:
            # IDLE状态：如果未锁定（target_face_id为None），显示所有人脸（白色框）
            # 如果已锁定（target_face_id不为None），只显示锁定的face（黄色框）
            if target_face_id is None:
                # 未锁定：显示所有人脸（白色框）
                faces_to_draw = faces
            else:
                # 已锁定：只显示锁定的face（黄色框）
                for face in faces:
                    if face.face_id == target_face_id:
                        faces_to_draw = [face]
                        break
        elif "SEARCH" in state_upper:
            # SEARCH状态：优先显示正在确认追踪的face（橙色框）
            # 如果找不到匹配的face_id，则显示所有人脸（用于调试）
            if target_face_id is not None:
                for face in faces:
                    if face.face_id == target_face_id:
                        faces_to_draw = [face]
                        break
                # 如果没找到匹配的face_id，显示所有人脸（用于调试不匹配问题）
                if not faces_to_draw:
                    faces_to_draw = faces
                    # 输出警告日志（每30帧一次，避免日志过多）
                    if self._debug_counter % 30 == 0:
                        self.get_logger().warn(
                            f"[SEARCH状态] target_face_id={target_face_id} 不匹配，"
                            f"检测到的face_ids={[f.face_id for f in faces]}，显示所有人脸"
                        )
        elif "TRACKING" in state_upper or "INTERVIEW" in state_upper:
            # TRACKING/INTERVIEW状态：优先显示正在追踪的face（绿色框）
            # 如果找不到匹配的face_id，则显示所有人脸（用于调试）
            if target_face_id is not None:
                for face in faces:
                    if face.face_id == target_face_id:
                        faces_to_draw = [face]
                        break
                # 如果没找到匹配的face_id，显示所有人脸（用于调试不匹配问题）
                if not faces_to_draw:
                    faces_to_draw = faces
                    # 输出警告日志（每30帧一次，避免日志过多）
                    if self._debug_counter % 30 == 0:
                        self.get_logger().warn(
                            f"[{state}状态] target_face_id={target_face_id} 不匹配，"
                            f"检测到的face_ids={[f.face_id for f in faces]}，显示所有人脸"
                        )
        else:
            # 其他状态：显示所有人脸（默认灰色框）
            faces_to_draw = faces
        
        # 绘制人脸框
        for face in faces_to_draw:
            x1 = int(face.x * w)
            y1 = int(face.y * h)
            x2 = int((face.x + face.width) * w)
            y2 = int((face.y + face.height) * h)
            
            # 检查当前face是否匹配target_face_id
            is_target = (target_face_id is not None and face.face_id == target_face_id)
            
            # 根据状态和是否匹配选择颜色
            state_upper = state.upper()
            if "TRACKING" in state_upper or "INTERVIEW" in state_upper:
                if is_target:
                    color = self.COLOR_TRACKING_TARGET
                    # 如果是 INTERVIEW 状态显示 INTERVIEW，否则显示 TRACKING
                    label = "INTERVIEW" if "INTERVIEW" in state_upper else "TRACKING"
                else:
                    # 不匹配的face用灰色显示
                    color = self.COLOR_SIDE_FACE
                    label = "OTHER"
            elif "SEARCH" in state_upper:
                if is_target:
                    color = self.COLOR_SEARCH_CONFIRM
                    label = "SEARCH"
                else:
                    # 不匹配的face用灰色显示
                    color = self.COLOR_SIDE_FACE
                    label = "OTHER"
            elif "IDLE" in state_upper:
                if target_face_id is None:
                    # IDLE未锁定：白色框
                    color = self.COLOR_IDLE_ALL
                    label = "IDLE"
                else:
                    # IDLE已锁定：匹配的用黄色框，不匹配的用灰色
                    if is_target:
                        color = self.COLOR_IDLE_CANDIDATE
                        label = "IDLE"
                    else:
                        color = self.COLOR_SIDE_FACE
                        label = "OTHER"
            else:
                color = self.COLOR_SIDE_FACE
                label = "OTHER"
            
            # 绘制矩形框
            thickness = 3 if face.is_frontal else 2
            cv2.rectangle(viz_image, (x1, y1), (x2, y2), color, thickness)
            
            # 绘制标签
            label_text = f"{label}"
            if face.is_frontal:
                label_text += f" Yaw:{face.yaw:.0f}°"
            if target_face_id is not None and face.face_id == target_face_id:
                label_text += f" ID:{face.face_id}"
            
            # 标签背景
            (text_w, text_h), baseline = cv2.getTextSize(
                label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(viz_image, 
                         (x1, y1 - text_h - baseline - 5),
                         (x1 + text_w, y1),
                         color, -1)
            
            # 标签文字
            cv2.putText(viz_image, label_text,
                       (x1, y1 - baseline - 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                       (0, 0, 0), 1)
        
        # 绘制状态信息（左上角）
        self._draw_status_info(viz_image, state, faces, face_detected)
        
        # 转换为 ROS Image 并发布
        try:
            viz_msg = self._cv2_to_ros_image(viz_image)
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish visualization: {e}")
    
    def _draw_status_info(self, image, state, faces, face_detected):
        """在图像左上角绘制状态信息"""
        h, w = image.shape[:2]
        
        # 状态文本
        state_text = f"State: {state}"
        
        # 统计信息
        frontal_count = sum(1 for f in faces if f.is_frontal)
        total_count = len(faces)
        stats_text = f"Faces: {total_count} (Frontal: {frontal_count})"
        detected_text = f"Detected: {'YES' if face_detected else 'NO'}"
        
        # 添加 target_face_id 信息（用于调试）
        with self._data_lock:
            target_id = self._target_face_id
        target_text = f"Target ID: {target_id if target_id is not None else 'None'}"
        
        # 计算文本大小
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        line_height = 25
        
        texts = [state_text, stats_text, detected_text, target_text]
        max_width = 0
        
        for text in texts:
            (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
            max_width = max(max_width, text_w)
        
        # 绘制背景
        bg_height = len(texts) * line_height + 10
        cv2.rectangle(image, (5, 5), (max_width + 20, bg_height),
                     self.COLOR_TEXT_BG, -1)
        
        # 绘制文字
        y_offset = 25
        for i, text in enumerate(texts):
            cv2.putText(image, text, (10, y_offset + i * line_height),
                       font, font_scale, self.COLOR_TEXT, thickness)
    
    def _cv2_to_ros_image(self, cv_image):
        """将 OpenCV 图像转换为 ROS Image 消息"""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = cv_image.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

