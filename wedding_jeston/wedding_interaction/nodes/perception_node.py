"""
感知节点（简化版）

订阅相机图像，发布人脸检测结果
支持 USB 摄像头和 ROS2 Image Topic
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import numpy as np
import json
import time
import threading

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Warning: cv2 not available")

from ..perception import FaceDetector, FaceInfo


class PerceptionNode(Node):
    """
    感知节点（简化版 - 仅人脸检测）
    
    功能：
    1. 订阅相机图像（ROS2 Topic 或 USB 摄像头）
    2. 进行人脸检测 + 正脸判断
    3. 发布检测结果
    
    Topics:
    - 订阅: /camera/color/image_raw (Image)
    - 发布: /wedding/perception/faces_json (String) - 所有人脸 JSON
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
        # 参数
        self.declare_parameter('use_usb_camera', False)
        self.declare_parameter('usb_camera_id', 0)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('detection_rate', 30.0)
        self.declare_parameter('use_face_mesh', True)
        self.declare_parameter('model_selection', 0)  # 0: 近距离, 1: 远距离
        self.declare_parameter('force_opencv', False)  # False: 使用 MediaPipe (如果可用), True: 强制使用 OpenCV
        
        # Face Detector Thresholds
        self.declare_parameter('face_frontal_yaw_threshold', 30.0)
        self.declare_parameter('face_frontal_pitch_threshold', 20.0)
        self.declare_parameter('face_min_detection_confidence', 0.5)
        self.declare_parameter('face_min_tracking_confidence', 0.5)
        self.declare_parameter('face_min_size', 0.05) # 最小人脸尺寸 (比例 0-1)
        
        # Face Tracker
        self.declare_parameter('face_tracker_iou_threshold', 0.15)
        self.declare_parameter('face_tracker_distance_threshold', 0.30)
        self.declare_parameter('face_tracker_max_time_since_seen', 5.0)
        
        self._use_usb_camera = self.get_parameter('use_usb_camera').value
        self._usb_camera_id = self.get_parameter('usb_camera_id').value
        self._image_topic = self.get_parameter('image_topic').value
        self._detection_rate = self.get_parameter('detection_rate').value
        self._use_face_mesh = self.get_parameter('use_face_mesh').value
        self._model_selection = self.get_parameter('model_selection').value
        self._force_opencv = self.get_parameter('force_opencv').value
        
        # Thresholds
        face_frontal_yaw_threshold = self.get_parameter('face_frontal_yaw_threshold').value
        face_frontal_pitch_threshold = self.get_parameter('face_frontal_pitch_threshold').value
        face_min_detection_confidence = self.get_parameter('face_min_detection_confidence').value
        face_min_tracking_confidence = self.get_parameter('face_min_tracking_confidence').value
        face_min_size = self.get_parameter('face_min_size').value
        
        face_tracker_iou_threshold = self.get_parameter('face_tracker_iou_threshold').value
        face_tracker_distance_threshold = self.get_parameter('face_tracker_distance_threshold').value
        face_tracker_max_time_since_seen = self.get_parameter('face_tracker_max_time_since_seen').value
        
        # 人脸检测器
        self._face_detector = FaceDetector(
            use_face_mesh=self._use_face_mesh,
            min_detection_confidence=face_min_detection_confidence,
            min_tracking_confidence=face_min_tracking_confidence,
            frontal_yaw_threshold=face_frontal_yaw_threshold,
            frontal_pitch_threshold=face_frontal_pitch_threshold,
            tracker_iou_threshold=face_tracker_iou_threshold,
            tracker_distance_threshold=face_tracker_distance_threshold,
            tracker_max_time_since_seen=face_tracker_max_time_since_seen,
            model_selection=self._model_selection,
            force_opencv=self._force_opencv,
            min_face_size=face_min_size,
            logger=self.get_logger() # 传递 ROS logger
        )
        
        # 无需 cv_bridge，直接处理 ROS Image
        
        # 最新图像
        self._latest_image = None
        self._image_lock = threading.Lock()
        
        # USB 摄像头
        self._cap = None
        self._camera_thread = None
        
        # QoS
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅器
        if not self._use_usb_camera:
            self._image_sub = self.create_subscription(
                Image,
                self._image_topic,
                self._image_callback,
                qos_sensor
            )
            self.get_logger().info(f"Subscribing to {self._image_topic}")
        else:
            self._image_sub = None
            self._start_usb_camera()
        
        # 发布器
        self._faces_pub = self.create_publisher(
            String, '/wedding/perception/faces_json', qos_reliable)
        
        # 检测定时器
        timer_period = 1.0 / self._detection_rate
        self._detection_timer = self.create_timer(timer_period, self._detection_loop)
        
        # 统计
        self._frame_count = 0
        self._start_time = time.time()
        self._last_processed_ts = -1.0  # 上一次处理的帧时间戳
        
        self.get_logger().info(f"PerceptionNode initialized "
                              f"(use_usb={self._use_usb_camera}, rate={self._detection_rate}Hz)")
    
    def _start_usb_camera(self) -> None:
        """启动 USB 摄像头采集线程"""
        if not CV_AVAILABLE:
            self.get_logger().error("OpenCV not available, cannot use USB camera")
            return
        
        self._cap = cv2.VideoCapture(self._usb_camera_id)
        if not self._cap.isOpened():
            self.get_logger().error(f"Cannot open USB camera {self._usb_camera_id}")
            return
        
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        
        self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._camera_thread.start()
        
        self.get_logger().info(f"USB camera {self._usb_camera_id} started")
    
    def _camera_loop(self) -> None:
        """USB 摄像头采集循环"""
        while rclpy.ok():
            if self._cap is None:
                break
            ret, frame = self._cap.read()
            if ret:
                timestamp = time.time()
                with self._image_lock:
                    # 统一使用 tuple (image, timestamp) 格式
                    self._latest_image = (frame, timestamp)
            time.sleep(0.01)  # 100 Hz
    
    def _image_callback(self, msg: Image) -> None:
        """ROS2 图像回调 - 直接处理 ROS Image 数据，不使用 cv_bridge"""
        try:
            # 获取图像尺寸
            height = msg.height
            width = msg.width
            encoding = msg.encoding
            
            # 根据编码格式转换
            if encoding == 'rgb8':
                # RGB -> BGR
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
                self.get_logger().warn(f"Unsupported encoding: {encoding}")
                return
            
            # 确保图像是连续的
            cv_image = np.ascontiguousarray(cv_image)
            
            with self._image_lock:
                # 存储图像和 Header (包含时间戳)
                self._latest_image = (cv_image, msg.header)
                
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def _detection_loop(self) -> None:
        """检测主循环"""
        # 获取图像
        with self._image_lock:
            image_data = self._latest_image
        
        if image_data is None:
            return
            
        # 解析图像和时间戳
        image = None
        timestamp = 0.0
        
        if isinstance(image_data, tuple):
            img_obj, ts_obj = image_data
            image = img_obj
            if hasattr(ts_obj, 'stamp'): # ROS Header
                timestamp = ts_obj.stamp.sec + ts_obj.stamp.nanosec * 1e-9
            else: # Float timestamp
                timestamp = float(ts_obj)
            
            # [Fix for Sim] 如果 timestamp 为 0 (仿真环境可能未设置时间)，使用当前时间
            if timestamp == 0.0:
                timestamp = time.time()
        else:
            # 兼容旧代码（虽然应该不会执行到这里）
            image = image_data
            timestamp = time.time()
            
        # 检查是否是新帧 (容差 1ms)
        if timestamp <= self._last_processed_ts + 0.001:
            # 这一帧已经处理过了，跳过
            return
            
        self._last_processed_ts = timestamp

        
        if image is None:
            return
        
        # 人脸检测
        faces = self._face_detector.detect(image)
        
        # 打印检测结果（每帧都打印，用于调试face_id变化）
        if faces:
            face_ids = [f.face_id for f in faces]
            self.get_logger().info(f"[PerceptionNode] 检测到 {len(faces)} 个人脸, IDs={face_ids}:")
            for i, face in enumerate(faces):
                self.get_logger().info(
                    f"  face[{i}]: id={face.face_id}, "
                    f"pos=({face.center_x:.4f}, {face.center_y:.4f}), "
                    f"frontal={face.is_frontal}, "
                    f"area={face.area:.4f}, "
                    f"confidence={face.confidence:.3f}, "
                    f"bbox=({face.x:.3f}, {face.y:.3f}, {face.width:.3f}, {face.height:.3f})"
                )
        else:
            self.get_logger().debug("[PerceptionNode] 未检测到人脸")
        
        # 发布结果
        self._publish_results(faces, timestamp)
        
        # 统计
        self._frame_count += 1
        if self._frame_count % 100 == 0:
            elapsed = time.time() - self._start_time
            fps = self._frame_count / elapsed
            # 获取追踪统计
            stats = self._face_detector.get_tracking_stats()
            self.get_logger().info(f"[PerceptionNode] FPS: {fps:.1f}, Faces: {len(faces)}, "
                                 f"追踪统计: {stats}")
    
    def _publish_results(self, faces: list, timestamp: float) -> None:
        """发布检测结果"""
        # 发布 faces_json
        faces_data = [self._face_to_dict(f, timestamp) for f in faces]
        faces_msg = String()
        faces_msg.data = json.dumps({
            'faces': faces_data, 
            'count': len(faces),
            'timestamp': timestamp
        })
        self._faces_pub.publish(faces_msg)
    
    def _face_to_dict(self, face: FaceInfo, timestamp: float) -> dict:
        """将 FaceInfo 转换为字典"""
        # 使用传入的准确时间戳
        ts = timestamp
        
        return {
            'x': face.x,
            'y': face.y,
            'width': face.width,
            'height': face.height,
            'area': face.area,
            'center_x': face.center_x,
            'center_y': face.center_y,
            'is_frontal': face.is_frontal,
            'yaw': face.yaw,
            'pitch': face.pitch,
            'roll': face.roll,
            'confidence': face.confidence,
            'face_id': face.face_id,
            'timestamp': ts,  # 使用准确的时间戳
        }
    
    def destroy_node(self) -> None:
        """清理资源"""
        self._face_detector.release()
        
        if self._cap is not None:
            self._cap.release()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
