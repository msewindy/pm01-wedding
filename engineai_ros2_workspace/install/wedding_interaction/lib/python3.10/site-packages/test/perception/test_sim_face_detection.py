#!/usr/bin/env python3
"""
测试仿真相机人脸检测

订阅 Mujoco 仿真相机图像，检测人脸并显示结果
（不依赖 cv_bridge，避免 NumPy 版本冲突）

Usage:
    # 先启动仿真器
    ros2 launch mujoco_simulator mujoco_simulator.launch.py
    
    # 然后运行此脚本
    cd /home/lingjing/project/engine_ai/wedding_jeston
    python3 test/perception/test_sim_face_detection.py
"""

import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

import numpy as np
import time

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Error: OpenCV not available")
    print("Install: pip install opencv-python")
    sys.exit(1)

from wedding_interaction.perception import FaceDetector


def ros_image_to_cv2(msg: Image) -> np.ndarray:
    """
    将 ROS Image 消息转换为 OpenCV 图像（不使用 cv_bridge）
    """
    # 获取图像尺寸
    height = msg.height
    width = msg.width
    encoding = msg.encoding
    
    # 将 data 转换为 numpy 数组
    if encoding == 'rgb8':
        # RGB 格式
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == 'bgr8':
        # BGR 格式
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
    elif encoding == 'mono8':
        # 灰度
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        # 尝试通用处理
        channels = len(msg.data) // (height * width)
        if channels == 3:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif channels == 4:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        else:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    return img


class SimFaceDetectionTest(Node):
    def __init__(self):
        super().__init__('sim_face_detection_test')
        
        self.face_detector = FaceDetector(use_face_mesh=True)
        
        # 统计
        self.frame_count = 0
        self.face_count = 0
        self.last_print_time = time.time()
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅仿真相机
        self.image_sub = self.create_subscription(
            Image,
            '/camera/head/rgb/image_raw',
            self.image_callback,
            qos
        )
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("仿真相机人脸检测测试")
        self.get_logger().info("=" * 50)
        self.get_logger().info("订阅: /camera/head/rgb/image_raw")
        self.get_logger().info("按 Ctrl+C 退出，按 'q' 关闭窗口")
        self.get_logger().info("")
        
    def image_callback(self, msg: Image):
        try:
            # 转换图像（不使用 cv_bridge）
            cv_image = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
        
        self.frame_count += 1
        
        # 人脸检测
        faces = self.face_detector.detect(cv_image)
        
        # 绘制结果
        for face in faces:
            h, w = cv_image.shape[:2]
            x1 = int(face.x * w)
            y1 = int(face.y * h)
            x2 = int((face.x + face.width) * w)
            y2 = int((face.y + face.height) * h)
            
            # 颜色：正脸绿色，侧脸橙色
            color = (0, 255, 0) if face.is_frontal else (0, 165, 255)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
            
            # 标签
            label = f"{'FRONTAL' if face.is_frontal else 'SIDE'} yaw={face.yaw:.0f}"
            cv2.putText(cv_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 统计信息
        if faces:
            self.face_count += 1
        
        # 每秒打印一次统计
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            frontal_faces = [f for f in faces if f.is_frontal]
            self.get_logger().info(
                f"帧: {self.frame_count} | "
                f"检测到人脸帧: {self.face_count} | "
                f"当前帧人脸: {len(faces)} (正脸: {len(frontal_faces)})"
            )
            self.last_print_time = current_time
        
        # 显示状态
        status = f"Faces: {len(faces)} | Frontal: {len([f for f in faces if f.is_frontal])}"
        cv2.putText(cv_image, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        if not faces:
            cv2.putText(cv_image, "No face detected", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 显示图像
        cv2.imshow("Sim Camera Face Detection", cv_image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("退出...")
            rclpy.shutdown()
    
    def destroy_node(self):
        self.face_detector.release()
        cv2.destroyAllWindows()
        
        # 打印总结
        print()
        print("=" * 50)
        print("测试总结")
        print("=" * 50)
        print(f"总帧数: {self.frame_count}")
        print(f"检测到人脸的帧数: {self.face_count}")
        if self.frame_count > 0:
            rate = self.face_count / self.frame_count * 100
            print(f"检测率: {rate:.1f}%")
            if rate < 10:
                print()
                print("⚠️ 检测率很低！MediaPipe 可能无法检测仿真中的 3D 人体模型。")
                print("建议使用 mock 数据进行状态机测试。")
        print()
        
        super().destroy_node()


def main():
    rclpy.init()
    node = SimFaceDetectionTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
