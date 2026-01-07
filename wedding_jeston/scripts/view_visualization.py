#!/usr/bin/env python3
"""
可视化图像查看工具

使用 OpenCV 显示感知可视化结果

Usage:
    python3 scripts/view_visualization.py
    # 或指定 topic
    python3 scripts/view_visualization.py --topic /wedding/perception/visualization
"""

import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

import numpy as np

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Error: cv2 not available. Please install: pip install opencv-python")


class VisualizationViewer(Node):
    """可视化图像查看器"""
    
    def __init__(self, topic: str):
        super().__init__('visualization_viewer')
        
        self.topic = topic
        
        # QoS
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅可视化图像
        self.image_sub = self.create_subscription(
            Image, self.topic, self._on_image, qos_sensor)
        
        self.get_logger().info(f"VisualizationViewer initialized")
        self.get_logger().info(f"  Subscribing to: {self.topic}")
        self.get_logger().info(f"  Press 'q' to quit")
    
    def _on_image(self, msg: Image):
        """处理可视化图像"""
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
            else:
                self.get_logger().warn(f"Unsupported encoding: {encoding}")
                return
            
            # 显示图像
            cv2.imshow('Perception Visualization', cv_image)
            cv2.waitKey(1)  # 非阻塞等待
            
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")


def main():
    parser = argparse.ArgumentParser(description='View perception visualization')
    parser.add_argument(
        '--topic',
        type=str,
        default='/wedding/perception/visualization',
        help='ROS2 topic to subscribe (default: /wedding/perception/visualization)'
    )
    
    args = parser.parse_args()
    
    if not CV_AVAILABLE:
        print("Error: OpenCV not available. Please install: pip install opencv-python")
        return
    
    rclpy.init()
    node = VisualizationViewer(args.topic)
    
    try:
        print("Waiting for visualization images...")
        print("Press 'q' in the image window to quit")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

