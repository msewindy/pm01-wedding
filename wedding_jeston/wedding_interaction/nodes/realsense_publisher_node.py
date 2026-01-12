import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSensePublisherNode(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        # Declare parameters
        self.declare_parameter('topic_name', '/camera/color/image_raw')
        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        # Get parameters
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value

        # Initialize Publisher
        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize RealSense Pipeline
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Enable Color Stream
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            
            # Start Pipeline
            self.pipeline.start(self.config)
            self.get_logger().info(f'RealSense Pipeline started. Publishing to {self.topic_name} at {self.fps}Hz')
            self.get_logger().info(f'Resolution: {self.width}x{self.height}')
            
            # Create Timer
            timer_period = 1.0 / self.fps
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RealSense: {str(e)}')
            raise e

    def timer_callback(self):
        try:
            # Wait for a coherent pair of frames: depth and color
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                return

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_frame"
            
            # Publish
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RealSensePublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
