import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSensePublisherNode(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        # Declare parameters
        self.declare_parameter('topic_name', '/camera/head/rgb/image_raw')
        self.declare_parameter('fps', 30)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        # 明确指定 serial_number 为字符串类型
        self.declare_parameter(
            'serial_number', 
            '', 
            ParameterDescriptor(
                description='RealSense 设备序列号（优先使用）',
                dynamic_typing=True
            )
        )
        self.declare_parameter('device_index', -1)   # 设备索引（如果 serial_number 未提供）

        # Get parameters
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        
        # Handle serial_number - can be string or integer (convert to string)
        serial_param = self.get_parameter('serial_number')
        if serial_param.type_ == ParameterType.PARAMETER_STRING:
            self.serial_number = serial_param.get_parameter_value().string_value
        elif serial_param.type_ == ParameterType.PARAMETER_INTEGER:
            # If passed as integer, convert to string
            self.serial_number = str(serial_param.get_parameter_value().integer_value)
        else:
            self.serial_number = ''
        
        self.device_index = self.get_parameter('device_index').get_parameter_value().integer_value

        # Initialize Publishers
        self.image_publisher_ = self.create_publisher(Image, self.topic_name, 10)
        
        # CameraInfo topic name (standard ROS2 convention: replace /image_raw with /camera_info)
        # e.g., /camera/color/image_raw -> /camera/color/camera_info
        camera_info_topic = "/camera/head/camera_info"
        self.camera_info_publisher_ = self.create_publisher(CameraInfo, camera_info_topic, 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be set from RealSense)
        self.camera_info = None
        
        # Initialize RealSense Pipeline
        try:
            # List all available devices
            ctx = rs.context()
            devices = ctx.query_devices()
            device_count = len(devices)
            
            if device_count == 0:
                raise RuntimeError("未找到 RealSense 设备")
            
            self.get_logger().info(f"发现 {device_count} 个 RealSense 设备:")
            for i, dev in enumerate(devices):
                serial = dev.get_info(rs.camera_info.serial_number)
                name = dev.get_info(rs.camera_info.name)
                self.get_logger().info(f"  设备 {i}: {name} (序列号: {serial})")
            
            # Select device
            selected_device = None
            if self.serial_number:
                # Select by serial number
                for dev in devices:
                    if dev.get_info(rs.camera_info.serial_number) == self.serial_number:
                        selected_device = dev
                        break
                if selected_device is None:
                    raise RuntimeError(f"未找到序列号为 {self.serial_number} 的设备")
                self.get_logger().info(f"使用序列号指定的设备: {self.serial_number}")
            elif self.device_index >= 0:
                # Select by index
                if self.device_index >= device_count:
                    raise RuntimeError(f"设备索引 {self.device_index} 超出范围（共 {device_count} 个设备）")
                selected_device = devices[self.device_index]
                serial = selected_device.get_info(rs.camera_info.serial_number)
                self.get_logger().info(f"使用索引指定的设备 {self.device_index} (序列号: {serial})")
            else:
                # Use first available device (default behavior)
                selected_device = devices[0]
                serial = selected_device.get_info(rs.camera_info.serial_number)
                self.get_logger().info(f"使用默认设备（第一个可用设备，序列号: {serial}）")
            
            # Enable device by serial number
            if selected_device:
                try:
                    self.get_logger().info(f"正在对设备 {serial} 执行硬件复位以确保状态正常...")
                    selected_device.hardware_reset()
                    self.get_logger().info("硬件复位已触发，等待 5 秒让设备重新连接...")
                    import time
                    time.sleep(5)
                except Exception as e:
                    self.get_logger().warn(f"硬件复位失败 (可能不需要): {e}")

            # Re-initialize context and pipeline after reset
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Enable device by serial number (Re-enable after reset)
            self.config.enable_device(serial)
            
            # Enable Color Stream
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            
            # Start Pipeline
            self.profile = self.pipeline.start(self.config)
            
            # Get device info for logging
            active_device = self.profile.get_device()
            active_serial = active_device.get_info(rs.camera_info.serial_number)
            active_name = active_device.get_info(rs.camera_info.name)
            self.get_logger().info(f"已连接到设备: {active_name} (序列号: {active_serial})")
            
            # Get camera intrinsics from the color stream
            color_stream = self.profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            # Create CameraInfo message
            self.camera_info = CameraInfo()
            self.camera_info.header.frame_id = "camera_color_frame"
            self.camera_info.width = self.width
            self.camera_info.height = self.height
            self.camera_info.distortion_model = "plumb_bob"
            
            # Intrinsic matrix K (3x3, row-major)
            # K = [fx  0  cx]
            #     [0  fy  cy]
            #     [0   0   1]
            self.camera_info.k = [
                intrinsics.fx, 0.0, intrinsics.ppx,
                0.0, intrinsics.fy, intrinsics.ppy,
                0.0, 0.0, 1.0
            ]
            
            # Distortion coefficients D (from RealSense)
            self.camera_info.d = list(intrinsics.coeffs[:5])  # RealSense provides 5 coefficients
            
            # Rectification matrix R (identity)
            self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            # Projection matrix P (3x4, row-major)
            # P = [fx  0  cx  0]
            #     [0  fy  cy  0]
            #     [0   0   1   0]
            self.camera_info.p = [
                intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
                0.0, intrinsics.fy, intrinsics.ppy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            self.get_logger().info(f'RealSense Pipeline started. Publishing to {self.topic_name} at {self.fps}Hz')
            self.get_logger().info(f'Resolution: {self.width}x{self.height}')
            self.get_logger().info(f'Camera intrinsics: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}, '
                                  f'cx={intrinsics.ppx:.1f}, cy={intrinsics.ppy:.1f}')
            self.get_logger().info(f'Publishing CameraInfo to: {camera_info_topic}')
            
            # Create Timer
            timer_period = 1.0 / self.fps
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RealSense: {str(e)}')
            raise e

    def reset_camera(self):
        """Reset the camera and restart the pipeline"""
        self.get_logger().warn("检测到相机故障，正在尝试自动恢复...")
        
        # Stop existing pipeline
        try:
            self.pipeline.stop()
        except:
            pass
            
        # Hardware reset
        try:
            if self.profile:
                dev = self.profile.get_device()
                dev.hardware_reset()
                self.get_logger().info("已触发硬件复位，等待 5 秒...")
                import time
                time.sleep(5)
        except Exception as e:
            self.get_logger().warn(f"硬件复位失败: {e}")
            
        # Restart pipeline
        try:
            self.get_logger().info("正在重新启动 Pipeline...")
            # Re-create pipeline and config to be safe
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Re-enable device and stream
            # Note: We need to store the serial number to re-enable the correct device
            if hasattr(self, 'serial_number') and self.serial_number:
                self.config.enable_device(self.serial_number)
            
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info("相机已成功恢复！")
        except Exception as e:
            self.get_logger().error(f"相机恢复失败: {e}")

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except:
            pass
        super().destroy_node()

    def timer_callback(self):
        try:
            # Wait for a coherent pair of frames: depth and color
            # Reduced timeout to 1000ms to detect failure faster (default is 5000ms)
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                return

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Get current timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            msg.header.stamp = timestamp
            msg.header.frame_id = "camera_color_frame"
            
            # Publish image
            self.image_publisher_.publish(msg)
            
            # Publish camera info (with same timestamp)
            if self.camera_info is not None:
                self.camera_info.header.stamp = timestamp
                self.camera_info_publisher_.publish(self.camera_info)
            
        except RuntimeError as e:
            # RealSense throws RuntimeError for timeout
            if "Frame didn't arrive" in str(e):
                self.get_logger().error(f'获取帧超时: {str(e)}')
                self.reset_camera()
            else:
                self.get_logger().error(f'RuntimeError in timer callback: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
