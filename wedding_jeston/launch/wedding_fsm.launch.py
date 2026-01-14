"""
婚礼互动 FSM Launch 文件
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('wedding_interaction')
    config_file = os.path.join(pkg_dir, 'config', 'parameters.yaml')

    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='500.0',
        description='运动控制频率 (Hz) - 与 interface_example 一致'
    )

    use_usb_camera_arg = DeclareLaunchArgument(
        'use_usb_camera',
        default_value='false',
        description='Use USB camera instead of ROS2 image topic'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/head/rgb/image_raw',  # Mujoco 仿真相机 topic
        description='ROS2 image topic to subscribe'
    )
    
    # 运动适配器节点（延迟 1 秒启动，等待 FSM 初始化）
    motion_adapter_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='wedding_interaction',
                executable='motion_adapter',
                name='motion_adapter_node',
                output='screen',
                parameters=[config_file, {
                    {'control_rate': LaunchConfiguration('control_rate')},
                }],
                emulate_tty=True,
            )
        ]
    )

    
    # FSM 节点
    fsm_node = Node(
        package='wedding_interaction',
        executable='wedding_fsm_node',
        name='wedding_fsm_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )

    perception_node = Node(
        package='wedding_interaction',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[config_file, {
            'use_usb_camera': LaunchConfiguration('use_usb_camera'),
            'image_topic': LaunchConfiguration('image_topic'),
        }],
    )
    
    return LaunchDescription([
        fsm_node,
        control_rate_arg,
        motion_adapter_node,
        use_usb_camera_arg,
        image_topic_arg,
        perception_node,
    ])

