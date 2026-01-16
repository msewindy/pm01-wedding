"""
Motion Only Launch 文件

仅启动 MotionAdapterNode，用于调试动作资源，避免 FSM 逻辑干扰。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('wedding_interaction')
    config_file = os.path.join(pkg_dir, 'config', 'parameters.yaml')

    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='500.0',
        description='运动控制频率 (Hz)'
    )

    # 运动适配器节点
    motion_adapter_node = Node(
        package='wedding_interaction',
        executable='motion_adapter',
        name='motion_adapter_node',
        output='screen',
        parameters=[config_file, {
            'control_rate': LaunchConfiguration('control_rate'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        control_rate_arg,
        motion_adapter_node,
    ])
