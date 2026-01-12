"""
IDLE 状态仿真调试 Launch 文件

启动 FSM 节点和运动适配器，用于在 Mujoco 仿真中调试 IDLE 状态
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('wedding_interaction')
    config_file = os.path.join(pkg_dir, 'config', 'fsm_config.yaml')
    
    # Launch 参数
    fsm_rate_arg = DeclareLaunchArgument(
        'fsm_rate',
        default_value='50.0',
        description='FSM 运行频率 (Hz)'
    )
    
    
    # FSM 节点
    fsm_node = Node(
        package='wedding_interaction',
        executable='wedding_fsm_node',
        name='wedding_fsm_node',
        output='screen',
        parameters=[
            config_file,
            {'fsm_rate': LaunchConfiguration('fsm_rate')},
        ],
        emulate_tty=True,
    )
    
    
    # 语音播放节点
    # speech_player_node = Node(
    #     package='wedding_interaction',
    #     executable='speech_player',
    #     name='speech_player_node',
    #     output='screen',
    #     parameters=[
    #         # 音频目录：使用项目源码目录下的 audio_resources
    #         # 开发环境下使用源码路径，部署时修改为实际路径
    #         {'audio_dir': '/home/lingjing/project/engine_ai/wedding_jeston/audio_resources'},
    #         {'tts_enabled': True},
    #         {'tts_engine': 'mock'},  # 仿真模式使用 mock，真机改为 piper
    #     ],
    #     emulate_tty=True,
    # )
    
    return LaunchDescription([
        fsm_rate_arg,
        fsm_node,
        speech_player_node,
    ])

