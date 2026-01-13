"""
Interview 测试 Launch 文件

用于测试Interview功能的完整流程

Usage:
    # 先启动仿真器
    ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
    
    # 再启动此 launch
    ros2 launch wedding_interaction interview_test.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成Launch描述"""
    
    # 参数声明
    fsm_rate_arg = DeclareLaunchArgument(
        'fsm_rate',
        default_value='25.0',
        description='FSM loop rate in Hz'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='500.0',
        description='Motion control rate in Hz'
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
    
    enable_action_arg = DeclareLaunchArgument(
        'enable_action',
        default_value='true',
        description='Enable action execution'
    )
    
    enable_speech_arg = DeclareLaunchArgument(
        'enable_speech',
        default_value='true',
        description='Enable speech playback'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable perception visualization'
    )
    
    # 各节点独立的日志级别参数
    perception_node_log_level_arg = DeclareLaunchArgument(
        'perception_node_log_level',
        default_value='warn',
        description='Log level for perception_node (debug, info, warn, error, fatal)'
    )
    
    fsm_node_log_level_arg = DeclareLaunchArgument(
        'fsm_node_log_level',
        default_value='info',
        description='Log level for wedding_fsm_node (debug, info, warn, error, fatal)'
    )
    
    motion_adapter_log_level_arg = DeclareLaunchArgument(
        'motion_adapter_log_level',
        default_value='warn',
        description='Log level for motion_adapter_node (debug, info, warn, error, fatal)'
    )
    
    speech_player_log_level_arg = DeclareLaunchArgument(
        'speech_player_log_level',
        default_value='warn',
        description='Log level for speech_player_node (debug, info, warn, error, fatal)'
    )
    
    perception_visualizer_log_level_arg = DeclareLaunchArgument(
        'perception_visualizer_log_level',
        default_value='warn',
        description='Log level for perception_visualizer_node (debug, info, warn, error, fatal)'
    )
    
    # 音频资源目录
    audio_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'audio_resources'
    )
    
    return LaunchDescription([
        # 参数
        fsm_rate_arg,
        control_rate_arg,
        use_usb_camera_arg,
        image_topic_arg,
        enable_action_arg,
        enable_speech_arg,
        enable_visualization_arg,
        perception_node_log_level_arg,
        fsm_node_log_level_arg,
        motion_adapter_log_level_arg,
        speech_player_log_level_arg,
        perception_visualizer_log_level_arg,
        
        # 日志
        LogInfo(msg=['Starting Interview test environment...']),
        LogInfo(msg=['FSM rate: ', LaunchConfiguration('fsm_rate'), ' Hz']),
        LogInfo(msg=['Image topic: ', LaunchConfiguration('image_topic')]),
        LogInfo(msg=['Perception node log level: ', LaunchConfiguration('perception_node_log_level')]),
        LogInfo(msg=['FSM node log level: ', LaunchConfiguration('fsm_node_log_level')]),
        LogInfo(msg=['Motion adapter log level: ', LaunchConfiguration('motion_adapter_log_level')]),
        LogInfo(msg=['Speech player log level: ', LaunchConfiguration('speech_player_log_level')]),
        LogInfo(msg=['Perception visualizer log level: ', LaunchConfiguration('perception_visualizer_log_level')]),
        
        # 感知节点
        Node(
            package='wedding_interaction',
            executable='perception_node',
            name='perception_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('perception_node_log_level')],
            parameters=[{
                'use_usb_camera': LaunchConfiguration('use_usb_camera'),
                'image_topic': LaunchConfiguration('image_topic'),
                'detection_rate': 30.0,
                'use_face_mesh': True,
                'model_selection': 0,
            }],
        ),
        
        # FSM 节点
        Node(
            package='wedding_interaction',
            executable='wedding_fsm_node',
            name='wedding_fsm_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('fsm_node_log_level')],
            parameters=[{
                'fsm_rate': LaunchConfiguration('fsm_rate'),
                'initial_state': 'IDLE',
                'enable_action': LaunchConfiguration('enable_action'),
                'enable_speech': LaunchConfiguration('enable_speech'),
            }],
        ),
        
        # 运动适配节点
        Node(
            package='wedding_interaction',
            executable='motion_adapter',
            name='motion_adapter_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('motion_adapter_log_level')],
            parameters=[{
                'control_rate': LaunchConfiguration('control_rate'),
                'head_max_vel': 1.5,
                'head_max_acc': 5.0,
                'waist_max_vel': 1.0,
                'waist_max_acc': 4.0,
            }],
        ),
        
        # 语音播放节点
        Node(
            package='wedding_interaction',
            executable='speech_player',  # 注意：setup.py中定义的是speech_player，不是speech_player_node
            name='speech_player_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('speech_player_log_level')],
            parameters=[{
                'audio_dir': audio_dir,
                'tts_enabled': True,
                'tts_engine': 'edge',  # 使用 edge-tts 以匹配预录音频音色
                'tts_voice': 'zh-CN-XiaoxiaoNeural',  # 与 generate_audio.py 一致
                'tts_rate': '+0%',
            }],
        ),
        
        # 感知可视化节点（可选）
        Node(
            package='wedding_interaction',
            executable='perception_visualizer',
            name='perception_visualizer_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('perception_visualizer_log_level')],
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'output_topic': '/wedding/perception/visualization',
                'publish_rate': 30.0,
            }],
            condition=IfCondition(LaunchConfiguration('enable_visualization')),
        ),
        
        # 测试提示信息
        LogInfo(
            msg=['Interview test launch file started']
        ),
        LogInfo(
            msg=['To trigger interview, run:']
        ),
        LogInfo(
            msg=['  ros2 topic pub /wedding/fsm/command std_msgs/String "data: \'interview\'" --once']
        ),
    ])

