"""
动作管理器 (ActionManager)

负责加载动作包配置，并向底层节点（SpeechPlayer, MotionAdapter）分发指令
支持：
- 状态化管理 (Active Action State)
- 动作 Pose/Motion 发布
- 语音调度 (Speech Scheduling): 支持 Once(单次) 和 Periodic(周期) 模式
- 延迟执行
"""

import random
import time
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..config.action_resources import ACTION_PACKS


class ActionState:
    """当前执行的动作状态"""
    def __init__(self, name: str, pack: Dict, start_time: float):
        self.name = name
        self.pack = pack
        self.start_time = start_time
        
        # 语音调度状态
        self.speech_config = pack.get('speech_config')
        self.next_speech_time = 0.0
        self.speech_count = 0
        self.speech_played_once = False
        
        self._init_speech_schedule(start_time)

    def _init_speech_schedule(self, current_time: float):
        """初始化语音调度"""
        if not self.speech_config:
            return

        delay = self.speech_config.get('delay', 0.0)
        self.next_speech_time = current_time + delay
    
    def get_speech_mode(self) -> Optional[str]:
        if not self.speech_config: return None
        return self.speech_config.get('mode', 'once')


class ActionManager:
    """
    动作执行管理器 (Stateful)
    
    通常嵌入在 FSM 节点中使用。
    必须周期性调用 update(current_time) 以驱动定时逻辑。
    """
    
    def __init__(self, node: Node):
        self.node = node
        
        # ROS 发布者
        # 统一使用 set_pose 话题
        self.pose_pub = node.create_publisher(String, '/wedding/motion/set_pose', 10)
        self.motion_pub = node.create_publisher(String, '/wedding/motion/set_motion', 10)
        self.speech_pub = node.create_publisher(String, '/wedding/audio/play', 10)
        
        # 内部状态
        self.current_action: Optional[ActionState] = None
        self.enable_speech = True
        
        self.node.get_logger().info(f"ActionManager initialized. Loaded {len(ACTION_PACKS)} packs.")

    def execute_action(self, action_name: str, enable_speech: bool = True) -> Optional[str]:
        """
        开始执行指定动作
        
        Args:
            action_name: 动作包名称
            enable_speech: 是否允许语音 (全局开关)
            
        Returns:
            str: 运动策略类型
        """
        if action_name not in ACTION_PACKS:
            self.node.get_logger().error(f"Action '{action_name}' not found!")
            return None
        
        if self.current_action and self.current_action.name == action_name:
            # Action already running, update timestamp or just continue?
            # To avoid log spam, we just return the strategy or continue logic without logging
            return self.current_action.pack.get('motion_strategy', 'stop')
        
        pack = ACTION_PACKS[action_name]
        self.enable_speech = enable_speech
        
        now = self.node.get_clock().now().nanoseconds / 1e9
        
        # 更新状态
        self.current_action = ActionState(action_name, pack, now)
        self.node.get_logger().info(f"Action started: {action_name}")
        
        # 1. 立即发布 Pose
        if pack.get('pose'):
            msg = String()
            msg.data = pack['pose']
            self.pose_pub.publish(msg)
            
        # 2. 立即发布 Motion
        motion_strategy = pack.get('motion_strategy')
        if motion_strategy:
            msg = String()
            msg.data = motion_strategy
            self.motion_pub.publish(msg)
        else:
            msg = String()
            msg.data = "stop"
            self.motion_pub.publish(msg)
            
        # 3. 语音逻辑交由 update() 处理 (支持延迟)
        # 如果 delay=0，update 循环中会立即触发
        # 为了响应迅速，可以在这里手动调一次 update 检查
        self.update(now)
            
        return motion_strategy

    def stop_motion(self):
        """停止当前动作和运动"""
        msg = String()
        msg.data = "stop"
        self.motion_pub.publish(msg)
        self.current_action = None

    def update(self, current_time: float):
        """
        主循环更新 (定频调用)
        处理语音调度等时间相关逻辑
        """
        if not self.current_action:
            return
            
        # 检查语音
        if self.enable_speech and self.current_action.speech_config:
            self._update_speech(current_time)

    def _update_speech(self, current_time: float):
        state = self.current_action
        config = state.speech_config
        
        # 检查是否到达播放时间
        if current_time >= state.next_speech_time:
            mode = state.get_speech_mode()
            
            should_play = False
            
            if mode == 'once':
                if not state.speech_played_once:
                    should_play = True
                    state.speech_played_once = True
                    # Once 模式播放完不需要再调度
                    state.next_speech_time = float('inf') 
            
            elif mode == 'periodic':
                should_play = True
                # 调度下一次
                interval_min = config.get('interval_min', 10.0)
                interval_max = config.get('interval_max', 20.0)
                interval = random.uniform(interval_min, interval_max)
                state.next_speech_time = current_time + interval
            
            # 执行播放
            if should_play:
                # 检查概率 (如果配置了)
                probability = config.get('probability', 1.0)
                if random.random() > probability:
                    should_play = False
                    # 即使不播放，如果是 periodic 模式，通过前面的逻辑已经调度了下一次时间
                    # 所以这里不需要额外处理，直接跳过本次
                    if mode == 'periodic':
                        pass # 下一次时间已经设置好了
                
            if should_play:
                speech_group = state.pack.get('speech_group')
                if speech_group and len(speech_group) > 0:
                    # Shuffle Bag Logic
                    if not hasattr(state, '_speech_bag') or not state._speech_bag:
                        state._speech_bag = list(speech_group)
                        random.shuffle(state._speech_bag)
                        # 避免重置后第一个和上一次最后一个相同 (仅当 group > 1)
                        if len(speech_group) > 1 and hasattr(state, '_last_speech'):
                            if state._speech_bag[0] == state._last_speech:
                                # 简单交换前两个
                                state._speech_bag[0], state._speech_bag[1] = state._speech_bag[1], state._speech_bag[0]

                    speech_id = state._speech_bag.pop(0)
                    state._last_speech = speech_id
                    
                    msg = String()
                    msg.data = speech_id
                    self.speech_pub.publish(msg)
                    # self.node.get_logger().info(f"Action speech: {speech_id}")
