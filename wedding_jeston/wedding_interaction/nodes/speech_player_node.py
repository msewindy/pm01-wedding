"""
语音播放节点

订阅语音请求，播放预录音频或调用 TTS
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from ..audio import SpeechManager, SpeechType


class SpeechPlayerNode(Node):
    """
    语音播放节点
    
    订阅：
    - /wedding/audio/play: 语音请求（ID 或文本）
    - /wedding/audio/stop: 停止播放
    
    发布：
    - /wedding/audio/playing: 是否正在播放
    - /wedding/audio/current: 当前播放的语音 ID
    
    服务：
    - /wedding/audio/list: 列出所有预录语音
    """
    
    def __init__(self):
        super().__init__('speech_player_node')
        
        # 声明参数
        self.declare_parameter('audio_dir', '')
        self.declare_parameter('tts_enabled', True)
        self.declare_parameter('tts_engine', 'edge')  # "edge" (推荐), "piper", "sherpa", "mock"
        self.declare_parameter('tts_model', '')
        self.declare_parameter('tts_voice', 'zh-CN-XiaoxiaoNeural')  # Edge TTS 声音
        self.declare_parameter('tts_rate', '+0%')  # Edge TTS 语速
        
        audio_dir = self.get_parameter('audio_dir').value
        tts_enabled = self.get_parameter('tts_enabled').value
        tts_engine = self.get_parameter('tts_engine').value
        tts_model = self.get_parameter('tts_model').value
        tts_voice = self.get_parameter('tts_voice').value
        tts_rate = self.get_parameter('tts_rate').value
        
        # 如果未指定音频目录，使用默认目录
        if not audio_dir:
            try:
                share_dir = get_package_share_directory('wedding_interaction')
                audio_dir = str(Path(share_dir) / "audio_resources")
            except Exception as e:
                # Fallback for local development
                print(f"Error finding share directory: {e}")
                audio_dir = str(Path(__file__).parent.parent.parent / "audio_resources")
        
        print(f"DEBUG: SpeechPlayer using audio_dir: {audio_dir}")
        
        # 创建语音管理器
        self.speech_manager = SpeechManager(
            audio_dir=audio_dir if audio_dir else None,
            tts_enabled=tts_enabled,
            tts_engine=tts_engine,
            tts_model=tts_model if tts_model else None,
            tts_voice=tts_voice,
            tts_rate=tts_rate,
        )
        
        # 设置回调
        self.speech_manager.on_start(self._on_speech_start)
        self.speech_manager.on_finish(self._on_speech_finish)
        
        # ========== 订阅者 ==========
        
        self.play_sub = self.create_subscription(
            String,
            '/wedding/audio/play',
            self._on_play_request,
            10
        )
        
        self.stop_sub = self.create_subscription(
            Bool,
            '/wedding/audio/stop',
            self._on_stop_request,
            10
        )
        
        # ========== 发布者 ==========
        
        self.playing_pub = self.create_publisher(
            Bool,
            '/wedding/audio/playing',
            10
        )
        
        self.current_pub = self.create_publisher(
            String,
            '/wedding/audio/current',
            10
        )
        
        # ========== 服务 ==========
        
        self.list_srv = self.create_service(
            Trigger,
            '/wedding/audio/list',
            self._on_list_request
        )
        
        # ========== 定时器 ==========
        
        # 状态发布
        self.status_timer = self.create_timer(0.5, self._publish_status)
        
        # 当前播放 ID
        self._current_speech_id = ""
        
        self.get_logger().info("SpeechPlayerNode initialized")
        self.get_logger().info(f"Audio dir: {audio_dir}")
        self.get_logger().info(f"TTS engine: {tts_engine}")
        self.get_logger().info(f"Prerecorded speeches: {len(self.speech_manager.list_prerecorded())}")
    
    def _on_play_request(self, msg: String) -> None:
        """处理播放请求"""
        text_or_id = msg.data.strip()
        
        if not text_or_id:
            return
        
        self.get_logger().info(f"Play request: {text_or_id}")
        
        # 检查是否是特殊命令
        if text_or_id.startswith("interrupt:"):
            # 打断式播放
            content = text_or_id[10:]
            self.speech_manager.speak(content, interrupt=True)
        elif text_or_id.startswith("priority:"):
            # 带优先级的播放
            parts = text_or_id[9:].split(":", 1)
            if len(parts) == 2:
                try:
                    priority = int(parts[0])
                    content = parts[1]
                    self.speech_manager.speak(content, priority=priority)
                except ValueError:
                    self.speech_manager.speak(text_or_id)
        else:
            # 普通播放
            self.speech_manager.speak(text_or_id)
    
    def _on_stop_request(self, msg: Bool) -> None:
        """处理停止请求"""
        if msg.data:
            self.get_logger().info("Stop request received")
            self.speech_manager.stop()
    
    def _on_list_request(self, request, response) -> Trigger.Response:
        """列出所有预录语音"""
        speeches = self.speech_manager.list_prerecorded()
        lines = [f"{k}: {v}" for k, v in speeches.items()]
        response.success = True
        response.message = "\n".join(lines)
        return response
    
    def _on_speech_start(self, speech_id: str) -> None:
        """语音开始播放回调"""
        self._current_speech_id = speech_id
        self.get_logger().info(f"Playing: {speech_id}")
    
    def _on_speech_finish(self, speech_id: str) -> None:
        """语音播放完成回调"""
        self._current_speech_id = ""
        self.get_logger().info(f"Finished: {speech_id}")
    
    def _publish_status(self) -> None:
        """发布状态"""
        # 发布是否正在播放
        playing_msg = Bool()
        playing_msg.data = self.speech_manager.is_playing
        self.playing_pub.publish(playing_msg)
        
        # 发布当前播放 ID
        current_msg = String()
        current_msg.data = self._current_speech_id
        self.current_pub.publish(current_msg)
    
    def destroy_node(self):
        """销毁节点"""
        self.speech_manager.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechPlayerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

