#!/usr/bin/env python3
"""
Interview 手动功能测试脚本

用于手动测试Interview功能的完整流程

Usage:
    # 1. 启动仿真器和节点
    ros2 launch mujoco_simulator mujoco_simulator.launch.py product:=pm_v2
    ros2 launch wedding_interaction interview_test.launch.py
    
    # 2. 运行此脚本
    python3 test/functional/test_interview_manual.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import os


class InterviewTestNode(Node):
    """Interview测试节点"""
    
    def __init__(self):
        super().__init__('interview_test_node')
        
        # 发布者
        self.command_pub = self.create_publisher(
            String,
            '/wedding/fsm/command',
            10
        )
        
        # 订阅者
        self.state_sub = self.create_subscription(
            String,
            '/wedding/fsm/state',
            self._on_state,
            10
        )
        
        self.speech_sub = self.create_subscription(
            String,
            '/wedding/audio/play',
            self._on_speech,
            10
        )
        
        self.pose_sub = self.create_subscription(
            String,
            '/wedding/motion/pose',
            self._on_pose,
            10
        )
        
        # 订阅调试日志（用于查看录制相关信息）
        self.debug_log_sub = self.create_subscription(
            String,
            '/wedding/fsm/debug_log',
            self._on_debug_log,
            10
        )
        
        # 状态跟踪
        self.current_state = "UNKNOWN"
        self.last_speech = ""
        self.last_pose = ""
        self.debug_logs = []
        
        self.get_logger().info("Interview test node initialized")
        self.get_logger().info("Waiting for FSM to be ready...")
    
    def _on_state(self, msg: String):
        """状态回调"""
        if self.current_state != msg.data:
            self.get_logger().info(f"[状态变化] {self.current_state} -> {msg.data}")
            self.current_state = msg.data
    
    def _on_speech(self, msg: String):
        """语音回调"""
        if self.last_speech != msg.data:
            self.get_logger().info(f"[语音播放] {msg.data}")
            self.last_speech = msg.data
    
    def _on_pose(self, msg: String):
        """Pose回调"""
        if self.last_pose != msg.data:
            self.get_logger().info(f"[Pose变化] {msg.data}")
            self.last_pose = msg.data
    
    def _on_debug_log(self, msg: String):
        """调试日志回调"""
        log_msg = msg.data
        # 过滤录制相关的日志
        if any(keyword in log_msg.lower() for keyword in ['recording', 'record', 'video', 'audio', 'interview']):
            self.get_logger().info(f"[FSM日志] {log_msg}")
            self.debug_logs.append(log_msg)
    
    def wait_for_state(self, target_state: str, timeout: float = 10.0):
        """等待状态转换"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.current_state == target_state:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False
    
    def trigger_interview(self):
        """触发采访"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("触发采访命令")
        self.get_logger().info("=" * 60)
        
        # 确保当前在TRACKING状态
        if self.current_state != "TRACKING":
            self.get_logger().warning(f"当前状态不是TRACKING，而是: {self.current_state}")
            self.get_logger().info("等待进入TRACKING状态...")
            if not self.wait_for_state("TRACKING", timeout=10.0):
                self.get_logger().error("未能进入TRACKING状态，无法触发采访")
                return False
        
        # 发送命令
        msg = String()
        msg.data = "interview"
        self.command_pub.publish(msg)
        self.get_logger().info("已发送采访命令，等待状态转换...")
        
        # 等待一小段时间确保命令被接收
        time.sleep(0.2)
        
        # 等待转换到INTERVIEW状态（增加超时时间）
        if self.wait_for_state("INTERVIEW", timeout=10.0):
            self.get_logger().info("✓ 成功进入INTERVIEW状态")
            return True
        else:
            self.get_logger().error(f"✗ 未能进入INTERVIEW状态，当前状态: {self.current_state}")
            return False
    
    def monitor_interview(self, duration: float = 30.0):
        """监控采访过程"""
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"监控采访过程（{duration}秒）")
        self.get_logger().info("=" * 60)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.current_state != "INTERVIEW":
                self.get_logger().info(f"状态已变化: {self.current_state}")
                break
            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(0.1)
        
        self.get_logger().info("监控结束")
    
    def check_recording_file(self, save_path: str = "/tmp/wedding_blessings/"):
        """检查录制文件"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("检查录制文件")
        self.get_logger().info("=" * 60)
        
        # 显示录制相关的调试日志
        if self.debug_logs:
            self.get_logger().info("录制相关日志:")
            for log in self.debug_logs[-10:]:  # 显示最后10条
                self.get_logger().info(f"  {log}")
        
        # 检查保存目录
        if not os.path.exists(save_path):
            self.get_logger().warning(f"保存路径不存在: {save_path}")
            self.get_logger().info("尝试创建目录...")
            try:
                os.makedirs(save_path, exist_ok=True)
                self.get_logger().info(f"目录已创建: {save_path}")
            except Exception as e:
                self.get_logger().error(f"无法创建目录: {e}")
            return
        
        # 检查临时文件（可能在/tmp目录）
        temp_dir = "/tmp"
        temp_video_files = [f for f in os.listdir(temp_dir) if f.startswith("temp_video_") and f.endswith(".avi")]
        temp_audio_files = [f for f in os.listdir(temp_dir) if f.startswith("temp_audio_") and f.endswith(".wav")]
        
        if temp_video_files:
            self.get_logger().info(f"找到临时视频文件: {len(temp_video_files)} 个")
        if temp_audio_files:
            self.get_logger().info(f"找到临时音频文件: {len(temp_audio_files)} 个")
        
        # 检查最终文件
        files = [f for f in os.listdir(save_path) if f.endswith('.mp4')]
        if files:
            # 按修改时间排序，获取最新的文件
            files.sort(key=lambda f: os.path.getmtime(os.path.join(save_path, f)), reverse=True)
            latest_file = os.path.join(save_path, files[0])
            file_size = os.path.getsize(latest_file)
            file_size_mb = file_size / (1024 * 1024)
            
            self.get_logger().info(f"✓ 找到录制文件: {latest_file}")
            self.get_logger().info(f"  文件大小: {file_size_mb:.2f} MB")
            self.get_logger().info(f"  修改时间: {time.ctime(os.path.getmtime(latest_file))}")
        else:
            self.get_logger().warning("未找到录制文件")
            self.get_logger().info("可能的原因:")
            self.get_logger().info("  1. 录制器启动失败（检查依赖：opencv, pyaudio, ffmpeg）")
            self.get_logger().info("  2. 视频帧回调返回None（检查图像topic是否正常）")
            self.get_logger().info("  3. 录制时间太短（小于5秒）")
            self.get_logger().info("  4. 合并失败（检查ffmpeg是否可用）")


def main():
    """主函数"""
    rclpy.init()
    node = InterviewTestNode()
    
    try:
        # 等待FSM初始化
        time.sleep(2.0)
        
        # 等待进入TRACKING状态（如果有目标）
        node.get_logger().info("等待FSM进入TRACKING状态...")
        node.wait_for_state("TRACKING", timeout=10.0)
        
        if node.current_state != "TRACKING":
            node.get_logger().warning("当前不在TRACKING状态，可能无法触发采访")
            node.get_logger().info(f"当前状态: {node.current_state}")
        
        # 触发采访
        if node.trigger_interview():
            # 监控采访过程
            node.monitor_interview(duration=30.0)
            
            # 检查录制文件
            node.check_recording_file()
        
        node.get_logger().info("=" * 60)
        node.get_logger().info("测试完成")
        node.get_logger().info("=" * 60)
        
    except KeyboardInterrupt:
        node.get_logger().info("测试中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

