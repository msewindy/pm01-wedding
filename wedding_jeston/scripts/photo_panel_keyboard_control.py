#!/usr/bin/env python3
"""
Photo Panel 键盘控制节点

使用键盘控制仿真中 photo_panel 的位移，支持平滑持续移动

控制方式（持续移动，按住按键）：
- W / ↑ : 远离机器人 (X+)，以固定速度持续移动
- S / ↓ : 靠近机器人 (X-)，以固定速度持续移动
- A / ← : 向左移动 (Y+)，以固定速度持续移动
- D / → : 向右移动 (Y-)，以固定速度持续移动
- 其他键: 停止移动
- R : 重置到初始位置
- Q : 退出

移动参数：
- 移动速度: 15cm/s (0.15m/s)
- 更新频率: 50Hz

发布话题：
- /sim/photo_panel/position (geometry_msgs/Point): photo_panel 的目标位置

Usage:
    python3 scripts/photo_panel_keyboard_control.py
"""

import sys
import termios
import tty
import threading
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class PhotoPanelKeyboardControl(Node):
    """Photo Panel 键盘控制节点"""
    
    # 初始位置（更近以增大人脸像素）
    INITIAL_X = 1.0
    INITIAL_Y = 0.0
    INITIAL_Z = 1.5
    
    # 移动参数
    MOVE_STEP = 0.1  # 每次移动距离（米，已废弃，保留用于兼容）
    MOVE_VELOCITY = 0.15  # 移动速度（米/秒）= 15cm/s
    UPDATE_RATE = 50.0  # 位置更新频率（Hz）
    X_MIN = 1.0      # 最近距离
    X_MAX = 5.0      # 最远距离
    Y_MIN = -1.5     # 最左侧
    Y_MAX = 1.5      # 最右侧
    
    def __init__(self):
        super().__init__('photo_panel_keyboard_control')
        
        # 当前位置
        self.current_x = self.INITIAL_X
        self.current_y = self.INITIAL_Y
        self.current_z = self.INITIAL_Z
        
        # 发布器
        self.position_pub = self.create_publisher(
            Point,
            '/sim/photo_panel/position',
            10
        )
        
        # 运行标志
        self.running = True
        
        # 移动状态：'forward', 'backward', 'left', 'right', None
        self.move_direction = None
        self.last_update_time = time.time()
        self.last_key_time = time.time()  # 最后一次按键时间
        self.key_timeout = 0.2  # 按键超时时间（秒），超过此时间无新输入则停止移动
        
        # 打印控制说明
        self.get_logger().info("=" * 50)
        self.get_logger().info("Photo Panel 键盘控制")
        self.get_logger().info("=" * 50)
        self.get_logger().info("控制方式（持续移动）:")
        self.get_logger().info("  W / ↑ : 远离机器人 (X+)，按住持续移动")
        self.get_logger().info("  S / ↓ : 靠近机器人 (X-)，按住持续移动")
        self.get_logger().info("  A / ← : 向左移动 (Y+)，按住持续移动")
        self.get_logger().info("  D / → : 向右移动 (Y-)，按住持续移动")
        self.get_logger().info("  其他键: 停止移动")
        self.get_logger().info("  R     : 重置到初始位置")
        self.get_logger().info("  Q     : 退出")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"移动速度: {self.MOVE_VELOCITY*100:.1f}cm/s")
        self.get_logger().info(f"初始位置: X={self.current_x:.2f}m, Y={self.current_y:.2f}m")
        self.get_logger().info(f"X 范围: {self.X_MIN}m ~ {self.X_MAX}m")
        self.get_logger().info(f"Y 范围: {self.Y_MIN}m ~ {self.Y_MAX}m")
        self.get_logger().info("")
        
        # 发布初始位置
        self._publish_position()
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        
        # 定时器用于位置更新和 ROS2 spin
        update_period = 1.0 / self.UPDATE_RATE
        self.timer = self.create_timer(update_period, self._timer_callback)
    
    def _timer_callback(self):
        """定时器回调，更新位置并保持节点运行"""
        if not self.running:
            rclpy.shutdown()
            return
        
        # 持续移动更新
        self._update_position()
    
    def _keyboard_loop(self):
        """键盘监听循环（非阻塞）"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(fd)
            while self.running:
                # 非阻塞输入检测
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    self.last_key_time = time.time()  # 更新按键时间
                    
                    if ch == '\x1b':  # 转义序列（方向键）
                        # 检查是否有更多字符可读
                        if select.select([sys.stdin], [], [], 0.01)[0]:
                            ch2 = sys.stdin.read(1)
                            if select.select([sys.stdin], [], [], 0.01)[0]:
                                ch3 = sys.stdin.read(1)
                                if ch2 == '[':
                                    if ch3 == 'A':  # ↑
                                        self._start_move('forward')
                                    elif ch3 == 'B':  # ↓
                                        self._start_move('backward')
                                    elif ch3 == 'C':  # →
                                        self._start_move('right')
                                    elif ch3 == 'D':  # ←
                                        self._start_move('left')
                    elif ch.lower() == 'w':
                        self._start_move('forward')
                    elif ch.lower() == 's':
                        self._start_move('backward')
                    elif ch.lower() == 'a':
                        self._start_move('left')
                    elif ch.lower() == 'd':
                        self._start_move('right')
                    elif ch.lower() == 'r':
                        self._stop_move()
                        self._reset_position()
                    elif ch.lower() == 'q' or ch == '\x03':  # q 或 Ctrl+C
                        self.get_logger().info("退出...")
                        self.running = False
                        break
                    else:
                        # 其他按键停止移动
                        self._stop_move()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def _start_move(self, direction):
        """开始移动（设置移动方向）"""
        if self.move_direction != direction:
            direction_names = {
                'forward': '远离机器人 (X+)',
                'backward': '靠近机器人 (X-)',
                'left': '向左移动 (Y+)',
                'right': '向右移动 (Y-)'
            }
            self.get_logger().info(f"开始移动: {direction_names.get(direction, direction)}")
        self.move_direction = direction
        self.last_update_time = time.time()
        self.last_key_time = time.time()  # 更新按键时间
    
    def _stop_move(self):
        """停止移动"""
        if self.move_direction is not None:
            self.get_logger().info("停止移动")
            self.move_direction = None
    
    def _update_position(self):
        """根据移动方向和速度更新位置"""
        if self.move_direction is None:
            return
        
        # 检查按键超时（模拟按键松开）
        current_time = time.time()
        if current_time - self.last_key_time > self.key_timeout:
            self._stop_move()
            return
        
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # 计算本次移动距离
        distance = self.MOVE_VELOCITY * dt
        
        moved = False
        if self.move_direction == 'forward':
            # 远离机器人 (X+)
            new_x = self.current_x + distance
            if new_x <= self.X_MAX:
                self.current_x = new_x
                moved = True
            else:
                self.current_x = self.X_MAX
                self._stop_move()
                self.get_logger().warning(f"已达最远距离: {self.X_MAX}m")
        elif self.move_direction == 'backward':
            # 靠近机器人 (X-)
            new_x = self.current_x - distance
            if new_x >= self.X_MIN:
                self.current_x = new_x
                moved = True
            else:
                self.current_x = self.X_MIN
                self._stop_move()
                self.get_logger().warning(f"已达最近距离: {self.X_MIN}m")
        elif self.move_direction == 'left':
            # 向左移动 (Y+)
            new_y = self.current_y + distance
            if new_y <= self.Y_MAX:
                self.current_y = new_y
                moved = True
            else:
                self.current_y = self.Y_MAX
                self._stop_move()
                self.get_logger().warning(f"已达最左侧: {self.Y_MAX}m")
        elif self.move_direction == 'right':
            # 向右移动 (Y-)
            new_y = self.current_y - distance
            if new_y >= self.Y_MIN:
                self.current_y = new_y
                moved = True
            else:
                self.current_y = self.Y_MIN
                self._stop_move()
                self.get_logger().warning(f"已达最右侧: {self.Y_MIN}m")
        
        # 发布更新后的位置
        if moved:
            self.get_logger().info(f"目标位置: X={self.current_x:.3f}m, Y={self.current_y:.3f}m, Z={self.current_z:.3f}m")
            self._publish_position()
    
    def _reset_position(self):
        """重置到初始位置"""
        self.current_x = self.INITIAL_X
        self.current_y = self.INITIAL_Y
        self._publish_position()
        self.get_logger().info(f"重置: X={self.current_x:.2f}m, Y={self.current_y:.2f}m")
    
    def _publish_position(self):
        """发布当前位置"""
        msg = Point()
        msg.x = self.current_x
        msg.y = self.current_y
        msg.z = self.current_z
        self.position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PhotoPanelKeyboardControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
