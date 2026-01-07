#!/usr/bin/env python3
"""
感知模块测试脚本

使用 USB 摄像头测试人脸检测和跟踪
不依赖 ROS2，可以直接运行

Usage:
    python test/perception/test_perception.py
    python test/perception/test_perception.py --camera 0
    python test/perception/test_perception.py --image test.jpg
"""

import argparse
import sys
import time
import numpy as np

# 添加项目路径
import os
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False
    print("Error: OpenCV not available. Install with: pip install opencv-python")
    sys.exit(1)

try:
    import mediapipe as mp
    MP_AVAILABLE = True
except ImportError:
    MP_AVAILABLE = False
    print("Warning: MediaPipe not available. Install with: pip install mediapipe")

from wedding_interaction.perception import FaceDetector, FaceTracker


def draw_face(frame, face, is_tracked=False):
    """在图像上绘制人脸框"""
    h, w = frame.shape[:2]
    x1 = int(face.x * w)
    y1 = int(face.y * h)
    x2 = int((face.x + face.width) * w)
    y2 = int((face.y + face.height) * h)
    
    # 根据是否正脸和是否被跟踪选择颜色
    if is_tracked:
        color = (0, 255, 255)  # 黄色 = 跟踪中
        thickness = 3
    elif face.is_frontal:
        color = (0, 255, 0)    # 绿色 = 正脸
        thickness = 2
    else:
        color = (0, 165, 255)  # 橙色 = 侧脸
        thickness = 2
    
    # 状态文本
    if face.is_frontal:
        status = "FRONTAL"
    else:
        status = "SIDE"
    
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
    
    # 显示状态和角度
    label1 = f"{status}"
    label2 = f"Y:{face.yaw:+.0f} P:{face.pitch:+.0f}"
    
    cv2.putText(frame, label1, (x1, y1 - 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(frame, label2, (x1, y1 - 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    
    # 绘制中心点
    cx, cy = int(face.center_x * w), int(face.center_y * h)
    cv2.circle(frame, (cx, cy), 5, color, -1)
    
    # 绘制头部朝向指示线（基于 yaw）
    if abs(face.yaw) > 5:  # 只在明显偏转时显示
        line_length = 30
        angle_rad = face.yaw * 3.14159 / 180
        end_x = int(cx + line_length * np.sin(angle_rad))
        end_y = cy
        cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 255), 2, tipLength=0.3)


def draw_tracking_info(frame, tracker):
    """绘制跟踪信息"""
    h, w = frame.shape[:2]
    info = tracker.get_info()
    
    # 绘制有效区域
    x_min, x_max, y_min, y_max = tracker.VALID_REGION
    cv2.rectangle(frame, 
                  (int(x_min * w), int(y_min * h)),
                  (int(x_max * w), int(y_max * h)),
                  (100, 100, 100), 1)
    
    # 状态信息面板背景
    panel_h = 140
    cv2.rectangle(frame, (5, 5), (250, panel_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (5, 5), (250, panel_h), (100, 100, 100), 1)
    
    # 状态信息
    y_offset = 25
    line_height = 22
    
    texts = [
        f"Has Target: {info['has_target']}",
        f"Is Frontal: {info['is_frontal']}",
        f"Duration: {info['frontal_duration']:.1f}s / 2.0s",
        f"Confirmed: {info['is_confirmed']}",
    ]
    
    for i, text in enumerate(texts):
        # 根据状态选择颜色
        if 'Confirmed: True' in text:
            text_color = (0, 255, 0)  # 绿色
        elif 'Has Target: True' in text:
            text_color = (0, 255, 255)  # 黄色
        else:
            text_color = (255, 255, 255)  # 白色
        
        cv2.putText(frame, text, (15, y_offset + i * line_height),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, text_color, 1)
    
    # 进度条
    progress = min(1.0, info['frontal_duration'] / 2.0)
    bar_x, bar_y = 15, panel_h - 20
    bar_w, bar_h = 220, 12
    
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 50, 50), -1)
    if progress > 0:
        fill_w = int(bar_w * progress)
        color = (0, 255, 0) if info['is_confirmed'] else (0, 200, 255)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), color, -1)
    cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (150, 150, 150), 1)
    
    # 绘制跟踪目标位置
    if info['has_target']:
        tx, ty = info['position']
        # 绘制十字准心
        cx, cy = int(tx * w), int(ty * h)
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 2)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 20, (0, 255, 255), 2)


def test_camera(camera_id: int = 0):
    """使用 USB 摄像头测试"""
    print(f"\n=== 感知模块测试 (Camera {camera_id}) ===\n")
    
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {camera_id}")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 创建检测器和跟踪器
    face_detector = FaceDetector(use_face_mesh=MP_AVAILABLE)
    tracker = FaceTracker()
    
    print("Controls:")
    print("  q - Quit")
    print("  r - Reset tracker")
    print()
    print("显示说明:")
    print("  绿色框 = 正脸")
    print("  橙色框 = 侧脸")
    print("  黄色框 = 跟踪中的目标")
    print("  进度条 = 正脸持续时间 (满 2s 确认)")
    print()
    
    frame_count = 0
    start_time = time.time()
    last_time = start_time
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # 检测人脸
            faces = face_detector.detect(frame)
            
            # 更新跟踪器
            state = tracker.update(faces, dt)
            
            # 绘制所有人脸
            for face in faces:
                is_tracked = (state.target_face is not None and 
                             face.distance_to(state.target_face) < 0.05)
                draw_face(frame, face, is_tracked=is_tracked)
            
            # 绘制跟踪信息
            draw_tracking_info(frame, tracker)
            
            # 确认状态提示
            if tracker.is_target_confirmed:
                cv2.putText(frame, "TARGET LOCKED!", 
                           (frame.shape[1] // 2 - 100, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # FPS
            frame_count += 1
            if frame_count % 30 == 0:
                fps = frame_count / (current_time - start_time)
                print(f"\rFPS: {fps:.1f}  Faces: {len(faces)}  "
                      f"Tracking: {state.has_target}  "
                      f"Confirmed: {tracker.is_target_confirmed}", end="")
            
            cv2.imshow("Face Detection & Tracking", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                tracker.reset()
                print("\nTracker reset")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        face_detector.release()
        print("\n\nTest completed.")


def test_image(image_path: str):
    """测试单张图片"""
    print(f"\n=== 感知模块测试 (Image: {image_path}) ===\n")
    
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Cannot read image {image_path}")
        return
    
    # 创建检测器
    face_detector = FaceDetector(use_face_mesh=MP_AVAILABLE)
    
    # 检测
    faces = face_detector.detect(frame)
    
    print(f"Detected {len(faces)} faces")
    
    for i, face in enumerate(faces):
        print(f"  Face {i}: pos=({face.center_x:.2f}, {face.center_y:.2f}), "
              f"area={face.area:.4f}, frontal={face.is_frontal}, "
              f"yaw={face.yaw:+.1f}, pitch={face.pitch:+.1f}")
    
    # 绘制
    for face in faces:
        draw_face(frame, face)
    
    cv2.imshow("Face Detection", frame)
    print("\nPress any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    face_detector.release()


def main():
    parser = argparse.ArgumentParser(description="Face Detection & Tracking Test")
    parser.add_argument('--camera', type=int, default=0,
                        help='USB camera ID (default: 0)')
    parser.add_argument('--image', type=str, default=None,
                        help='Test single image instead of camera')
    args = parser.parse_args()
    
    print("=" * 50)
    print("Face Detection & Tracking Test")
    print("=" * 50)
    print(f"OpenCV: {CV_AVAILABLE}")
    print(f"MediaPipe: {MP_AVAILABLE}")
    
    if args.image:
        test_image(args.image)
    else:
        test_camera(args.camera)


if __name__ == '__main__':
    main()
