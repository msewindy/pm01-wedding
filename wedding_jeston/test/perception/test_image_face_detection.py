#!/usr/bin/env python3
"""
测试图片人脸检测

使用 MediaPipe 检测图片中的人脸

Usage:
    python3 test/perception/test_image_face_detection.py <图片路径>
    
示例:
    python3 test/perception/test_image_face_detection.py /tmp/sim_screenshot.png
"""

import sys
import os

# 添加项目路径
import os
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

import cv2
import numpy as np

from wedding_interaction.perception import FaceDetector


def main():
    # 检查参数
    if len(sys.argv) < 2:
        print("用法: python3 test_image_face_detection.py <图片路径>")
        print("示例: python3 test_image_face_detection.py /tmp/screenshot.png")
        sys.exit(1)
    
    image_path = sys.argv[1]
    
    # 检查文件是否存在
    if not os.path.exists(image_path):
        print(f"错误: 文件不存在 - {image_path}")
        sys.exit(1)
    
    print("=" * 50)
    print("图片人脸检测测试")
    print("=" * 50)
    print(f"图片: {image_path}")
    print()
    
    # 读取图片
    image = cv2.imread(image_path)
    if image is None:
        print(f"错误: 无法读取图片 - {image_path}")
        sys.exit(1)
    
    print(f"图片尺寸: {image.shape[1]} x {image.shape[0]}")
    print()
    
    # 创建检测器
    print("初始化 FaceDetector...")
    detector = FaceDetector(use_face_mesh=True)
    
    # 检测人脸
    print("检测人脸...")
    faces = detector.detect(image)
    
    print()
    print("=" * 50)
    print("检测结果")
    print("=" * 50)
    print(f"检测到 {len(faces)} 个人脸")
    print()
    
    if faces:
        for i, face in enumerate(faces):
            print(f"人脸 {i + 1}:")
            print(f"  位置: ({face.center_x:.2f}, {face.center_y:.2f})")
            print(f"  尺寸: {face.width:.3f} x {face.height:.3f}")
            print(f"  面积: {face.area:.4f}")
            print(f"  正脸: {'是' if face.is_frontal else '否'}")
            print(f"  角度: yaw={face.yaw:.1f}°, pitch={face.pitch:.1f}°, roll={face.roll:.1f}°")
            print(f"  置信度: {face.confidence:.2f}")
            print()
        
        # 绘制结果
        result_image = image.copy()
        h, w = result_image.shape[:2]
        
        for face in faces:
            x1 = int(face.x * w)
            y1 = int(face.y * h)
            x2 = int((face.x + face.width) * w)
            y2 = int((face.y + face.height) * h)
            
            # 颜色：正脸绿色，侧脸橙色
            color = (0, 255, 0) if face.is_frontal else (0, 165, 255)
            cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 3)
            
            # 标签
            label = f"{'FRONTAL' if face.is_frontal else 'SIDE'}"
            cv2.putText(result_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            # 角度
            angles = f"Y:{face.yaw:.0f} P:{face.pitch:.0f}"
            cv2.putText(result_image, angles, (x1, y2 + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 显示结果
        print("显示结果图像...")
        print("按任意键关闭窗口")
        
        cv2.imshow("Face Detection Result", result_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        # 保存结果
        result_path = image_path.rsplit('.', 1)[0] + '_result.png'
        cv2.imwrite(result_path, result_image)
        print(f"结果已保存: {result_path}")
        
        print()
        print("✅ MediaPipe 可以检测到仿真人体模型的人脸！")
    else:
        print("❌ 未检测到人脸")
        print()
        print("可能的原因:")
        print("1. 仿真人体模型的面部纹理不够真实")
        print("2. 人脸角度过大（侧脸）")
        print("3. 图片分辨率太低")
        print("4. 光照条件不佳")
        print()
        print("建议:")
        print("- 尝试使用 mock 数据进行状态机测试")
        print("- 或使用 USB 摄像头进行感知模块开发")
        
        # 仍然显示原图
        cv2.imshow("Original Image (No Face Detected)", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    # 清理
    detector.release()
    print()


if __name__ == '__main__':
    main()

