#!/usr/bin/env python3
"""
测试人脸检测的最小人脸像素尺寸

使用 OpenCV Haar Cascade 进行测试
(MediaPipe FaceLandmarker 对这张图片检测失败)

Usage:
    python3 test/perception/test_min_face_size.py <图片路径>
"""

import sys
import os
import cv2
import numpy as np


def detect_faces_opencv(image):
    """使用 OpenCV Haar Cascade 检测人脸"""
    cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
    face_cascade = cv2.CascadeClassifier(cascade_path)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = face_cascade.detectMultiScale(
        gray, 
        scaleFactor=1.1, 
        minNeighbors=5, 
        minSize=(20, 20)  # 最小检测尺寸
    )
    
    faces = []
    for (x, y, w, h) in detections:
        faces.append({
            'x': x,
            'y': y,
            'width': w,
            'height': h
        })
    
    return faces


def test_at_scale(image, scale):
    """在指定缩放比例下测试"""
    h, w = image.shape[:2]
    new_w = int(w * scale)
    new_h = int(h * scale)
    
    if new_w < 50 or new_h < 50:
        return None
    
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    faces = detect_faces_opencv(resized)
    
    return {
        'scale': scale,
        'image_size': (new_w, new_h),
        'num_faces': len(faces),
        'faces': faces
    }


def main():
    if len(sys.argv) < 2:
        image_path = "/home/lingjing/project/engine_ai/engineai_ros2_workspace/src/simulation/mujoco/assets/resource/environment/163734429_1565343012618.jpg"
    else:
        image_path = sys.argv[1]
    
    if not os.path.exists(image_path):
        print(f"错误: 文件不存在 - {image_path}")
        sys.exit(1)
    
    print("=" * 70)
    print("人脸检测最小像素尺寸测试 (OpenCV Haar Cascade)")
    print("=" * 70)
    print(f"测试图片: {os.path.basename(image_path)}")
    print()
    
    # 读取图片
    image = cv2.imread(image_path)
    if image is None:
        print(f"错误: 无法读取图片")
        sys.exit(1)
    
    h, w = image.shape[:2]
    print(f"原始图片尺寸: {w} x {h}")
    
    # 原图检测
    print("\n--- 原图检测 ---")
    faces = detect_faces_opencv(image)
    print(f"检测到 {len(faces)} 个人脸")
    
    if faces:
        print("\n各人脸详情:")
        for i, face in enumerate(faces):
            print(f"  人脸 {i+1}: {face['width']:.0f} x {face['height']:.0f} 像素, "
                  f"位置: ({face['x']}, {face['y']})")
        
        # 统计
        face_sizes = [f['width'] for f in faces]
        print(f"\n人脸尺寸范围: {min(face_sizes):.0f} ~ {max(face_sizes):.0f} 像素")
    
    expected_faces = len(faces)
    
    if expected_faces == 0:
        print("\n警告: 原图未检测到人脸")
        return
    
    # 测试不同缩放比例
    print("\n" + "=" * 70)
    print("缩放测试")
    print("=" * 70)
    print(f"{'缩放比例':^10} | {'图片尺寸':^15} | {'检测人数':^10} | {'最小人脸尺寸':^20}")
    print("-" * 70)
    
    scales = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.45, 0.4, 0.35, 0.3, 0.28, 0.26, 0.24, 0.22, 0.2, 0.18, 0.15]
    
    results = []
    last_full_detection = None
    last_any_detection = None
    
    for scale in scales:
        result = test_at_scale(image, scale)
        if result is None:
            continue
        
        results.append(result)
        
        new_w, new_h = result['image_size']
        num_faces = result['num_faces']
        
        # 计算最小人脸尺寸
        if num_faces > 0:
            min_w = min(f['width'] for f in result['faces'])
            min_h = min(f['height'] for f in result['faces'])
            face_size_str = f"{min_w:.0f} x {min_h:.0f} px"
            last_any_detection = result
        else:
            min_w, min_h = 0, 0
            face_size_str = "N/A"
        
        # 状态标记
        if num_faces == expected_faces:
            status = "✓"
            last_full_detection = result
        elif num_faces > 0:
            status = "△"
        else:
            status = "✗"
        
        size_str = f"{new_w} x {new_h}"
        print(f"{scale*100:>8.0f}%  | {size_str:^15} | {num_faces:^6} {status:^3} | {face_size_str:^20}")
    
    # 精细测试
    if len(results) >= 2:
        print("\n" + "=" * 70)
        print("精细测试 (在临界区域)")
        print("=" * 70)
        
        # 找临界区间 - 所有人脸到部分人脸
        for i in range(len(results) - 1):
            if results[i]['num_faces'] > results[i+1]['num_faces'] and results[i]['num_faces'] == expected_faces:
                start_scale = results[i]['scale']
                end_scale = results[i+1]['scale']
                
                print(f"\n在 {start_scale*100:.0f}% ~ {end_scale*100:.0f}% 区间精细测试:")
                
                # 精细测试
                for j in range(1, 20):
                    scale = start_scale - (start_scale - end_scale) * j / 20
                    result = test_at_scale(image, scale)
                    if result is None:
                        continue
                    
                    new_w, new_h = result['image_size']
                    num_faces = result['num_faces']
                    
                    if num_faces > 0:
                        min_w = min(f['width'] for f in result['faces'])
                        min_h = min(f['height'] for f in result['faces'])
                        face_size_str = f"{min_w:.0f} x {min_h:.0f} px"
                    else:
                        face_size_str = "N/A"
                    
                    status = "✓" if num_faces == expected_faces else ("△" if num_faces > 0 else "✗")
                    
                    if num_faces == expected_faces:
                        if last_full_detection is None or scale < last_full_detection['scale']:
                            last_full_detection = result
                    
                    if num_faces > 0:
                        last_any_detection = result
                    
                    size_str = f"{new_w} x {new_h}"
                    print(f"{scale*100:>8.1f}%  | {size_str:^15} | {num_faces:^6} {status:^3} | {face_size_str:^20}")
                
                break
    
    # 总结
    print("\n" + "=" * 70)
    print("测试结论")
    print("=" * 70)
    
    print(f"\n原图检测到: {expected_faces} 个人脸")
    
    if last_full_detection:
        scale = last_full_detection['scale']
        new_w, new_h = last_full_detection['image_size']
        faces = last_full_detection['faces']
        
        min_face_w = min(f['width'] for f in faces)
        min_face_h = min(f['height'] for f in faces)
        avg_face_w = np.mean([f['width'] for f in faces])
        avg_face_h = np.mean([f['height'] for f in faces])
        
        print(f"\n【检测全部 {expected_faces} 个人脸】")
        print(f"  最小图片缩放: {scale*100:.1f}%")
        print(f"  对应图片尺寸: {new_w} x {new_h}")
        print(f"  此时最小人脸尺寸: {min_face_w:.0f} x {min_face_h:.0f} 像素")
        print(f"  此时平均人脸尺寸: {avg_face_w:.0f} x {avg_face_h:.0f} 像素")
    
    if last_any_detection and last_any_detection != last_full_detection:
        scale = last_any_detection['scale']
        new_w, new_h = last_any_detection['image_size']
        faces = last_any_detection['faces']
        num_faces = len(faces)
        
        min_face_w = min(f['width'] for f in faces)
        min_face_h = min(f['height'] for f in faces)
        
        print(f"\n【至少检测到 1 个人脸】")
        print(f"  最小图片缩放: {scale*100:.1f}%")
        print(f"  对应图片尺寸: {new_w} x {new_h}")
        print(f"  此时检测到: {num_faces} 个人脸")
        print(f"  此时最小人脸尺寸: {min_face_w:.0f} x {min_face_h:.0f} 像素")
    
    print("\n【建议】")
    if last_full_detection:
        min_face = min(f['width'] for f in last_full_detection['faces'])
        print(f"  - OpenCV Haar Cascade 建议最小人脸尺寸: ~{min_face:.0f} 像素")
        print(f"  - MediaPipe 通常需要更大的人脸尺寸 (约 100x100 像素以上)")
        print(f"  - 在仿真中，确保人脸区域在图像中占足够大的比例")
    
    # 生成可视化结果
    print("\n生成可视化结果...")
    result_image = image.copy()
    faces = detect_faces_opencv(image)
    
    for i, face in enumerate(faces):
        x1 = int(face['x'])
        y1 = int(face['y'])
        x2 = int(face['x'] + face['width'])
        y2 = int(face['y'] + face['height'])
        
        cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        label = f"#{i+1}: {face['width']:.0f}x{face['height']:.0f}px"
        cv2.putText(result_image, label, (x1, y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    result_path = image_path.rsplit('.', 1)[0] + '_face_detection_result.png'
    cv2.imwrite(result_path, result_image)
    print(f"结果图片已保存: {result_path}")
    
    print("\n测试完成!")


if __name__ == '__main__':
    main()
