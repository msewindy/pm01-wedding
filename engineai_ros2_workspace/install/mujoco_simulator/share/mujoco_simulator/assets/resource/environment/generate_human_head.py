#!/usr/bin/env python3
"""
生成低多边形人头模型用于MuJoCo仿真中的人脸检测测试
模型包含基本的脸部特征：眼睛、鼻子、嘴巴轮廓
"""

import numpy as np
import os

def create_sphere(center, radius, u_segments=16, v_segments=12):
    """创建球体的顶点和面"""
    vertices = []
    
    for i in range(v_segments + 1):
        v = i / v_segments
        phi = v * np.pi
        
        for j in range(u_segments):
            u = j / u_segments
            theta = u * 2 * np.pi
            
            x = center[0] + radius * np.sin(phi) * np.cos(theta)
            y = center[1] + radius * np.sin(phi) * np.sin(theta)
            z = center[2] + radius * np.cos(phi)
            
            vertices.append([x, y, z])
    
    faces = []
    for i in range(v_segments):
        for j in range(u_segments):
            p1 = i * u_segments + j
            p2 = i * u_segments + (j + 1) % u_segments
            p3 = (i + 1) * u_segments + (j + 1) % u_segments
            p4 = (i + 1) * u_segments + j
            
            faces.append([p1, p2, p3])
            faces.append([p1, p3, p4])
    
    return np.array(vertices), np.array(faces)


def create_ellipsoid(center, radii, u_segments=20, v_segments=16):
    """创建椭球体（用于头部形状）"""
    vertices = []
    
    for i in range(v_segments + 1):
        v = i / v_segments
        phi = v * np.pi
        
        for j in range(u_segments):
            u = j / u_segments
            theta = u * 2 * np.pi
            
            x = center[0] + radii[0] * np.sin(phi) * np.cos(theta)
            y = center[1] + radii[1] * np.sin(phi) * np.sin(theta)
            z = center[2] + radii[2] * np.cos(phi)
            
            vertices.append([x, y, z])
    
    faces = []
    for i in range(v_segments):
        for j in range(u_segments):
            p1 = i * u_segments + j
            p2 = i * u_segments + (j + 1) % u_segments
            p3 = (i + 1) * u_segments + (j + 1) % u_segments
            p4 = (i + 1) * u_segments + j
            
            faces.append([p1, p2, p3])
            faces.append([p1, p3, p4])
    
    return np.array(vertices), np.array(faces)


def create_cylinder(center, radius, height, segments=12):
    """创建圆柱体（用于脖子）"""
    vertices = []
    
    # 顶面和底面中心
    top_center = len(vertices)
    vertices.append([center[0], center[1], center[2] + height/2])
    bottom_center = len(vertices)
    vertices.append([center[0], center[1], center[2] - height/2])
    
    # 侧面顶点
    top_ring_start = len(vertices)
    for i in range(segments):
        theta = i / segments * 2 * np.pi
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        vertices.append([x, y, center[2] + height/2])
    
    bottom_ring_start = len(vertices)
    for i in range(segments):
        theta = i / segments * 2 * np.pi
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        vertices.append([x, y, center[2] - height/2])
    
    faces = []
    
    # 顶面
    for i in range(segments):
        next_i = (i + 1) % segments
        faces.append([top_center, top_ring_start + i, top_ring_start + next_i])
    
    # 底面
    for i in range(segments):
        next_i = (i + 1) % segments
        faces.append([bottom_center, bottom_ring_start + next_i, bottom_ring_start + i])
    
    # 侧面
    for i in range(segments):
        next_i = (i + 1) % segments
        t1 = top_ring_start + i
        t2 = top_ring_start + next_i
        b1 = bottom_ring_start + i
        b2 = bottom_ring_start + next_i
        faces.append([t1, b1, b2])
        faces.append([t1, b2, t2])
    
    return np.array(vertices), np.array(faces)


def create_nose(center, size):
    """创建简单的鼻子形状（金字塔形）"""
    vertices = [
        # 底部四角
        [center[0] - size[0]/2, center[1], center[2] - size[2]/2],
        [center[0] + size[0]/2, center[1], center[2] - size[2]/2],
        [center[0] + size[0]/2, center[1], center[2] + size[2]/2],
        [center[0] - size[0]/2, center[1], center[2] + size[2]/2],
        # 尖端
        [center[0], center[1] + size[1], center[2]]
    ]
    
    faces = [
        [0, 1, 4],  # 前面
        [1, 2, 4],  # 右面
        [2, 3, 4],  # 后面
        [3, 0, 4],  # 左面
        [0, 3, 2],  # 底面
        [0, 2, 1]   # 底面
    ]
    
    return np.array(vertices), np.array(faces)


def create_human_head_model(output_path):
    """创建完整的人头模型"""
    all_vertices = []
    all_faces = []
    vertex_offset = 0
    
    # 1. 头部主体 (椭球形)
    head_center = [0, 0, 0]
    head_radii = [0.09, 0.11, 0.12]  # x, y, z 方向的半径 (米)
    head_v, head_f = create_ellipsoid(head_center, head_radii, u_segments=24, v_segments=18)
    all_vertices.extend(head_v)
    all_faces.extend(head_f + vertex_offset)
    vertex_offset += len(head_v)
    
    # 2. 左眼 (小球体，稍微凹进去)
    left_eye_center = [0.035, 0.085, 0.03]
    left_eye_v, left_eye_f = create_sphere(left_eye_center, 0.015, u_segments=10, v_segments=8)
    all_vertices.extend(left_eye_v)
    all_faces.extend(left_eye_f + vertex_offset)
    vertex_offset += len(left_eye_v)
    
    # 3. 右眼
    right_eye_center = [-0.035, 0.085, 0.03]
    right_eye_v, right_eye_f = create_sphere(right_eye_center, 0.015, u_segments=10, v_segments=8)
    all_vertices.extend(right_eye_v)
    all_faces.extend(right_eye_f + vertex_offset)
    vertex_offset += len(right_eye_v)
    
    # 4. 鼻子
    nose_center = [0, 0.10, -0.01]
    nose_size = [0.025, 0.03, 0.05]
    nose_v, nose_f = create_nose(nose_center, nose_size)
    all_vertices.extend(nose_v)
    all_faces.extend(nose_f + vertex_offset)
    vertex_offset += len(nose_v)
    
    # 5. 脖子 (圆柱体)
    neck_center = [0, 0, -0.18]
    neck_v, neck_f = create_cylinder(neck_center, 0.045, 0.12, segments=16)
    all_vertices.extend(neck_v)
    all_faces.extend(neck_f + vertex_offset)
    vertex_offset += len(neck_v)
    
    # 6. 左耳 (小椭球)
    left_ear_center = [0.10, 0, -0.02]
    left_ear_radii = [0.015, 0.01, 0.03]
    left_ear_v, left_ear_f = create_ellipsoid(left_ear_center, left_ear_radii, u_segments=8, v_segments=6)
    all_vertices.extend(left_ear_v)
    all_faces.extend(left_ear_f + vertex_offset)
    vertex_offset += len(left_ear_v)
    
    # 7. 右耳
    right_ear_center = [-0.10, 0, -0.02]
    right_ear_radii = [0.015, 0.01, 0.03]
    right_ear_v, right_ear_f = create_ellipsoid(right_ear_center, right_ear_radii, u_segments=8, v_segments=6)
    all_vertices.extend(right_ear_v)
    all_faces.extend(right_ear_f + vertex_offset)
    vertex_offset += len(right_ear_v)
    
    all_vertices = np.array(all_vertices)
    all_faces = np.array(all_faces)
    
    # 写入OBJ文件
    with open(output_path, 'w') as f:
        f.write("# Human Head Model for Face Detection Testing\n")
        f.write("# Generated for MuJoCo simulation\n")
        f.write(f"# Vertices: {len(all_vertices)}, Faces: {len(all_faces)}\n\n")
        
        # 写入顶点
        for v in all_vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        
        f.write("\n")
        
        # 写入面 (OBJ索引从1开始)
        for face in all_faces:
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    
    print(f"✅ 人头模型已生成: {output_path}")
    print(f"   顶点数: {len(all_vertices)}")
    print(f"   面数: {len(all_faces)}")
    
    return output_path


def create_full_body_model(output_path):
    """创建简化的全身人体模型（低多边形）"""
    all_vertices = []
    all_faces = []
    vertex_offset = 0
    
    # 身高约1.7米的人体模型
    # 注意：生成后会调整z坐标使脚底在z=0
    
    # 1. 头部
    head_center = [0, 0, 1.65]
    head_radii = [0.09, 0.11, 0.12]
    head_v, head_f = create_ellipsoid(head_center, head_radii, u_segments=20, v_segments=14)
    all_vertices.extend(head_v)
    all_faces.extend(head_f + vertex_offset)
    vertex_offset += len(head_v)
    
    # 2. 左眼
    left_eye_center = [0.035, 0.085 + head_center[1], 1.68]
    left_eye_v, left_eye_f = create_sphere(left_eye_center, 0.015, u_segments=8, v_segments=6)
    all_vertices.extend(left_eye_v)
    all_faces.extend(left_eye_f + vertex_offset)
    vertex_offset += len(left_eye_v)
    
    # 3. 右眼
    right_eye_center = [-0.035, 0.085 + head_center[1], 1.68]
    right_eye_v, right_eye_f = create_sphere(right_eye_center, 0.015, u_segments=8, v_segments=6)
    all_vertices.extend(right_eye_v)
    all_faces.extend(right_eye_f + vertex_offset)
    vertex_offset += len(right_eye_v)
    
    # 4. 鼻子
    nose_center = [0, 0.10 + head_center[1], 1.64]
    nose_size = [0.025, 0.03, 0.05]
    nose_v, nose_f = create_nose(nose_center, nose_size)
    all_vertices.extend(nose_v)
    all_faces.extend(nose_f + vertex_offset)
    vertex_offset += len(nose_v)
    
    # 5. 脖子
    neck_center = [0, 0, 1.45]
    neck_v, neck_f = create_cylinder(neck_center, 0.05, 0.12, segments=12)
    all_vertices.extend(neck_v)
    all_faces.extend(neck_f + vertex_offset)
    vertex_offset += len(neck_v)
    
    # 6. 躯干 (椭球体)
    torso_center = [0, 0, 1.15]
    torso_radii = [0.18, 0.12, 0.28]
    torso_v, torso_f = create_ellipsoid(torso_center, torso_radii, u_segments=16, v_segments=12)
    all_vertices.extend(torso_v)
    all_faces.extend(torso_f + vertex_offset)
    vertex_offset += len(torso_v)
    
    # 7. 左上臂
    left_upper_arm = [0.25, 0, 1.25]
    left_ua_v, left_ua_f = create_cylinder(left_upper_arm, 0.045, 0.28, segments=10)
    all_vertices.extend(left_ua_v)
    all_faces.extend(left_ua_f + vertex_offset)
    vertex_offset += len(left_ua_v)
    
    # 8. 右上臂
    right_upper_arm = [-0.25, 0, 1.25]
    right_ua_v, right_ua_f = create_cylinder(right_upper_arm, 0.045, 0.28, segments=10)
    all_vertices.extend(right_ua_v)
    all_faces.extend(right_ua_f + vertex_offset)
    vertex_offset += len(right_ua_v)
    
    # 9. 左前臂
    left_forearm = [0.25, 0, 0.95]
    left_fa_v, left_fa_f = create_cylinder(left_forearm, 0.035, 0.26, segments=10)
    all_vertices.extend(left_fa_v)
    all_faces.extend(left_fa_f + vertex_offset)
    vertex_offset += len(left_fa_v)
    
    # 10. 右前臂
    right_forearm = [-0.25, 0, 0.95]
    right_fa_v, right_fa_f = create_cylinder(right_forearm, 0.035, 0.26, segments=10)
    all_vertices.extend(right_fa_v)
    all_faces.extend(right_fa_f + vertex_offset)
    vertex_offset += len(right_fa_v)
    
    # 11. 臀部/骨盆
    pelvis_center = [0, 0, 0.85]
    pelvis_radii = [0.16, 0.10, 0.10]
    pelvis_v, pelvis_f = create_ellipsoid(pelvis_center, pelvis_radii, u_segments=14, v_segments=10)
    all_vertices.extend(pelvis_v)
    all_faces.extend(pelvis_f + vertex_offset)
    vertex_offset += len(pelvis_v)
    
    # 12. 左大腿
    left_thigh = [0.09, 0, 0.58]
    left_th_v, left_th_f = create_cylinder(left_thigh, 0.07, 0.42, segments=10)
    all_vertices.extend(left_th_v)
    all_faces.extend(left_th_f + vertex_offset)
    vertex_offset += len(left_th_v)
    
    # 13. 右大腿
    right_thigh = [-0.09, 0, 0.58]
    right_th_v, right_th_f = create_cylinder(right_thigh, 0.07, 0.42, segments=10)
    all_vertices.extend(right_th_v)
    all_faces.extend(right_th_f + vertex_offset)
    vertex_offset += len(right_th_v)
    
    # 14. 左小腿
    left_calf = [0.09, 0, 0.25]
    left_cf_v, left_cf_f = create_cylinder(left_calf, 0.05, 0.40, segments=10)
    all_vertices.extend(left_cf_v)
    all_faces.extend(left_cf_f + vertex_offset)
    vertex_offset += len(left_cf_v)
    
    # 15. 右小腿
    right_calf = [-0.09, 0, 0.25]
    right_cf_v, right_cf_f = create_cylinder(right_calf, 0.05, 0.40, segments=10)
    all_vertices.extend(right_cf_v)
    all_faces.extend(right_cf_f + vertex_offset)
    vertex_offset += len(right_cf_v)
    
    # 16. 左耳
    left_ear_center = [0.10, 0, 1.63]
    left_ear_radii = [0.015, 0.01, 0.03]
    left_ear_v, left_ear_f = create_ellipsoid(left_ear_center, left_ear_radii, u_segments=6, v_segments=4)
    all_vertices.extend(left_ear_v)
    all_faces.extend(left_ear_f + vertex_offset)
    vertex_offset += len(left_ear_v)
    
    # 17. 右耳
    right_ear_center = [-0.10, 0, 1.63]
    right_ear_radii = [0.015, 0.01, 0.03]
    right_ear_v, right_ear_f = create_ellipsoid(right_ear_center, right_ear_radii, u_segments=6, v_segments=4)
    all_vertices.extend(right_ear_v)
    all_faces.extend(right_ear_f + vertex_offset)
    vertex_offset += len(right_ear_v)
    
    all_vertices = np.array(all_vertices)
    all_faces = np.array(all_faces)
    
    # 调整z坐标，使脚底正好在z=0
    min_z = all_vertices[:, 2].min()
    all_vertices[:, 2] -= min_z
    actual_height = all_vertices[:, 2].max()
    
    # 写入OBJ文件
    with open(output_path, 'w') as f:
        f.write("# Low-poly Human Body Model for Face Detection Testing\n")
        f.write("# Generated for MuJoCo simulation\n")
        f.write(f"# Vertices: {len(all_vertices)}, Faces: {len(all_faces)}\n")
        f.write(f"# Height: {actual_height:.2f}m, Face facing +Y direction\n")
        f.write("# Feet at z=0, compatible with MuJoCo ground plane\n\n")
        
        for v in all_vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        
        f.write("\n")
        
        for face in all_faces:
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    
    print(f"✅ 全身人体模型已生成: {output_path}")
    print(f"   实际高度: {actual_height:.2f}m")
    print(f"   顶点数: {len(all_vertices)}")
    print(f"   面数: {len(all_faces)}")
    
    return output_path


def create_scaled_body_model(output_path, height=1.7, name="human"):
    """
    创建指定身高的人体模型
    height: 身高（米），默认1.7米
    """
    all_vertices = []
    all_faces = []
    vertex_offset = 0
    
    # 根据身高计算缩放比例 (基准身高1.7米)
    scale = height / 1.7
    
    # 身体各部分的基准位置和尺寸会根据scale缩放
    def scaled_pos(base_z):
        return base_z * scale
    
    def scaled_size(base_size):
        return base_size * scale
    
    # 1. 头部 (头部比例略小一些以保持自然)
    head_scale = scale * 0.95 if scale > 1 else scale * 1.05
    head_center = [0, 0, scaled_pos(1.65)]
    head_radii = [0.09 * head_scale, 0.11 * head_scale, 0.12 * head_scale]
    head_v, head_f = create_ellipsoid(head_center, head_radii, u_segments=20, v_segments=14)
    all_vertices.extend(head_v)
    all_faces.extend(head_f + vertex_offset)
    vertex_offset += len(head_v)
    
    # 2. 左眼
    left_eye_center = [0.035 * head_scale, 0.085 * head_scale, scaled_pos(1.68)]
    left_eye_v, left_eye_f = create_sphere(left_eye_center, 0.015 * head_scale, u_segments=8, v_segments=6)
    all_vertices.extend(left_eye_v)
    all_faces.extend(left_eye_f + vertex_offset)
    vertex_offset += len(left_eye_v)
    
    # 3. 右眼
    right_eye_center = [-0.035 * head_scale, 0.085 * head_scale, scaled_pos(1.68)]
    right_eye_v, right_eye_f = create_sphere(right_eye_center, 0.015 * head_scale, u_segments=8, v_segments=6)
    all_vertices.extend(right_eye_v)
    all_faces.extend(right_eye_f + vertex_offset)
    vertex_offset += len(right_eye_v)
    
    # 4. 鼻子
    nose_center = [0, 0.10 * head_scale, scaled_pos(1.64)]
    nose_size = [0.025 * head_scale, 0.03 * head_scale, 0.05 * head_scale]
    nose_v, nose_f = create_nose(nose_center, nose_size)
    all_vertices.extend(nose_v)
    all_faces.extend(nose_f + vertex_offset)
    vertex_offset += len(nose_v)
    
    # 5. 脖子
    neck_center = [0, 0, scaled_pos(1.45)]
    neck_v, neck_f = create_cylinder(neck_center, scaled_size(0.05), scaled_size(0.12), segments=12)
    all_vertices.extend(neck_v)
    all_faces.extend(neck_f + vertex_offset)
    vertex_offset += len(neck_v)
    
    # 6. 躯干
    torso_center = [0, 0, scaled_pos(1.15)]
    torso_radii = [scaled_size(0.18), scaled_size(0.12), scaled_size(0.28)]
    torso_v, torso_f = create_ellipsoid(torso_center, torso_radii, u_segments=16, v_segments=12)
    all_vertices.extend(torso_v)
    all_faces.extend(torso_f + vertex_offset)
    vertex_offset += len(torso_v)
    
    # 7. 左上臂
    left_upper_arm = [scaled_size(0.25), 0, scaled_pos(1.25)]
    left_ua_v, left_ua_f = create_cylinder(left_upper_arm, scaled_size(0.045), scaled_size(0.28), segments=10)
    all_vertices.extend(left_ua_v)
    all_faces.extend(left_ua_f + vertex_offset)
    vertex_offset += len(left_ua_v)
    
    # 8. 右上臂
    right_upper_arm = [-scaled_size(0.25), 0, scaled_pos(1.25)]
    right_ua_v, right_ua_f = create_cylinder(right_upper_arm, scaled_size(0.045), scaled_size(0.28), segments=10)
    all_vertices.extend(right_ua_v)
    all_faces.extend(right_ua_f + vertex_offset)
    vertex_offset += len(right_ua_v)
    
    # 9. 左前臂
    left_forearm = [scaled_size(0.25), 0, scaled_pos(0.95)]
    left_fa_v, left_fa_f = create_cylinder(left_forearm, scaled_size(0.035), scaled_size(0.26), segments=10)
    all_vertices.extend(left_fa_v)
    all_faces.extend(left_fa_f + vertex_offset)
    vertex_offset += len(left_fa_v)
    
    # 10. 右前臂
    right_forearm = [-scaled_size(0.25), 0, scaled_pos(0.95)]
    right_fa_v, right_fa_f = create_cylinder(right_forearm, scaled_size(0.035), scaled_size(0.26), segments=10)
    all_vertices.extend(right_fa_v)
    all_faces.extend(right_fa_f + vertex_offset)
    vertex_offset += len(right_fa_v)
    
    # 11. 臀部/骨盆
    pelvis_center = [0, 0, scaled_pos(0.85)]
    pelvis_radii = [scaled_size(0.16), scaled_size(0.10), scaled_size(0.10)]
    pelvis_v, pelvis_f = create_ellipsoid(pelvis_center, pelvis_radii, u_segments=14, v_segments=10)
    all_vertices.extend(pelvis_v)
    all_faces.extend(pelvis_f + vertex_offset)
    vertex_offset += len(pelvis_v)
    
    # 12. 左大腿
    left_thigh = [scaled_size(0.09), 0, scaled_pos(0.58)]
    left_th_v, left_th_f = create_cylinder(left_thigh, scaled_size(0.07), scaled_size(0.42), segments=10)
    all_vertices.extend(left_th_v)
    all_faces.extend(left_th_f + vertex_offset)
    vertex_offset += len(left_th_v)
    
    # 13. 右大腿
    right_thigh = [-scaled_size(0.09), 0, scaled_pos(0.58)]
    right_th_v, right_th_f = create_cylinder(right_thigh, scaled_size(0.07), scaled_size(0.42), segments=10)
    all_vertices.extend(right_th_v)
    all_faces.extend(right_th_f + vertex_offset)
    vertex_offset += len(right_th_v)
    
    # 14. 左小腿
    left_calf = [scaled_size(0.09), 0, scaled_pos(0.25)]
    left_cf_v, left_cf_f = create_cylinder(left_calf, scaled_size(0.05), scaled_size(0.40), segments=10)
    all_vertices.extend(left_cf_v)
    all_faces.extend(left_cf_f + vertex_offset)
    vertex_offset += len(left_cf_v)
    
    # 15. 右小腿
    right_calf = [-scaled_size(0.09), 0, scaled_pos(0.25)]
    right_cf_v, right_cf_f = create_cylinder(right_calf, scaled_size(0.05), scaled_size(0.40), segments=10)
    all_vertices.extend(right_cf_v)
    all_faces.extend(right_cf_f + vertex_offset)
    vertex_offset += len(right_cf_v)
    
    # 16. 左耳
    left_ear_center = [0.10 * head_scale, 0, scaled_pos(1.63)]
    left_ear_radii = [0.015 * head_scale, 0.01 * head_scale, 0.03 * head_scale]
    left_ear_v, left_ear_f = create_ellipsoid(left_ear_center, left_ear_radii, u_segments=6, v_segments=4)
    all_vertices.extend(left_ear_v)
    all_faces.extend(left_ear_f + vertex_offset)
    vertex_offset += len(left_ear_v)
    
    # 17. 右耳
    right_ear_center = [-0.10 * head_scale, 0, scaled_pos(1.63)]
    right_ear_radii = [0.015 * head_scale, 0.01 * head_scale, 0.03 * head_scale]
    right_ear_v, right_ear_f = create_ellipsoid(right_ear_center, right_ear_radii, u_segments=6, v_segments=4)
    all_vertices.extend(right_ear_v)
    all_faces.extend(right_ear_f + vertex_offset)
    vertex_offset += len(right_ear_v)
    
    all_vertices = np.array(all_vertices)
    all_faces = np.array(all_faces)
    
    # 调整z坐标，使脚底正好在z=0
    min_z = all_vertices[:, 2].min()
    all_vertices[:, 2] -= min_z
    actual_height = all_vertices[:, 2].max()
    
    # 写入OBJ文件
    with open(output_path, 'w') as f:
        f.write(f"# Low-poly Human Body Model - {name}\n")
        f.write("# Generated for MuJoCo simulation\n")
        f.write(f"# Vertices: {len(all_vertices)}, Faces: {len(all_faces)}\n")
        f.write(f"# Target Height: {height:.2f}m, Actual: {actual_height:.2f}m\n")
        f.write("# Feet at z=0, Face facing +Y direction\n\n")
        
        for v in all_vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        
        f.write("\n")
        
        for face in all_faces:
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    
    print(f"✅ {name} 模型已生成: {output_path}")
    print(f"   目标身高: {height:.2f}m, 实际: {actual_height:.2f}m")
    print(f"   顶点数: {len(all_vertices)}")
    print(f"   面数: {len(all_faces)}")
    
    return output_path


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_dir = os.path.join(script_dir, "human_meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    
    # 生成人头模型
    head_path = os.path.join(mesh_dir, "human_head.obj")
    create_human_head_model(head_path)
    
    # 生成标准全身模型 (1.7米)
    body_path = os.path.join(mesh_dir, "human_body.obj")
    create_full_body_model(body_path)
    
    # 生成高个子模型 (1.85米)
    tall_path = os.path.join(mesh_dir, "human_tall.obj")
    create_scaled_body_model(tall_path, height=1.85, name="高个子")
    
    # 生成矮个子模型 (1.55米)
    short_path = os.path.join(mesh_dir, "human_short.obj")
    create_scaled_body_model(short_path, height=1.55, name="矮个子")
    
    # 生成儿童/小个子模型 (1.30米)
    child_path = os.path.join(mesh_dir, "human_child.obj")
    create_scaled_body_model(child_path, height=1.30, name="儿童")
    
    print("\n📁 模型文件位置:")
    print(f"   头部模型: {head_path}")
    print(f"   标准全身模型 (1.70m): {body_path}")
    print(f"   高个子模型 (1.85m): {tall_path}")
    print(f"   矮个子模型 (1.55m): {short_path}")
    print(f"   儿童模型 (1.30m): {child_path}")

