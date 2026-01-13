"""
人脸跟随辅助工具

提供基于 PID 控制的跟随功能，供各状态使用

核心原理：
- 从图像像素偏差计算目标对象和当前look_at方向的角度差
- 当前look_at的方向就是图像的中心线
- 使用相机成像模型：x_image = fx * X_real / Z_real + cx
- 角度 = atan((x_image - cx) / fx)，不需要Z_real
- 用这个角度进行PID控制，驱动机器人关节旋转
"""

import math
from typing import Tuple, Optional


class FaceFollowingHelper:
    """
    人脸跟随辅助工具
    
    提供基于 PID 控制的跟随功能，根据目标偏离图像中心的角度差计算旋转速度
    
    核心原理：
    1. 从图像像素偏差计算角度：θ = atan((x_image - cx) / fx)
    2. 使用角度误差进行PID控制
    3. 将角度变化转换为look_at变化
    
    PID 控制参数：
    """
    # ========== PID 控制配置 ==========
    # PID 控制参数（基于角度误差）
    # 减小 Kp 以防止过冲和震荡 (原 0.1)
    Kp = 0.05  # 比例系数（减小以平滑运动）
    # 说明：Kp=0.05 表示角度误差 0.1rad 时，单帧角度变化 0.005rad
    #       如果控制周期为 0.02s，则角速度 = 0.25 rad/s
    
    Ki = 0.0  # 积分系数（可选，用于消除稳态误差）
    
    Kd = 0.01  # 微分系数（增加阻尼，防止震荡）
    
    # 方向反转标志（如果机器人旋转方向与目标移动方向相反，设置为 True）
    REVERSE_DIRECTION = True
    
    
    # 最大单帧角度变化（防止过大变化导致抖动）
    # 减小以限制最大速度
    MAX_ANGLE_CHANGE_PER_FRAME = 0.02  # rad，约1.1°

    
    # 接近目标时的减速配置
    APPROACH_DAMPING_ENABLED = True  # 是否启用接近目标减速
    APPROACH_ANGLE_THRESHOLD = 0.10  # 接近目标角度阈值（rad，约5.7°，增大阈值，更早开始减速）
    APPROACH_DAMPING = 0.3  # 接近目标时的阻尼系数（减小系数，增强减速效果）
    
    @staticmethod
    def pixel_to_angle(pixel_offset: float, fx: float) -> float:
        """
        从像素偏差计算角度
        
        相机模型：x_image = fx * X_real / Z_real + cx
        角度：θ = atan(X_real / Z_real) = atan((x_image - cx) / fx)
        
        注意：不需要Z_real，角度只依赖于像素偏差和焦距
        
        Args:
            pixel_offset: 像素偏差（x_image - cx），单位：像素
            fx: 焦距（像素），从camera_info获取
        
        Returns:
            角度（弧度）
        """
        if abs(fx) < 1e-6:  # 避免除零
            return 0.0
        return math.atan(pixel_offset / fx)
    
    @staticmethod
    def normalized_to_pixel_offset(normalized_offset: float, image_width: float) -> float:
        """
        将归一化偏差转换为像素偏差
        
        Args:
            normalized_offset: 归一化偏差（0-1范围，中心为0.5，偏差 = target - 0.5）
            image_width: 图像宽度（像素）
        
        Returns:
            像素偏差（像素）
        """
        return normalized_offset * image_width
    
    
    @staticmethod
    def pid_follow(target_x: float, target_y: float,
                   current_look_at_x: float, current_look_at_y: float,
                   fx: float, fy: float,
                   image_width: float, image_height: float,
                   integral_x: float = 0.0, integral_y: float = 0.0,
                   last_error_x: float = 0.0, last_error_y: float = 0.0,
                   dt: float = 0.02,
                   logger = None,
                   log_prefix: str = "") -> Tuple[float, float, float, float]:
        """
        基于 PID 控制的跟随（基于角度误差）
        
        核心思想：
        1. 从图像像素偏差计算角度：θ = atan((x_image - cx) / fx)
        2. 使用角度误差进行PID控制
        3. 将角度变化转换为look_at变化
        
        关键优势：
        - 不需要估计Z_real（深度），角度计算只依赖像素偏差和焦距
        - 物理意义清晰：直接控制旋转角度
        - 不需要建立图像偏差到关节运动范围的映射关系
        
        过冲预防机制：
        - 过冲检测：检测当前角度是否已经超过目标角度，如果过冲则应用阻尼
        - 接近目标减速：当角度误差较小时提前减速，避免过冲
        
        Args:
            target_x, target_y: 目标在图像中的位置（归一化坐标 0-1）
            current_look_at_x, current_look_at_y: 当前 look_at 位置（归一化坐标 0-1）
            fx, fy: 焦距（像素），从camera_info获取
            image_width, image_height: 图像尺寸（像素）
            integral_x, integral_y: 积分项（需要外部维护，初始为 0.0）
            last_error_x, last_error_y: 上一帧角度误差（需要外部维护，初始为 0.0）
            dt: 控制周期（秒），应根据实际FSM频率计算
                 建议：25Hz -> 0.04s, 30Hz -> 0.033s
                 注意：应匹配或低于图像采集频率，避免PID控制发散
            logger: 日志记录器（可选），用于输出详细日志
            log_prefix: 日志前缀（可选），用于区分不同调用来源
        
        Returns:
            (new_look_at_x, new_look_at_y, new_integral_x, new_integral_y)
        """
        # 日志辅助函数
        # 使用 info 级别确保日志能输出（debug 级别可能被过滤）
        def log_debug(msg: str) -> None:
            if logger is not None:
                prefix = f"{log_prefix} " if log_prefix else ""
                full_msg = f"{prefix}{msg}"
                # 优先使用 info 级别，确保日志能输出
                if hasattr(logger, 'info'):
                    logger.info(full_msg)
                elif hasattr(logger, 'debug'):
                    logger.debug(full_msg)
        
        # 1. 计算归一化偏差（相对于图像中心 0.5）
        normalized_offset_x = target_x - 0.5  # 偏差：-0.5 到 +0.5
        log_debug(f"[PID输入] target=({target_x:.4f}, {target_y:.4f}), "
                 f"current_look_at=({current_look_at_x:.4f}, {current_look_at_y:.4f}), "
                 f"normalized_offset_x={normalized_offset_x:.4f}")
        
        # 2. 转换为像素偏差
        pixel_offset_x = FaceFollowingHelper.normalized_to_pixel_offset(
            normalized_offset_x, image_width
        )
        #log_debug(f"[像素转换] image_width={image_width:.1f}, pixel_offset_x={pixel_offset_x:.2f}px")
        
        # 3. 计算角度误差（弧度）
        angle_error_x = FaceFollowingHelper.pixel_to_angle(pixel_offset_x, fx)
        angle_error_x_deg = math.degrees(angle_error_x)
        #log_debug(f"[角度计算] fx={fx:.1f}, angle_error_x={angle_error_x:.4f}rad ({angle_error_x_deg:.2f}°)")
        
        #当偏差角度小于0.02rad/约1.1度时，认为目标已经接近中心，此时停止跟随
        # 减小阈值，避免在快速移动时过早停止
        if abs(angle_error_x) < 0.02:
            log_debug(f"[停止跟随] 角度误差 {angle_error_x:.4f}rad < 0.02rad，目标已接近中心")
            return (current_look_at_x, 0.5, integral_x, 0.0)
        
        # 4. P 控制：比例项（角速度与角度误差成正比）
        p_x = FaceFollowingHelper.Kp * angle_error_x
        #log_debug(f"[P控制] Kp={FaceFollowingHelper.Kp}, p_x={p_x:.6f}rad")
        
        # 5. I 控制：积分项（可选，用于消除稳态误差）
        integral_x += angle_error_x * dt
        i_x = FaceFollowingHelper.Ki * integral_x
        #log_debug(f"[I控制] Ki={FaceFollowingHelper.Ki}, integral_x={integral_x:.6f}, i_x={i_x:.6f}rad")
        
        # 6. D 控制：微分项（可选，用于防止超调）
        derivative_x = (angle_error_x - last_error_x) / dt if dt > 0 else 0.0
        d_x = FaceFollowingHelper.Kd * derivative_x
        #log_debug(f"[D控制] Kd={FaceFollowingHelper.Kd}, derivative_x={derivative_x:.6f}rad/s, "
        #         f"d_x={d_x:.6f}rad, last_error_x={last_error_x:.6f}rad")
        
        # 7. 计算 PID 输出（角度变化量，单位：弧度）
        angle_change_x_before_damping = p_x + i_x + d_x
        angle_change_x = angle_change_x_before_damping
        #log_debug(f"[PID输出] angle_change_x={angle_change_x:.6f}rad "
        #        f"(P={p_x:.6f} + I={i_x:.6f} + D={d_x:.6f})")
        
        # 8. 方向反转（机器人旋转方向与目标移动方向相反，必须取反）
        if FaceFollowingHelper.REVERSE_DIRECTION:
            angle_change_x = -angle_change_x
        #    log_debug(f"[方向反转] REVERSE_DIRECTION=True, angle_change_x={angle_change_x:.6f}rad")
        
        # 9. 【接近目标减速】
        # 当角度误差较小时，提前减速，避免过冲
        if FaceFollowingHelper.APPROACH_DAMPING_ENABLED:
            if abs(angle_error_x) < FaceFollowingHelper.APPROACH_ANGLE_THRESHOLD:
                angle_change_x_before = angle_change_x
                angle_change_x *= FaceFollowingHelper.APPROACH_DAMPING
                log_debug(f"[接近减速] 角度误差 {abs(angle_error_x):.4f}rad < "
                         f"阈值 {FaceFollowingHelper.APPROACH_ANGLE_THRESHOLD:.4f}rad, "
                         f"阻尼={FaceFollowingHelper.APPROACH_DAMPING}, "
                         f"angle_change_x: {angle_change_x_before:.6f} -> {angle_change_x:.6f}rad")
        
        # 10. 限制角度变化量（防止过大变化导致抖动）
        max_angle_change = FaceFollowingHelper.MAX_ANGLE_CHANGE_PER_FRAME
        angle_change_x_before_limit = angle_change_x
        angle_change_x = max(-max_angle_change, min(max_angle_change, angle_change_x))

        
        # 11. 将角度变化转换为 look_at 变化
        # 关键：角度变化（弧度）需要转换为归一化坐标的变化
        # 假设关节运动范围约为 0.48 rad（27.5°），对应 look_at 范围 0-1
        # 因此：look_at_change = angle_change / 0.48
        # 但这里直接使用角度变化作为 look_at 变化，需要根据实际情况调整
        # 暂时使用一个缩放因子：如果角度变化 0.01 rad，对应 look_at 变化约 0.02（2%）
        # 更合理的做法：根据实际关节运动范围计算
        # 假设最大角度范围是 ±0.24 rad（约 ±13.75°），对应 look_at 从 0 到 1
        # 因此：look_at_change = angle_change / (2 * 0.24) = angle_change / 0.48
        JOINT_ANGLE_RANGE = 1.5708  # 关节运动范围（弧度），对应 look_at 0-1
        look_at_change_x = angle_change_x / JOINT_ANGLE_RANGE

        
        # 12. 更新 look_at
        new_look_at_x = current_look_at_x + look_at_change_x
        
        # 13. 限制范围（0-1）
        new_look_at_x = max(0.0, min(1.0, new_look_at_x))

        
        # 13. 最终结果日志
        log_debug(f"[PID结果] new_look_at=({new_look_at_x:.4f}, 0.5), "
                 f"new_integral_x={integral_x:.6f}, angle_error_x={angle_error_x:.6f}rad")
        
        return (new_look_at_x, 0.5, integral_x, 0.0)
