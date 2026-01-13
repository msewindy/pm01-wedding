"""
动作基元资源 (Poses & Motions)
"""

# ==================== 静态姿态 (Poses) ====================
# 定义特定关节的目标角度 (弧度)
# Key: 姿态名称
# Value: {joint_idx: angle, ...}

POSES = {
    # 自然垂下
    "neutral": {
        13: 0.0, 14: 0.0, 15: 0.0, 16: 0.0
    },
    
    # 举麦克风 (右手)
    "mic_hold": {
        13: -0.5, # R_SHOULDER_PITCH (抬起)
        14: -0.2, # R_SHOULDER_ROLL (外展)
        15: 0.0,  # R_ELBOW_YAW
        16: -1.2  # R_ELBOW_PITCH (弯曲)
    },
    
    # 挥手准备 (举起手臂)
    "wave_start": {
        13: -1.5, # R_SHOULDER_PITCH (高举)
        14: 0.0,
        15: 0.0,
        16: -0.5  # R_ELBOW_PITCH (微弯)
    }
}


# ==================== 动态策略参数 (Motion Params) ====================
# 定义持续运动的参数
# type: 策略类型 (sway, wave, scan, track)

MOTIONS = {
    # 待机摆动 (Sway)
    "sway_normal": {
        "type": "sway",
        "period": 3.0,
        "head_amp": 0.2,   # 头部摆动幅度
        "waist_amp": 0.1,  # 腰部摆动幅度
        "phase_diff": 0.5  # 相位差
    },
    
    # 挥手 (Wave)
    "wave_greet": {
        "type": "wave",
        "period": 1.0,     # 挥手周期
        "arm_amp": 0.3,    # 手臂摆动幅度 (SHOULDER_ROLL or ELBOW)
        "joint_idx": 14    # 主要挥动关节 (e.g. 14=Roll)
    },
    
    # 环境扫描 (Scan)
    "scan_search": {
        "type": "scan",
        "period": 8.0,     # 扫描周期 (慢速)
        "range": 0.8,      # 扫描范围 (比例 0~1)
        "function": "sine" # 正弦扫描
    },
    
    # PID 跟随 (Track)
    # 这只是参数，具体的计算在 FSM 中进行
    "track_face": {
        "type": "track",
        "smooth": 0.2,
        "dead_zone": 0.05
    }
}
