"""
动作基元资源 (Poses & Motions)
"""

# ==================== 静态姿态 (Poses) ====================
# 定义特定关节的目标角度 (弧度)
# Key: 姿态名称
# Value: {joint_idx: angle, ...}

POSES = {
    # 自然垂下 (Hands down)
    "neutral": {
        13: 0.0, 14: 0.0, 15: 0.0, 16: 0.0, 17: 0.0, # Left Arm
        18: 0.0, 19: 0.0, 20: 0.0, 21: 0.0, 22: 0.0  # Right Arm
    },
    
    # 举麦克风 (左手 - J13-J17)
    "mic_hold": {
        13: -0.5, # L_SHOULDER_PITCH (抬起)
        14: 0.2,  # L_SHOULDER_ROLL (外展 - 注意符号可能需反转)
        15: 0.0,  # L_SHOULDER_YAW
        16: -1.2, # L_ELBOW_PITCH (弯曲)
        17: 0.0   # L_ELBOW_YAW
    },
    
    # 挥手准备 (右手 - J18-J22)
    "wave_start": {
        18: -1.7, # R_SHOULDER_PITCH (高举)
        19: 0.0,  # R_SHOULDER_ROLL
        20: 0.0,  # R_SHOULDER_YAW
        21: -0.6, # R_ELBOW_PITCH (微弯)
        22: 0.0   # R_ELBOW_YAW
    },

    # ==================== 新增姿态 ====================

    # 双手胸前比心 (Heart Chest)
    "heart_chest": {
        13: -0.5,  # L_SHOULDER_PITCH (抬起)
        14: 0.1,   # L_SHOULDER_ROLL (内收)
        15: -0.5,   # L_SHOULDER_YAW (向内转)
        16: -1.5,  # L_ELBOW_PITCH (弯曲 90度+)
        17: 0.0,
        
        18: -0.5,  # R_SHOULDER_PITCH
        19: -0.1,  # R_SHOULDER_ROLL
        20: 0.5,  # R_SHOULDER_YAW
        21: -1.5,  # R_ELBOW_PITCH
        22: 0.0
    },

    # 双手头顶比心 (Heart Head)
    "heart_head": {
        13: -2.0,  # L_SHOULDER_PITCH (高举)
        14: 0.8,   # L_SHOULDER_ROLL (张开后内扣)
        15: 0.0,
        16: -2.0,  # L_ELBOW_PITCH (大弯曲)
        17: 0.0,
        
        18: -2.0,  # R_SHOULDER_PITCH
        19: -0.8,  # R_SHOULDER_ROLL
        20: 0.0,
        21: -2.0,  # R_ELBOW_PITCH
        22: 0.0
    },

    # 双手叉腰 (Akimbo)
    "akimbo": {
        13: -0.2,  # L_SHOULDER_PITCH
        14: 0.8,   # L_SHOULDER_ROLL (张开)
        15: 0.0,
        16: -1.2,  # L_ELBOW_PITCH (手肘弯曲向外)
        17: 0.0,
        
        18: -0.2,  # R_SHOULDER_PITCH
        19: -0.8,  # R_SHOULDER_ROLL
        20: 0.0,
        21: -1.2,  # R_ELBOW_PITCH
        22: 0.0
    }
}


# ==================== 动态策略参数 (Motion Params) ====================
# 定义持续运动的参数
# type: 策略类型 (sway, wave, scan, track, multi_sine)

MOTIONS = {
    # 待机摆动 (Sway)
    "sway_normal": {
        "type": "sway",
        "period": 3.0,
        "head_amp": 0.2,   # 头部摆动幅度
        "waist_amp": 0.1,  # 腰部摆动幅度
        "phase_diff": 0.5  # 相位差
    },
    
    # 挥手 (Right Wave - Single Joint Legacy)
    "wave_greet": {
        "type": "wave",
        "period": 1.0,     # 挥手周期
        "arm_amp": 0.3,    # 手臂摆动幅度
        "joint_idx": 19    # R_SHOULDER_ROLL (右手Roll)
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
    },

    # ==================== 新增动态 (Multi-Sine) ====================
    # 呼吸 (Breath) - 肩膀微动
    "breath": {
        "type": "multi_sine",
        "joints": [
            {"id": 13, "amp": 0.05, "period": 4.0, "phase": 0.0}, # L_PITCH
            {"id": 18, "amp": 0.05, "period": 4.0, "phase": 0.0}, # R_PITCH
            # 也可以加一点手臂张开 (Roll)
            {"id": 14, "amp": 0.02, "period": 4.0, "phase": 0.5},
            {"id": 19, "amp": -0.02, "period": 4.0, "phase": 0.5},
        ]
    },

}
