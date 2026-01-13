"""
机器人硬件配置
"""

# 关节总数
NUM_JOINTS = 24

# 关键关节索引
WAIST_YAW_IDX = 12
HEAD_YAW_IDX = 23

# 关节分组索引
JOINTS_LEG = list(range(0, 12))       # J00-J11: 腿部
JOINTS_ARM_ALL = list(range(12, 23))  # J12-J22: 腰部+手臂 (注意: J12是腰)
# 右臂: J13=R_SHOULDER_PITCH, J14=R_SHOULDER_ROLL, J15=R_ELBOW_YAW, J16=R_ELBOW_PITCH
JOINTS_ARM_RIGHT = [13, 14, 15, 16]

# 关节限位 (弧度)
LIMITS = {
    "WAIST_YAW": 1.0472,  # ±60°
    "HEAD_YAW": 0.5236,   # ±30°
}

# 默认 PD 控制参数
DEFAULT_PD_PARAMS = {
    "LEG":  {"kp": 400.0, "kd": 5.0}, # 高刚度保持站立
    "BODY": {"kp": 200.0, "kd": 3.0}, # 腰/头中等刚度
    "ARM":  {"kp": 100.0, "kd": 2.0}, # 手臂低刚度自然动作
}

# 默认运动平滑参数
DEFAULT_SMOOTH_PARAMS = {
    "HEAD":  {"vel": 1.5, "acc": 5.0},
    "WAIST": {"vel": 1.0, "acc": 4.0},
    "ARM":   {"vel": 1.0, "acc": 2.0},
}
