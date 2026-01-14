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
# 右臂: J13=R_SHOULDER_PITCH... (Wait, URDF says J13 is LEFT)
# Refined Joint Map based on URDF:
# 13-17: Left Arm
# 18-22: Right Arm

JOINTS_ARM_LEFT = list(range(13, 18))  # 13, 14, 15, 16, 17
JOINTS_ARM_RIGHT = list(range(18, 23)) # 18, 19, 20, 21, 22

# 关节索引定义
L_SHOULDER_PITCH = 13
L_SHOULDER_ROLL = 14
L_SHOULDER_YAW = 15
L_ELBOW_PITCH = 16
L_ELBOW_YAW = 17

R_SHOULDER_PITCH = 18
R_SHOULDER_ROLL = 19
R_SHOULDER_YAW = 20
R_ELBOW_PITCH = 21
R_ELBOW_YAW = 22

# 关节限位 (弧度) - 基于 URDF 物理限位的 80% 安全范围
# 物理极限 * 0.8，确保所有动作在绝对安全区内
LIMITS = {
    # 躯干与头部
    "WAIST_YAW": 1.05,  # ±60° (足够转身)
    "HEAD_YAW": 0.52,   # ±30° (足够观察)
    
    # 左臂 (URDF * 0.8)
    "L_SHOULDER_PITCH_MIN": -2.37, "L_SHOULDER_PITCH_MAX": 2.23,
    "L_SHOULDER_ROLL_MIN": -0.49,  "L_SHOULDER_ROLL_MAX": 1.88,
    "L_SHOULDER_YAW_MIN": -2.09,   "L_SHOULDER_YAW_MAX": 2.09,
    "L_ELBOW_PITCH_MIN": -1.76,    "L_ELBOW_PITCH_MAX": 0.59,
    "L_ELBOW_YAW_MIN": -2.09,       "L_ELBOW_YAW_MAX": 2.09,

    # 右臂 (URDF * 0.8)
    "R_SHOULDER_PITCH_MIN": -2.37, "R_SHOULDER_PITCH_MAX": 2.23,
    "R_SHOULDER_ROLL_MIN": -1.88,  "R_SHOULDER_ROLL_MAX": 0.49,
    "R_SHOULDER_YAW_MIN": -2.09,   "R_SHOULDER_YAW_MAX": 2.09,
    "R_ELBOW_PITCH_MIN": -1.76,    "R_ELBOW_PITCH_MAX": 0.59,
    "R_ELBOW_YAW_MIN": -2.09,      "R_ELBOW_YAW_MAX": 2.09,
}

# 关节索引映射到限位键名前缀 (用于 Loop 查找)
# 注意：代码需要更新以支持 Min/Max
JOINT_LIMIT_MAP = {
    12: "WAIST_YAW",
    23: "HEAD_YAW",
    
    13: "L_SHOULDER_PITCH",
    14: "L_SHOULDER_ROLL",
    15: "L_SHOULDER_YAW",
    16: "L_ELBOW_PITCH",
    17: "L_ELBOW_YAW",
    
    18: "R_SHOULDER_PITCH",
    19: "R_SHOULDER_ROLL",
    20: "R_SHOULDER_YAW",
    21: "R_ELBOW_PITCH",
    22: "R_ELBOW_YAW",
}


