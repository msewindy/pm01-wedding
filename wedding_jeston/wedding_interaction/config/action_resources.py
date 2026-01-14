"""
综合动作包 (Action Packs)
"""

# 动作包定义
# Key: 动作名称
# Value: {
#   "pose": 静态姿态名称 (引用 motion_resources.POSES)
#   "motion": 动态名称 (引用 motion_resources.MOTIONS)
#   "speech_group": 语音ID列表 (可选, 随机播放)
# }

ACTION_PACKS = {
    # ==================== IDLE ====================
    "idle_normal": {
        "pose": "neutral",
        "motion_strategy": "sway_normal",
        "speech_group": [
            "idle_random_1",
            "idle_greeting"
        ],
        "speech_config": {
            "mode": "periodic",
            "interval_min": 10.0,
            "interval_max": 20.0,
            "delay": 1.0,
            "probability": 0.6 # 60% 概率播放语音，避免太吵
        }
    },
    
    # 静态观察 (Observe Phase)
    "idle_observe": {
        "pose": "neutral",
        "motion_strategy": None,  # 无运动，保持静止
        "speech_group": [],       # 不说话
        "speech_config": None
    },
    
    "greeting": {
        "pose": "wave_start",
        "motion_strategy": "wave_greet",
        "speech_group": [
            "greeting_hello", 
            "greeting_nice",
            "greeting_bride",
            "greeting_groom"
        ],
        "speech_config": {
            "mode": "once",
            "delay": 0.5
        },
        "duration": 3.5 # 动作持续时间 (wav + speech)
    },
    
    # ==================== SEARCH ====================
    "search_scan": {
        "pose": "neutral",
        "motion_strategy": "scan_search",
        "speech_group": [],
        "speech_config": None
    },
    
    # ==================== TRACKING ====================
    "tracking_active": {
        "pose": "neutral",
        "motion_strategy": "track_face",
        "speech_group": [
            "greeting_hello" # 示例: 可以在这里添加一些跟踪时的随机语音
        ],
        "speech_config": {
            "mode": "periodic",
            "interval_min": 15.0,
            "interval_max": 25.0,
            "delay": 2.0
        }
    },
    
    # ==================== INTERVIEW ====================
    "interview_hold": {
        "pose": "mic_hold",
        "motion_strategy": "track_face",
        "speech_group": []
    },
    
    # ==================== SYSTEM ====================
    "reset": {
        "pose": "neutral",
        "motion_strategy": None, # 停止动态
        "speech_group": []
    }
}
