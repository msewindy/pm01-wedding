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
            "idle_greeting", 
            "idle_welcome", 
            "idle_random_1",
            "idle_random_2"
        ],
        "speech_config": {
            "mode": "periodic",
            "interval_min": 3.0,
            "interval_max": 10.0,
            "delay": 0.5
        }
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
        }
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
            "interval_min": 12.0,
            "interval_max": 25.0,
            "delay": 5.0
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
