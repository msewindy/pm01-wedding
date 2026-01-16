"""
语音资源配置
"""

from typing import Dict

# 默认 TTS 配置
DEFAULT_TTS_CONFIG = {
    "engine": "edge",
    "voice": "zh-CN-YunxiaNeural",  # 默认声音：云下，男生
    "rate": "+0%",                    # 默认语速
    "volume": "+0%",                  # 默认音量
}

# 语音库定义 (ID -> Text)
SPEECH_LIBRARY: Dict[str, str] = {
    # ==================== IDLE 待机 ====================
    "idle_greeting": "你好呀，欢迎来到婚礼现场！",
    "idle_welcome": "欢迎欢迎，今天是个好日子！",
    "idle_random_1": "我是婚礼小记者，欢迎您对新人送出祝福",
    "idle_random_2": "需要合影吗？摆个Pose 就可以啦！",
    
    # ==================== 问候 ====================
    "greeting_hello": "你好！很高兴见到你！",
    "greeting_nice": "哇，你今天真好看！",
    "greeting_bride": "恭喜恭喜！新娘子真美！",
    "greeting_groom": "恭喜恭喜！新郎官好帅！",
    
    # ==================== 合影 ====================
    "photo_ready": "准备好了吗？",
    "photo_countdown_3": "三！",
    "photo_countdown_2": "二！",
    "photo_countdown_1": "一！",
    "photo_cheese": "茄子！",
    "photo_done": "拍好啦！照片很棒哦！",
    "photo_again": "再来一张吗？",
    
    # ==================== 送别 ====================

    
    # ==================== 采访 ====================
    "interview_start": "让我来采访一下！",
    "interview_question_relation": "请问你和新人是什么关系呀？",
    "interview_question_2": "有什么祝福想对新人说的吗？",
    "interview_question_3": "来分享一个你和新人的小故事吧！",
    "interview_thanks": "谢谢你的祝福！",
    
    # ==================== 系统 ====================
    "system_error": "抱歉，出了点小问题。",
    "system_too_close": "请稍微后退一点点哦。",
    "system_starting": "系统启动中，请稍等。",
}
