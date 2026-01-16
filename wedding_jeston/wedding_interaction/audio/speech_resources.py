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
    "idle_random_1": "我是婚礼小记者,想对新人送出祝福,站在我前面就可以啦！",

    # ==================== 问候 (已扩充至20+条,多种风格) ====================
    "greeting_hello": "你好！很高兴见到你！",
    "greeting_nice": "哇,你今天真好看！",
    "greeting_bride": "新娘子盈霖今天真美！",
    "greeting_groom": "新郎官余豪好帅！",
    "greeting_greeting": "你好呀,欢迎来到余豪和盈霖的婚礼！一起来分享这份喜悦吧！",
    # 文学风格
    "greeting_literary_1": "佳偶天成,祝福余豪盈霖今日缔结良缘。",
    "greeting_literary_2": "欢迎莅临,共证余豪与盈霖的爱情花开。",
    "greeting_literary_3": "琴瑟和鸣,祝福这对新人永沐爱河。",
    # 热情风格
    "greeting_warm_1": "热烈欢迎！为余豪和盈霖的幸福干杯！",
    "greeting_warm_2": "太开心了！和我们一起祝福新人吧！",
    "greeting_warm_3": "欢迎来到这充满爱与欢乐的婚礼殿堂！",
    "greeting_warm_4": "祝福的掌声,送给最登对的余豪和盈霖！",
    # 幽默活泼风格
    "greeting_humorous_1": "恭喜通关人生副本,解锁‘夫妻’成就！",
    "greeting_humorous_2": "今天的糖分,由这对新人承包了！",
    "greeting_humorous_3": "欢迎吃糖！余豪盈霖的喜糖管够哦！",
    "greeting_humorous_4": "检测到幸福能量爆棚！快来沾沾喜气！",
    # 通用祝福与欢迎
    "greeting_general_1": "余豪盈霖,白头偕老,永结同心！",
    "greeting_general_2": "欢迎您,一起祝福新人携手新征程！",
    "greeting_general_3": "感谢到来,共同见证这个美好的时刻！",
    "greeting_general_4": "祝福新人,也祝您今天玩得开心！",
    "greeting_general_5": "您好！祝新人百年好合,也祝您愉快！",

    # ==================== 采访 (问题已优化,更具体) ====================
    "interview_question_relation": "欢迎,请问您是新郎余豪的亲友,还是新娘盈霖的亲友呀？",
    "interview_question_1": "今天您最想对新人说的话是什么呢？",
    "interview_question_2": "对他们未来的小家庭,您最想送上什么祝愿呢？",
    "interview_thanks": "谢谢您的祝福,祝您观礼愉快！",
    # ==================== 系统 ====================
    "system_error": "抱歉,出了点小问题。",
    "system_too_close": "请稍微后退一点点哦。",
    "system_starting": "系统启动中,请稍等。",
}
