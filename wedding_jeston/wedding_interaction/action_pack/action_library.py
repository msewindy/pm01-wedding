"""
动作库

预定义的动作包集合
"""

from typing import Dict, List, Optional
import random

from .action_pack import ActionPack, ActionType


class ActionLibrary:
    """
    动作库
    
    管理所有预定义的动作包
    """
    
    def __init__(self):
        self._packs: Dict[str, ActionPack] = {}
        self._type_index: Dict[ActionType, List[str]] = {}
        
        # 加载预定义动作包
        self._load_default_packs()
    
    def _load_default_packs(self) -> None:
        """加载默认动作包"""
        
        # ==================== IDLE 待机动作 ====================
        
        # 待机摆动 - 大幅度
        self.register(ActionPack(
            name="idle_sway_large",
            action_type=ActionType.IDLE_SWAY,
            head_amplitude=0.25,      # ±14° 头部
            waist_amplitude=0.15,     # ±8° 腰部
            period=3.0,               # 3秒周期
            duration=0.0,             # 持续执行
            speech_id=None,           # 无语音
        ))
        
        # 待机摆动 - 中幅度
        self.register(ActionPack(
            name="idle_sway_medium",
            action_type=ActionType.IDLE_SWAY,
            head_amplitude=0.15,      # ±8° 头部
            waist_amplitude=0.08,     # ±4° 腰部
            period=2.5,               # 2.5秒周期
            duration=0.0,
            speech_id=None,
        ))
        
        # 待机摆动 + 问候语
        self.register(ActionPack(
            name="idle_sway_with_greeting",
            action_type=ActionType.IDLE_SWAY,
            head_amplitude=0.20,      # ±11° 头部
            waist_amplitude=0.10,     # ±5° 腰部
            period=2.5,
            duration=0.0,
            speech_id="idle_greeting",
            speech_text="你好呀，欢迎来到婚礼现场！",
            speech_delay=0.5,         # 延迟 0.5 秒播放
        ))
        
        # 待机摆动 + 问候语1（欢迎）
        self.register(ActionPack(
            name="idle_sway_welcome",
            action_type=ActionType.IDLE_SWAY,
            head_amplitude=0.22,
            waist_amplitude=0.12,
            period=2.8,
            duration=0.0,
            speech_id="idle_welcome",  # "欢迎欢迎，今天是个好日子！"
            speech_delay=1.0,
        ))
        
        # 待机摆动 + 问候语2（帮助）
        self.register(ActionPack(
            name="idle_sway_help",
            action_type=ActionType.IDLE_SWAY,
            head_amplitude=0.18,
            waist_amplitude=0.10,
            period=2.5,
            duration=0.0,
            speech_id="idle_random_1",  # "有什么可以帮你的吗？"
            speech_delay=0.8,
        ))
        
        # 待机摆动 + 问候语3（合影引导）
        self.register(ActionPack(
            name="idle_sway_photo_hint",
            action_type=ActionType.IDLE_SWAY,
            head_amplitude=0.20,
            waist_amplitude=0.11,
            period=2.6,
            duration=0.0,
            speech_id="idle_random_2",  # "需要合影吗？摆个 Pose 就可以啦！"
            speech_delay=1.2,
        ))
        
        # ==================== 问候动作 ====================
        
        # 挥手问候
        self.register(ActionPack(
            name="greeting_wave",
            action_type=ActionType.GREETING_WAVE,
            head_amplitude=0.1,
            waist_amplitude=0.05,
            period=1.0,
            duration=3.0,             # 3秒后结束
            speech_id="greeting",
            speech_text="你好！很高兴见到你！",
            speech_delay=0.2,
        ))
        
        # ==================== 合影动作 ====================
        
        # 比心 Pose
        self.register(ActionPack(
            name="pose_heart",
            action_type=ActionType.POSE_HEART,
            head_amplitude=0.0,       # 保持不动
            waist_amplitude=0.0,
            period=0.0,
            duration=5.0,
            speech_id="photo_ready",
            speech_text="准备好了，三、二、一！",
            speech_delay=0.5,
        ))
        
        # V 字 Pose
        self.register(ActionPack(
            name="pose_v_sign",
            action_type=ActionType.POSE_V_SIGN,
            head_amplitude=0.0,
            waist_amplitude=0.0,
            period=0.0,
            duration=5.0,
            speech_id="photo_v",
            speech_text="耶！茄子！",
            speech_delay=0.5,
        ))
        
        # ==================== 送别动作 ====================
        
        # 挥手告别
        self.register(ActionPack(
            name="farewell_wave",
            action_type=ActionType.FAREWELL_WAVE,
            head_amplitude=0.08,
            waist_amplitude=0.05,
            period=1.5,
            duration=3.0,
            speech_id="farewell",
            speech_text="再见！祝你们幸福美满！",
            speech_delay=0.2,
        ))
        
        # ==================== 基础动作 ====================
        
        # 向左看
        self.register(ActionPack(
            name="look_left",
            action_type=ActionType.LOOK_LEFT,
            head_amplitude=0.3,       # 固定偏移
            waist_amplitude=0.1,
            period=0.0,               # 非周期性
            duration=2.0,
            speech_id=None,
        ))
        
        # 向右看
        self.register(ActionPack(
            name="look_right",
            action_type=ActionType.LOOK_RIGHT,
            head_amplitude=-0.3,      # 负值表示右转
            waist_amplitude=-0.1,
            period=0.0,
            duration=2.0,
            speech_id=None,
        ))
        
        # 中立位
        self.register(ActionPack(
            name="neutral",
            action_type=ActionType.NEUTRAL,
            head_amplitude=0.0,
            waist_amplitude=0.0,
            period=0.0,
            duration=0.0,
            speech_id=None,
        ))
    
    def register(self, pack: ActionPack) -> None:
        """注册动作包"""
        self._packs[pack.name] = pack
        
        # 更新类型索引
        if pack.action_type not in self._type_index:
            self._type_index[pack.action_type] = []
        self._type_index[pack.action_type].append(pack.name)
    
    def get(self, name: str) -> Optional[ActionPack]:
        """根据名称获取动作包"""
        return self._packs.get(name)
    
    def get_by_type(self, action_type: ActionType) -> List[ActionPack]:
        """根据类型获取所有动作包"""
        names = self._type_index.get(action_type, [])
        return [self._packs[name] for name in names]
    
    def get_random(self, action_type: ActionType) -> Optional[ActionPack]:
        """随机获取指定类型的动作包"""
        packs = self.get_by_type(action_type)
        if not packs:
            return None
        return random.choice(packs)
    
    def list_all(self) -> List[str]:
        """列出所有动作包名称"""
        return list(self._packs.keys())
    
    def list_by_type(self, action_type: ActionType) -> List[str]:
        """列出指定类型的动作包名称"""
        return self._type_index.get(action_type, [])


# 全局动作库实例
_default_library: Optional[ActionLibrary] = None


def get_action_library() -> ActionLibrary:
    """获取默认动作库"""
    global _default_library
    if _default_library is None:
        _default_library = ActionLibrary()
    return _default_library

