"""
IDLE 状态（空闲）

行为：
- 执行待机动作包（idle_normal 或 greeting）
- 随机切换动作
- 检测到稳定人脸后切换至 SEARCH
"""

import math
import random
import time
from typing import TYPE_CHECKING, Optional, List

from ..enums import WeddingStateName
from ..wedding_state import WeddingState

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM
    from ...perception import FaceInfo


class IdleState(WeddingState):
    """
    空闲状态
    """
    
    # ========== 配置参数 ==========
    
    # 动作切换参数
    SWITCH_INTERVAL_MIN = 10.0
    SWITCH_INTERVAL_MAX = 20.0
    GREETING_PROBABILITY = 0.3    # 切换时执行打招呼的动作概率
    
    # 目标检测参数
    FACE_CONFIRM_TIME = 0.5       # 候选确认时间（秒）
    FACE_LOST_TOLERANCE = 0.2     # 候选丢失容忍时间（秒）
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.IDLE)
        
        # 切换控制
        self._next_switch_time = 0.0
        
        # 候选目标状态
        self._candidate_face: Optional['FaceInfo'] = None
        self._candidate_start_time: float = 0.0
        self._candidate_last_seen: float = 0.0
        
    def on_enter(self) -> None:
        self.log("Entering IDLE state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 重置候选
        self._reset_candidate()
        
        # 初始动作：普通闲聊/摆动
        self.execute_action("idle_normal")
        self._schedule_next_switch()
        
    def _reset_candidate(self) -> None:
        """重置候选目标状态"""
        self._candidate_face = None
        self._candidate_start_time = 0.0
        self._candidate_last_seen = 0.0
        self.fsm.data.perception.target_face_id = None
        
    def _schedule_next_switch(self):
        """计划下一次动作切换"""
        interval = random.uniform(self.SWITCH_INTERVAL_MIN, self.SWITCH_INTERVAL_MAX)
        self._next_switch_time = self.fsm.get_current_time() + interval
        
    def run(self) -> None:
        """执行待机逻辑"""
        current_time = self.fsm.get_current_time()
        
        # 1. 动作切换逻辑
        if current_time >= self._next_switch_time:
            # 随机决定是普通摆动还是打招呼
            if random.random() < self.GREETING_PROBABILITY:
                self.execute_action("greeting")
                self.log("Switching to action: greeting")
            else:
                self.execute_action("idle_normal")
                self.log("Switching to action: idle_normal")
            
            self._schedule_next_switch()
        
        # 2. 目标检测逻辑
        self._update_candidate(current_time)
        
        self.iter_count += 1
        
    def _update_candidate(self, current_time: float):
        """更新候选检测"""
        perception = self.fsm.data.perception
        faces = perception.faces
        
        # 1. 如果没有候选，寻找新候选
        if self._candidate_face is None:
            # 优先选择画面中心的正脸
            best_face = self.select_best_face(faces, center_x=0.5, center_y=0.5)
            if best_face:
                self._candidate_face = best_face
                self._candidate_start_time = current_time
                self._candidate_last_seen = current_time
                self.fsm.data.perception.target_face_id = best_face.face_id
                self.log(f"New candidate found: {best_face.face_id}")
            return

        # 2. 如果有候选，进行匹配跟踪
        matched = self.find_matching_face(faces, self._candidate_face)
        
        if matched and matched.is_frontal:
            # 匹配成功
            self._candidate_face = matched
            self._candidate_last_seen = current_time
            self.fsm.data.perception.target_face_id = matched.face_id
        else:
            # 丢失
            lost_duration = current_time - self._candidate_last_seen
            if lost_duration > self.FACE_LOST_TOLERANCE:
                self.log(f"Candidate lost (duration: {lost_duration:.1f}s)")
                self._reset_candidate()
    
    def check_transition(self) -> WeddingStateName:
        """检查状态转换"""
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
        
        # 检查候选确认
        if self._candidate_face:
            current_time = self.fsm.get_current_time()
            confirm_duration = current_time - self._candidate_start_time
            # 排除最后一次看到后的丢失时间
            effective_duration = confirm_duration - (current_time - self._candidate_last_seen)
            
            if effective_duration >= self.FACE_CONFIRM_TIME:
                self.log(f"Candidate confirmed ({effective_duration:.2f}s). Proceeding to SEARCH.")
                
                # 保存目标信息到 LockedTarget (供 SEARCH/TRACKING 使用)
                if self._candidate_face:
                    from ...perception import LockedTarget
                    self.fsm.data.perception.locked_target = LockedTarget.from_face(self._candidate_face)
                
                return WeddingStateName.SEARCH
        
        return self.state_name

    def on_exit(self) -> None:
        self.log("Exiting IDLE state")
