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
from ...config.action_resources import ACTION_PACKS

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM
    from ...perception import FaceInfo


class IdleState(WeddingState):
    """
    空闲状态
    """
    
    # ========== 配置参数 ==========
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.IDLE)
        
        # Load params from config
        config = self.fsm.data.config or {}
        
        # 动作切换参数
        self.switch_interval_min = config.get('idle_switch_interval_min', 10.0)
        self.switch_interval_max = config.get('idle_switch_interval_max', 20.0)
        self.greeting_probability = config.get('idle_greeting_probability', 0.3)
        self.observe_duration_min = config.get('idle_observe_duration_min', 3.0)
        self.observe_duration_max = config.get('idle_observe_duration_max', 5.0)
        
        # 目标检测参数
        self.face_confirm_time = config.get('idle_face_confirm_time', 0.5)
        self.face_lost_tolerance = config.get('idle_face_lost_tolerance', 0.2)
        
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
        
        # 初始进入 Active 阶段
        self._current_phase = "active"
        
        # 必须重置候选状态，防止从 SEARCH 返回时保留了旧的 candidate
        self._reset_candidate()
        
        self._enter_active_phase()
        
    def _enter_active_phase(self):
        """进入活跃动作阶段"""
        self.execute_action("idle_normal") # 默认动作
        
        # 设定下一次切换时间 (10~20s)
        # 这里的 switch 指 Active -> Observe 的切换，或者是 Active 内部动作变换（如果需要）
        # 这里简化为：Active 持续一段时间后，切入 Observe
        # 这里简化为：Active 持续一段时间后，切入 Observe
        interval = random.uniform(self.switch_interval_min, self.switch_interval_max)
        self._next_switch_time = self.fsm.get_current_time() + interval
        self.log(f"Phase: ACTIVE. Next switch in {interval:.1f}s")

    def _enter_observe_phase(self):
        """进入静止观察阶段"""
        self.execute_action("idle_observe")
        
        # 观察持续 3~5 秒
        observe_duration = random.uniform(self.observe_duration_min, self.observe_duration_max)
        self._next_switch_time = self.fsm.get_current_time() + observe_duration
        self.log(f"Phase: OBSERVE. Duration {observe_duration:.1f}s")
        
        # 重置候选
        self._reset_candidate()
        
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
        """
        空闲状态主循环
        逻辑：
        1. 阶段切换 (Active <-> Observe)
        2. 候选人脸检测与确认 -> 切换 SEARCH
        """
        self.iter_count += 1
        current_time = self.fsm.get_current_time()
        
        # 1. 阶段/动作切换逻辑
        if current_time >= self._next_switch_time:
            if self._current_phase == "active":
                # Active -> Observe
                self._current_phase = "observe"
                self._enter_observe_phase()
            else:
                # Observe -> Active
                self._current_phase = "active"
                
                # Active 开始时，随机选择动作
                # 设定 Active 持续时间
                duration = 0.0
                
                # Check for explicit duration in action config
                # TODO: execute_action could return config, but for now we look it up
                # Note: execute_action handles the actual execution.
                
                # Active 开始时，随机选择动作
                action_name = "idle_normal"
                if random.random() < self.greeting_probability:
                    action_name = "greeting"
                    self.execute_action(action_name)
                    self.log("Action: greeting")
                else:
                    action_name = "idle_normal"
                    self.execute_action(action_name)
                    self.log("Action: idle_normal")
                
                # Determine duration
                if action_name in ACTION_PACKS and 'duration' in ACTION_PACKS[action_name]:
                    duration = ACTION_PACKS[action_name]['duration']
                    self.log(f"Using defined duration: {duration}s")
                else:
                    duration = random.uniform(self.switch_interval_min, self.switch_interval_max)
                
                self._next_switch_time = current_time + duration
                self.log(f"Phase: ACTIVE. Next switch in {duration:.1f}s")
        
        # 2. 目标检测逻辑
        if self._current_phase == "observe":
            self._update_candidate(current_time)
        
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
            if lost_duration > self.face_lost_tolerance:
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
            
            if effective_duration >= self.face_confirm_time:
                self.log(f"Candidate confirmed ({effective_duration:.2f}s). Proceeding to SEARCH.")
                
                # 保存目标信息到 LockedTarget (供 SEARCH/TRACKING 使用)
                if self._candidate_face:
                    from ...perception import LockedTarget
                    self.fsm.data.perception.locked_target = LockedTarget.from_face(self._candidate_face)
                
                return WeddingStateName.SEARCH
        
        return self.state_name

    def on_exit(self) -> None:
        self.stop_action()
        self.log("Exiting IDLE state")
