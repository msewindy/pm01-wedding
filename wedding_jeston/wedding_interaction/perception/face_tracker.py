"""
人脸跟踪器

用于 SEARCH 状态的人脸跟踪，负责：
1. 通过 face_id 跨帧匹配人脸（完全依赖 FaceIDTracker 分配的 ID）
2. 维护正脸持续时间计时

注意：只依赖 face_id 匹配，如果 face_id 不存在或匹配失败，则进入目标丢失逻辑
"""

import time
import logging
from typing import List, Optional, Tuple
from dataclasses import dataclass

from .data_types import FaceInfo, LockedTarget
from typing import Union


@dataclass
class TrackingState:
    """跟踪状态信息"""
    has_target: bool = False          # 是否有跟踪目标
    target_face: Optional[FaceInfo] = None  # 当前目标人脸
    is_frontal: bool = False          # 目标是否正脸
    frontal_duration: float = 0.0     # 正脸持续时间（秒）
    tracking_duration: float = 0.0    # 总跟踪时间（秒）
    position: Tuple[float, float] = (0.5, 0.5)  # 目标位置


class FaceTracker:
    """
    人脸跟踪器
    
    用于 SEARCH 状态，维护对单个正脸目标的跟踪
    """
    
    # 有效区域
    VALID_REGION = (0.15, 0.85, 0.1, 0.9)  # (x_min, x_max, y_min, y_max)
    
    # 面积阈值
    MIN_FACE_AREA = 0.01    # 最小人脸面积
    MAX_FACE_AREA = 0.30    # 最大人脸面积（太近）
    
    # 确认时间
    CONFIRM_DURATION = 2.0  # 正脸持续 2s 确认
    
    def __init__(self):
        self.logger = logging.getLogger("FaceTracker")
        
        # 当前跟踪目标
        self._target: Optional[FaceInfo] = None
        
        # 计时
        self._tracking_start_time: float = 0.0  # 开始跟踪的时间
        self._frontal_start_time: float = 0.0   # 正脸开始的时间
        self._frontal_duration: float = 0.0     # 正脸累计持续时间
        
        self._last_update_time: float = 0.0
    
    def update(self, faces: List[FaceInfo], dt: float = 0.0) -> TrackingState:
        """
        更新跟踪状态
        
        Args:
            faces: 当前帧检测到的人脸列表
            dt: 距离上次更新的时间间隔（秒）
        
        Returns:
            当前跟踪状态
        """
        current_time = time.time()
        
        # 尝试选择或保持目标
        new_target = self._select_or_keep_target(faces)
        
        if new_target is not None:
            # 有目标
            if self._target is None:
                # 新开始跟踪
                self._start_tracking(new_target, current_time)
            else:
                # 继续跟踪
                self._continue_tracking(new_target, current_time)
        else:
            # 无目标
            if self._target is not None:
                self._stop_tracking()
        
        self._last_update_time = current_time
        
        return self._get_state()
    
    def _select_or_keep_target(self, faces: List[FaceInfo]) -> Optional[FaceInfo]:
        """
        选择或保持目标
        
        只依赖 face_id 匹配，如果 face_id 不存在则目标丢失
        """
        if self._target is not None:
            # 已有目标，尝试通过 face_id 匹配
            matched = self._find_matching_face(faces, self._target)
            if matched is not None:
                # 匹配成功，返回匹配的人脸（可以是侧脸）
                return matched
            
            # face_id 匹配失败，目标丢失
            return None
        
        # 没有目标（不应该发生，因为 IDLE 已经选择了）
        return None
    
    def _find_matching_face(self, faces: List[FaceInfo], 
                            target: FaceInfo) -> Optional[FaceInfo]:
        """
        在当前帧中找到与目标匹配的人脸
        
        匹配策略（优先级从高到低）：
        1. face_id 匹配（最可靠）
        2. 位置匹配（容错机制，当 face_id 变化时使用）
        
        Args:
            faces: 当前帧的人脸列表
            target: 目标人脸
        
        Returns:
            匹配的人脸（可以是侧脸），或 None（如果匹配失败）
        """
        # 1. 优先使用持久化 ID 匹配
        if target.face_id >= 0:
            for face in faces:
                if face.face_id == target.face_id:
                    # ID 匹配成功，直接返回（可以是侧脸）
                    return face
        
        # 2. ID 匹配失败，尝试位置匹配（容错机制）
        # 当 FaceIDTracker 偶尔匹配失败导致 face_id 变化时，通过位置匹配恢复跟踪
        target_center = target.center
        best_match = None
        best_distance = float('inf')
        POSITION_MATCH_THRESHOLD = 0.30  # 位置匹配阈值（归一化距离，从0.15放宽到0.30，处理ID切换时的位置跳变）
        
        # 优先匹配正脸，如果没有正脸再考虑侧脸
        for face in faces:
            # 计算位置距离
            face_center = face.center
            distance = ((target_center[0] - face_center[0]) ** 2 + 
                       (target_center[1] - face_center[1]) ** 2) ** 0.5
            
            # 如果距离足够近，且是当前最佳匹配
            # 优先匹配正脸，但如果没有正脸，也允许匹配侧脸
            if distance < POSITION_MATCH_THRESHOLD and distance < best_distance:
                # 如果当前最佳匹配是侧脸，且新匹配是正脸，优先选择正脸
                if best_match is None or (face.is_frontal and not best_match.is_frontal):
                    best_match = face
                    best_distance = distance
                # 如果都是正脸或都是侧脸，选择距离更近的
                elif face.is_frontal == best_match.is_frontal:
                    best_match = face
                    best_distance = distance
        
        # 调试日志：如果 ID 匹配失败且位置匹配也失败，记录原因
        if best_match is None and faces:
            # 找到最近的人脸距离，用于调试
            min_dist = float('inf')
            closest_id = -1
            for face in faces:
                d = ((target_center[0] - face.center[0]) ** 2 + (target_center[1] - face.center[1]) ** 2) ** 0.5
                if d < min_dist:
                    min_dist = d
                    closest_id = face.face_id
            
            self.logger.debug(f"[FaceTracker] Match failed: target_id={target.face_id} not found. "
                            f"Closest face id={closest_id} dist={min_dist:.3f} (thresh={POSITION_MATCH_THRESHOLD})")
        
        # 如果找到位置匹配，记录日志并返回
        if best_match is not None:
            self.logger.warning(
                f"Face ID mismatch: target_id={target.face_id} -> matched_id={best_match.face_id}, "
                f"using position match (distance={best_distance:.3f})"
            )
            return best_match
        
        # 匹配失败
        return None
    
    def _start_tracking(self, face: FaceInfo, current_time: float) -> None:
        """开始跟踪新目标"""
        self._target = face
        self._tracking_start_time = current_time
        self._frontal_start_time = current_time
        self._frontal_duration = 0.0
        self.logger.debug(f"Started tracking face at ({face.center_x:.2f}, {face.center_y:.2f})")
    
    def _continue_tracking(self, face: FaceInfo, current_time: float) -> None:
        """继续跟踪当前目标"""
        # 如果 face_id 发生变化（通过位置匹配恢复），更新目标的 face_id
        if self._target is not None and self._target.face_id != face.face_id:
            self.logger.info(
                f"Updating target face_id: {self._target.face_id} -> {face.face_id} "
                f"(position match recovery)"
            )
        
        self._target = face
        
        # 更新正脸持续时间
        if face.is_frontal:
            if self._frontal_start_time == 0.0:
                self._frontal_start_time = current_time
            self._frontal_duration = current_time - self._frontal_start_time
        else:
            # 不是正脸，重置计时
            self._frontal_start_time = 0.0
            self._frontal_duration = 0.0
    
    def _stop_tracking(self) -> None:
        """停止跟踪"""
        self.logger.debug("Target lost, stopped tracking")
        self._target = None
        self._tracking_start_time = 0.0
        self._frontal_start_time = 0.0
        self._frontal_duration = 0.0
    
    def _get_state(self) -> TrackingState:
        """获取当前跟踪状态"""
        if self._target is None:
            return TrackingState(
                has_target=False,
                target_face=None,
                is_frontal=False,
                frontal_duration=0.0,
                tracking_duration=0.0,
                position=(0.5, 0.5)
            )
        
        current_time = time.time()
        tracking_duration = current_time - self._tracking_start_time
        
        return TrackingState(
            has_target=True,
            target_face=self._target,
            is_frontal=self._target.is_frontal,
            frontal_duration=self._frontal_duration,
            tracking_duration=tracking_duration,
            position=self._target.center
        )
    
    @property
    def is_target_confirmed(self) -> bool:
        """目标是否已确认（正脸持续超过阈值）"""
        return self._frontal_duration >= self.CONFIRM_DURATION
    
    @property
    def current_target(self) -> Optional[FaceInfo]:
        """当前跟踪目标"""
        return self._target
    
    def get_locked_target(self) -> Optional[LockedTarget]:
        """获取锁定的目标信息（用于传递给 TRACKING 状态）"""
        if self._target is None or not self.is_target_confirmed:
            return None
        return LockedTarget.from_face(self._target)
    
    def set_initial_target(self, target: Union[FaceInfo, LockedTarget]) -> None:
        """
        设置初始目标（从 IDLE 传递）
        
        Args:
            target: FaceInfo 或 LockedTarget 对象
        """
        if isinstance(target, LockedTarget):
            # 从 LockedTarget 创建 FaceInfo（用于匹配）
            # 注意：LockedTarget 不包含所有 FaceInfo 字段，我们使用位置信息
            # center_x 和 center_y 是属性，不能直接设置，需要通过 x, y, width, height 计算
            self._target = FaceInfo(
                x=target.bbox[0],
                y=target.bbox[1],
                width=target.bbox[2],
                height=target.bbox[3],
                face_id=target.face_id,
                is_frontal=True,  # 假设是正脸（因为是从 IDLE 传递的）
                confidence=0.9,
                timestamp=target.lock_time
            )
        else:
            # 直接使用 FaceInfo
            self._target = target
        
        self._tracking_start_time = time.time()
        # 如果目标不是正脸，frontal_start_time 为 0
        self._frontal_start_time = time.time() if self._target.is_frontal else 0.0
        self._frontal_duration = 0.0
        
        self.logger.info(f"Set initial target: face_id={self._target.face_id}, "
                        f"pos=({self._target.center_x:.2f}, {self._target.center_y:.2f})")
    
    def reset(self) -> None:
        """重置跟踪器"""
        self._target = None
        self._tracking_start_time = 0.0
        self._frontal_start_time = 0.0
        self._frontal_duration = 0.0
        self._last_update_time = 0.0
        self.logger.debug("Tracker reset")
    
    def get_info(self) -> dict:
        """获取跟踪信息（用于调试显示）"""
        state = self._get_state()
        return {
            'has_target': state.has_target,
            'is_frontal': state.is_frontal,
            'frontal_duration': state.frontal_duration,
            'is_confirmed': self.is_target_confirmed,
            'position': state.position,
        }

