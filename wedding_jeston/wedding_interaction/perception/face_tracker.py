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
    has_target: bool = False          # 是否有跟踪目标 (含容忍期)
    target_face: Optional[FaceInfo] = None  # 当前目标人脸
    is_confirmed: bool = False        # 目标是否正脸确认 (for Search)
    is_lost: bool = False             # 是否处于丢失状态 (但未超时)
    lost_duration: float = 0.0        # 丢失持续时间
    frontal_duration: float = 0.0     # 正脸持续时间
    tracking_duration: float = 0.0    # 总跟踪时间
    position: Tuple[float, float] = (0.5, 0.5)  # 目标位置 (中心点)


class FaceTracker:
    """
    统一的人脸跟踪器
    
    负责：
    1. 跨帧目标关联 (ID匹配 + 位置匹配)
    2. 状态平滑与去抖动 (Grace Period)
    3. 目标确认逻辑 (Confirmation)
    """
    
    # 默认配置
    DEFAULT_CONFIRM_DURATION = 2.0
    DEFAULT_LOST_TIMEOUT = 1.0  # 默认短容忍 (for search)
    DEFAULT_POS_MATCH_THRESHOLD = 0.3
    
    # 有效区域
    VALID_REGION = (0.15, 0.85, 0.1, 0.9)
    MIN_FACE_AREA = 0.01
    
    def __init__(self, logger=None):
        self.logger = logger if logger else logging.getLogger("FaceTracker")
        
        # 配置参数
        self.confirm_duration = self.DEFAULT_CONFIRM_DURATION
        self.lost_timeout = self.DEFAULT_LOST_TIMEOUT
        self.pos_match_threshold = self.DEFAULT_POS_MATCH_THRESHOLD
        
        # 内部状态
        self._target: Optional[FaceInfo] = None   # 当前锁定的目标 (最新帧数据)
        self._last_known_target: Optional[FaceInfo] = None # 最后已知目标 (用于丢失期间预测)
        
        # 时间戳记录
        self._tracking_start_time: float = 0.0
        self._frontal_start_time: float = 0.0
        self._lost_start_time: float = 0.0
        self._last_update_time: float = 0.0
        
        # 状态累计
        self._frontal_duration_acc: float = 0.0
        
    def configure(self, lost_timeout: float = None, confirm_duration: float = None, pos_match_threshold: float = None):
        """动态配置参数"""
        if lost_timeout is not None:
            self.lost_timeout = lost_timeout
        if confirm_duration is not None:
            self.confirm_duration = confirm_duration
        if pos_match_threshold is not None:
            self.pos_match_threshold = pos_match_threshold
        
        self.logger.info(f"Tracker Config: lost_timeout={self.lost_timeout:.1f}s, "
                        f"confirm={self.confirm_duration:.1f}s, "
                        f"pos_thresh={self.pos_match_threshold:.2f}")

    def reset(self):
        """重置跟踪器"""
        self._target = None
        self._last_known_target = None
        self._tracking_start_time = 0.0
        self._frontal_start_time = 0.0
        self._lost_start_time = 0.0
        self._frontal_duration_acc = 0.0
        self.logger.debug("Tracker reset")

    def set_target(self, target: Union[FaceInfo, LockedTarget]):
        """手动设置目标 (从其他状态传递)"""
        current_time = time.time()
        
        if isinstance(target, LockedTarget):
             # 构造一个临时 FaceInfo，主要用 ID 和 位置
             face = FaceInfo(
                x=target.bbox[0], y=target.bbox[1], width=target.bbox[2], height=target.bbox[3],
                face_id=target.face_id, confidence=0.9, is_frontal=True, timestamp=current_time
             )
             self._target = face
        else:
            self._target = target
            
        self._last_known_target = self._target
        self._tracking_start_time = current_time
        self._frontal_start_time = current_time if self._target.is_frontal else 0.0
        self._lost_start_time = 0.0
        self.logger.info(f"Target set manually: {self._target.face_id}")

    def update(self, faces: List[FaceInfo]) -> TrackingState:
        """每一帧更新跟踪状态"""
        current_time = time.time()
        
        # 1. 如果当前没有目标，尝试在当前帧寻找新候选 (Search Mode)
        #    如果已经有目标 (_target or _last_known_target)，则尝试匹配
        
        matched_face = None
        
        if self._target is None and self._last_known_target is None:
            # Case A: 完全无目标 -> 寻找最佳候选
            matched_face = self._select_best_candidate(faces)
            if matched_face:
                # Found new target
                self._start_tracking(matched_face, current_time)
        else:
            # Case B: 已有目标 -> 尝试匹配
            # 优先用 _target (最新), 如果 _target 是 None (丢失中), 用 _last_known_target
            ref_target = self._target if self._target else self._last_known_target
            matched_face = self._find_matching_face(faces, ref_target)
            
            if matched_face:
                # Matched!
                self._update_tracking(matched_face, current_time)
            else:
                # Lost in this frame
                self._handle_lost(current_time)
        
        self._last_update_time = current_time
        return self._get_state(current_time)

    def _start_tracking(self, face: FaceInfo, now: float):
        self._target = face
        self._last_known_target = face
        self._tracking_start_time = now
        self._frontal_start_time = now if face.is_frontal else 0.0
        self._frontal_duration_acc = 0.0
        self._lost_start_time = 0.0
        self.logger.debug(f"Auto-acquired target: {face.face_id}")

    def _update_tracking(self, face: FaceInfo, now: float):
        # 如果从丢失中恢复
        if self._lost_start_time > 0:
            lost_dur = now - self._lost_start_time
            self.logger.debug(f"Target recovered after {lost_dur:.2f}s")
            self._lost_start_time = 0.0
            
        # ID 变更检查 (Position Match case)
        if self._target and self._target.face_id != face.face_id:
            self.logger.info(f"ID Changed {self._target.face_id} -> {face.face_id} (Position Match)")
            
        self._target = face
        self._last_known_target = face
        
        # 正脸计时
        if face.is_frontal:
            if self._frontal_start_time == 0.0:
                 self._frontal_start_time = now
        else:
            # 侧脸不中断累计，但打断连续计时? 
            # 需求: "正脸持续时间 >= 2s". 这里简单处理：只有连续正脸才算
            # 或者：只要没有彻底丢失，侧脸也算 confirmed? 
            # 原逻辑: 侧脸会重置 frontal_start_time. 保持原逻辑严格性.
            self._frontal_start_time = 0.0
            self._frontal_duration_acc = 0.0 # Reset accumulated if strict

    def _handle_lost(self, now: float):
        self._target = None # 当前帧无目标
        if self._lost_start_time == 0.0:
            self._lost_start_time = now
            # self.logger.debug("Target lost signal...")

    def _get_state(self, now: float) -> TrackingState:
        # 计算丢失时长
        lost_duration = 0.0
        is_lost = False
        
        if self._lost_start_time > 0:
            lost_duration = now - self._lost_start_time
            is_lost = True
        
        # 判断是否"真的丢失" (超过容忍时间)
        has_target = False
        if self._target:
            has_target = True
        elif self._last_known_target:
            # 处于丢失容忍期
            if lost_duration <= self.lost_timeout:
                has_target = True
            else:
                 # 超时，彻底放弃
                self._last_known_target = None 
                has_target = False
        
        # 正脸时长
        frontal_dur = 0.0
        if self._target and self._target.is_frontal and self._frontal_start_time > 0:
             frontal_dur = now - self._frontal_start_time
        
        # 构造返回状态
        # 如果处于容忍期 (_target is None but _last_known is valid), 用 _last_known 的位置
        final_face = self._target if self._target else self._last_known_target
        
        return TrackingState(
            has_target=has_target,
            target_face=final_face,
            is_confirmed=(frontal_dur >= self.confirm_duration),
            is_lost=is_lost,
            lost_duration=lost_duration,
            frontal_duration=frontal_dur,
            tracking_duration=(now - self._tracking_start_time) if self._tracking_start_time > 0 else 0.0,
            position=final_face.center if final_face else (0.5, 0.5)
        )

    def _select_best_candidate(self, faces: List[FaceInfo]) -> Optional[FaceInfo]:
        """选择最佳候选 (Search用)"""
        candidates = []
        for f in faces:
            if not f.is_frontal: continue
            if not f.is_in_region(*self.VALID_REGION): continue
            if f.area < self.MIN_FACE_AREA: continue
            candidates.append(f)
        
        if not candidates: return None
        # 选面积最大的
        return max(candidates, key=lambda x: x.area)

    def _find_matching_face(self, faces: List[FaceInfo], target: FaceInfo) -> Optional[FaceInfo]:
        """匹配逻辑: ID优先 -> 位置兜底"""
        if not faces: return None
        
        # 1. ID Match
        for f in faces:
            if f.face_id == target.face_id:
                return f
                
        # 2. Position Match (仅当 FaceIDTracker 不稳定时)
        # 阈值 (使用配置值)
        POS_THRESH = self.pos_match_threshold
        target_pos = target.center
        
        best = None
        min_dist = POS_THRESH
        
        for f in faces:
            dist = ((f.center_x - target_pos[0])**2 + (f.center_y - target_pos[1])**2)**0.5
            if dist < min_dist:
                min_dist = dist
                best = f
        
        if best:
             self.logger.debug(f"Position match detected: {target.face_id} -> {best.face_id} (dist={min_dist:.2f})")
             
        return best

    # Compatibility methods
    def get_locked_target(self) -> Optional[LockedTarget]:
        if self._target and self.is_target_confirmed: # Uses property below? No, use state
             # This method is for SearchState to lock target
             # We can't rely on self._get_state() here easily without time. 
             # Simpler:
             return LockedTarget.from_face(self._target)
        return None

    @property
    def has_target(self) -> bool:
        """是否有跟踪目标 (含容忍期)"""
        state = self._get_state(time.time())
        return state.has_target

    @property
    def is_target_confirmed(self):
        # Helper for external property access
        # Caution: this relies on update() being called recently
        state = self._get_state(time.time())
        return state.is_confirmed
    
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

