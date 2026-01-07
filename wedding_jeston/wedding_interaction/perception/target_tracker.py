"""
目标跟踪器

管理人脸-人体关联的目标跟踪逻辑
"""

import time
import logging
from typing import List, Optional, Tuple

from .data_types import FaceInfo, BodyInfo, TrackedTarget


class TargetTracker:
    """
    目标跟踪器
    
    核心功能：
    1. 管理当前跟踪目标
    2. 人脸-人体关联
    3. 正脸丢失时人体接管
    4. 正脸恢复时验证是否同一目标
    """
    
    # 配置参数
    T_FACE_CONFIRM = 2.0        # 正脸确认时间（秒）
    T_FACE_LOST_MAX = 3.0       # 正脸丢失最大容忍（秒）
    T_FACE_LOST_GRACE = 0.5     # 短暂丢失容忍（秒）
    T_BODY_LOCK = 0.3           # 人体锁定时间（秒）
    
    # 有效区域
    VALID_REGION = (0.15, 0.85, 0.1, 0.9)  # (x_min, x_max, y_min, y_max)
    
    # 面积阈值
    FACE_MIN_AREA = 0.01
    FACE_MAX_AREA = 0.30
    
    def __init__(self):
        """初始化目标跟踪器"""
        self.logger = logging.getLogger("TargetTracker")
        
        # 当前跟踪目标
        self._target: Optional[TrackedTarget] = None
        
        # 时间记录
        self._last_update_time = 0.0
        
        # 目标 ID 计数器
        self._next_target_id = 0
    
    @property
    def target(self) -> Optional[TrackedTarget]:
        """当前跟踪目标"""
        return self._target
    
    @property
    def has_target(self) -> bool:
        """是否有跟踪目标"""
        return self._target is not None and self._target.is_valid()
    
    @property
    def is_target_confirmed(self) -> bool:
        """目标是否已确认"""
        return self._target is not None and self._target.is_confirmed(self.T_FACE_CONFIRM)
    
    @property
    def is_target_lost(self) -> bool:
        """目标是否丢失"""
        return self._target is not None and self._target.is_lost(self.T_FACE_LOST_MAX)
    
    def update(self, faces: List[FaceInfo], bodies: List[BodyInfo], 
               dt: float) -> Optional[TrackedTarget]:
        """
        更新跟踪状态
        
        Args:
            faces: 检测到的人脸列表
            bodies: 检测到的人体列表
            dt: 时间间隔（秒）
        
        Returns:
            当前跟踪目标
        """
        current_time = time.time()
        
        if self._target is None:
            # 无目标，尝试选择新目标
            self._try_acquire_target(faces, bodies)
        else:
            # 有目标，更新跟踪状态
            self._update_tracking(faces, bodies, dt)
        
        self._last_update_time = current_time
        
        return self._target
    
    def _try_acquire_target(self, faces: List[FaceInfo], 
                            bodies: List[BodyInfo]) -> None:
        """
        尝试获取新目标
        
        选择最大的正脸，并关联对应的人体
        """
        # 选择最佳正脸
        best_face = self._select_best_face(faces)
        
        if best_face is None:
            return
        
        # 查找关联的人体
        associated_body = self._find_body_for_face(best_face, bodies)
        
        if associated_body is None:
            # 没有人体也可以开始跟踪，但优先有人体的
            # 暂时允许只有人脸的情况
            self.logger.debug("No body found for face, tracking face only")
        
        # 创建跟踪目标
        self._target = TrackedTarget(
            face=best_face,
            body=associated_body,
            position=best_face.center,
            is_face_visible=True,
            is_body_visible=associated_body is not None,
            is_frontal=best_face.is_frontal,
            first_detected_time=time.time(),
            face_visible_duration=0.0,
            face_lost_duration=0.0,
            body_lost_duration=0.0,
            target_id=self._get_next_target_id()
        )
        
        self.logger.info(f"New target acquired: id={self._target.target_id}, "
                        f"face={best_face.face_id}, "
                        f"body={associated_body.body_id if associated_body else 'None'}")
    
    def _update_tracking(self, faces: List[FaceInfo], 
                         bodies: List[BodyInfo], dt: float) -> None:
        """
        更新现有目标的跟踪状态
        
        关键逻辑：
        1. 如果正脸仍在当前人体内 → 更新人脸，继续计时
        2. 如果正脸丢失但人体仍在 → 人体接管，继续计时
        3. 如果有新正脸出现 → 验证是否在当前人体内
        4. 如果人脸和人体都丢失 → 累计丢失时间
        """
        target = self._target
        
        # 先更新人体状态
        current_body = self._find_matching_body(target.body, bodies)
        
        # 再更新人脸状态
        current_face = self._find_matching_face(target, faces, current_body)
        
        # 更新目标状态
        prev_face_visible = target.is_face_visible
        prev_body_visible = target.is_body_visible
        
        target.is_face_visible = current_face is not None and current_face.is_frontal
        target.is_body_visible = current_body is not None
        
        if current_face is not None:
            target.face = current_face
            target.is_frontal = current_face.is_frontal
        
        if current_body is not None:
            target.body = current_body
        
        # 更新位置
        target.position = target.get_tracking_position()
        
        # 更新时间统计
        if target.is_face_visible:
            # 正脸可见
            target.face_visible_duration += dt
            target.face_lost_duration = 0.0
        else:
            # 正脸丢失
            target.face_lost_duration += dt
        
        if target.is_body_visible:
            # 人体可见
            target.body_lost_duration = 0.0
        else:
            # 人体丢失
            target.body_lost_duration += dt
        
        # 检查是否应该放弃目标
        if self._should_abandon_target():
            self.logger.info(f"Target {target.target_id} lost, abandoning")
            self._target = None
    
    def _find_matching_face(self, target: TrackedTarget,
                            faces: List[FaceInfo],
                            current_body: Optional[BodyInfo]) -> Optional[FaceInfo]:
        """
        查找与当前目标匹配的人脸
        
        关键：如果已有人体，只接受在该人体内的人脸
        """
        if not faces:
            return None
        
        # 如果有关联的人体，只接受在该人体内的人脸
        if current_body is not None:
            valid_faces = [
                f for f in faces
                if f.is_frontal and current_body.contains_point(*f.center, region="upper")
            ]
        else:
            # 没有人体，使用之前的人脸 ID 匹配或位置匹配
            valid_faces = [f for f in faces if f.is_frontal]
        
        if not valid_faces:
            return None
        
        # 选择与上次人脸位置最近的
        if target.face is not None:
            last_pos = target.face.center
            return min(valid_faces, 
                      key=lambda f: self._distance(f.center, last_pos))
        
        # 否则选择最大的
        return max(valid_faces, key=lambda f: f.area)
    
    def _find_matching_body(self, last_body: Optional[BodyInfo],
                            bodies: List[BodyInfo]) -> Optional[BodyInfo]:
        """
        查找与上次人体匹配的人体
        
        使用位置相似度进行匹配
        """
        if not bodies:
            return None
        
        if last_body is None:
            # 没有上次人体，返回最大的
            return max(bodies, key=lambda b: b.area)
        
        # 查找位置最接近的人体
        last_pos = last_body.center
        
        # 找最近的人体
        closest = min(bodies, key=lambda b: self._distance(b.center, last_pos))
        
        # 检查是否足够近（阈值 0.3）
        if self._distance(closest.center, last_pos) < 0.3:
            return closest
        
        return None
    
    def _select_best_face(self, faces: List[FaceInfo]) -> Optional[FaceInfo]:
        """
        选择最佳人脸
        
        优先级：正脸 + 有效区域内 + 面积最大
        """
        x_min, x_max, y_min, y_max = self.VALID_REGION
        
        valid_faces = [
            f for f in faces
            if f.is_frontal
            and f.is_in_region(x_min, x_max, y_min, y_max)
            and self.FACE_MIN_AREA <= f.area <= self.FACE_MAX_AREA
        ]
        
        if not valid_faces:
            return None
        
        return max(valid_faces, key=lambda f: f.area)
    
    def _find_body_for_face(self, face: FaceInfo, 
                            bodies: List[BodyInfo]) -> Optional[BodyInfo]:
        """为人脸查找关联的人体"""
        fx, fy = face.center
        
        for body in bodies:
            if body.contains_point(fx, fy, region="upper"):
                return body
        
        return None
    
    def _should_abandon_target(self) -> bool:
        """检查是否应该放弃当前目标"""
        if self._target is None:
            return False
        
        target = self._target
        
        # 人脸和人体都丢失超过最大容忍时间
        if (not target.is_face_visible and 
            not target.is_body_visible and
            target.body_lost_duration > self.T_FACE_LOST_MAX):
            return True
        
        return False
    
    def _distance(self, p1: Tuple[float, float], 
                  p2: Tuple[float, float]) -> float:
        """计算两点距离"""
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    
    def _get_next_target_id(self) -> int:
        """获取下一个目标 ID"""
        target_id = self._next_target_id
        self._next_target_id += 1
        return target_id
    
    def reset(self) -> None:
        """重置跟踪器"""
        if self._target is not None:
            self.logger.info(f"Resetting tracker, abandoning target {self._target.target_id}")
        self._target = None
    
    def get_tracking_info(self) -> dict:
        """获取跟踪状态信息（用于调试）"""
        if self._target is None:
            return {
                'has_target': False,
                'target_id': -1,
                'position': (0.5, 0.5),
                'is_face_visible': False,
                'is_body_visible': False,
                'face_visible_duration': 0.0,
                'face_lost_duration': 0.0,
                'is_confirmed': False,
                'is_lost': False,
            }
        
        return {
            'has_target': True,
            'target_id': self._target.target_id,
            'position': self._target.position,
            'is_face_visible': self._target.is_face_visible,
            'is_body_visible': self._target.is_body_visible,
            'face_visible_duration': self._target.face_visible_duration,
            'face_lost_duration': self._target.face_lost_duration,
            'is_confirmed': self.is_target_confirmed,
            'is_lost': self.is_target_lost,
        }

