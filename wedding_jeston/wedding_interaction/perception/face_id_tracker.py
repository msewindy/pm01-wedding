"""
轻量级人脸 ID 追踪器

基于 IoU 和位置距离的跨帧人脸匹配，维护持久化 ID
配合 MediaPipe 的追踪能力，实现高效稳定的人脸追踪
"""

import time
import logging
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field
import numpy as np

from .data_types import FaceInfo


@dataclass
class TrackedFace:
    """追踪中的人脸信息"""
    face_id: int
    bbox: Tuple[float, float, float, float]  # (x, y, w, h)
    center: Tuple[float, float]
    last_seen: float  # 最后出现时间（用于判断未出现时间）
    confidence: float = 0.0


class FaceIDTracker:
    """
    轻量级人脸 ID 追踪器
    
    使用 IoU 和位置距离匹配跨帧人脸，维护持久化 ID
    配合 MediaPipe 的追踪能力，实现高效稳定的人脸追踪
    
    特点：
    1. 基于 IoU 的匹配（更准确）
    2. 支持多目标追踪
    3. 自动清理丢失目标
    4. 性能优化（30fps+）
    """
    
    # 匹配阈值
    IOU_THRESHOLD = 0.15  # IoU 匹配阈值（从0.25降低到0.15，提高运动和噪声下的匹配率）
    DISTANCE_THRESHOLD = 0.30  # 位置距离阈值（归一化，从0.25增加到0.30，增加容错）
    HIGH_IOU_THRESHOLD = 0.85  # 高 IoU 阈值（用于放宽距离限制）
    
    # 目标丢失清理（增加容错时间，避免位置固定时被过早清理）
    MAX_TIME_SINCE_SEEN = 5.0  # 最大未出现时间（秒，增加容错，允许短暂检测失败）
    
    def __init__(self):
        self.logger = logging.getLogger("FaceIDTracker")
        
        # 追踪目标字典 {face_id: TrackedFace}
        self._tracks: Dict[int, TrackedFace] = {}
        
        # 下一个可用的 face_id
        self._next_id = 0
        
        # 统计
        self._total_detections = 0
        self._total_matches = 0
    
    def update(self, faces: List[FaceInfo]) -> List[FaceInfo]:
        """
        更新追踪器，为检测到的人脸分配持久化 ID
        
        Args:
            faces: 当前帧检测到的人脸列表（未分配 ID 或 ID 无效）
        
        Returns:
            分配了持久化 ID 的人脸列表
        """
        current_time = time.time()
        self._total_detections += len(faces)
        
        # 记录开始状态
        if len(faces) > 0 or len(self._tracks) > 0:
            self.logger.debug(f"[FaceIDTracker] 更新开始: 检测到{len(faces)}个人脸, 现有{len(self._tracks)}个追踪目标")
        
        # 为每个检测到的人脸创建 bbox 用于匹配
        detections = []
        for face in faces:
            bbox = (face.x, face.y, face.width, face.height)
            center = face.center
            detections.append((face, bbox, center))
            if len(faces) > 0:
                self.logger.debug(f"[FaceIDTracker] 检测到人脸: bbox=({bbox[0]:.3f}, {bbox[1]:.3f}, {bbox[2]:.3f}, {bbox[3]:.3f}), "
                                f"center=({center[0]:.3f}, {center[1]:.3f})")
        
        # 记录现有追踪目标信息
        if len(self._tracks) > 0:
            for track_id, track in self._tracks.items():
                self.logger.debug(f"[FaceIDTracker] 现有追踪目标 ID={track_id}: "
                                f"bbox=({track.bbox[0]:.3f}, {track.bbox[1]:.3f}, {track.bbox[2]:.3f}, {track.bbox[3]:.3f}), "
                                f"center=({track.center[0]:.3f}, {track.center[1]:.3f}), "
                                f"last_seen={current_time - track.last_seen:.3f}s前")
        
        # 匹配现有追踪目标
        matched_tracks = set()
        matched_detections = set()
        
        # 使用 IoU 进行匹配
        for track_id, track in self._tracks.items():
            best_iou = 0.0  # 初始化为 0，而不是阈值，以便找到最佳匹配
            best_detection_idx = None
            best_distance = float('inf')
            
            for i, (face, bbox, center) in enumerate(detections):
                if i in matched_detections:
                    continue
                
                # 计算 IoU
                iou = self._calculate_iou(track.bbox, bbox)
                distance = self._distance(track.center, center)
                
                # 记录匹配尝试
                self.logger.debug(f"[FaceIDTracker] 匹配尝试: track_id={track_id} vs detection[{i}]: "
                                f"IoU={iou:.3f} (阈值={self.IOU_THRESHOLD}), "
                                f"distance={distance:.3f} (阈值={self.DISTANCE_THRESHOLD})")
                
                # 匹配策略（优先级从高到低）：
                # 1. 如果 IoU 很高（> 0.85），即使距离稍大也匹配（容错机制，处理微小变化）
                # 2. 如果距离很近（< 0.15），即使 IoU 稍低也匹配（处理 bbox 微小变化）
                # 3. 正常匹配：IoU > 阈值 且 距离 < 阈值
                is_high_iou_match = iou > self.HIGH_IOU_THRESHOLD and distance < self.DISTANCE_THRESHOLD * 2.5
                is_close_distance_match = distance < 0.15 and iou > 0.1  # 距离很近时，IoU 要求进一步降低
                is_normal_match = iou > self.IOU_THRESHOLD and distance < self.DISTANCE_THRESHOLD
                
                if is_high_iou_match or is_close_distance_match or is_normal_match:
                    # 选择 IoU 最高的匹配
                    if iou > best_iou:
                        best_iou = iou
                        best_detection_idx = i
                        best_distance = distance
            
            # 如果找到匹配（IoU 或距离满足条件），更新追踪目标
            # 注意：best_iou 可能小于 IOU_THRESHOLD（如果是 close_distance_match），但仍然有效
            if best_detection_idx is not None and best_iou > 0.2:
                face, bbox, center = detections[best_detection_idx]
                face.face_id = track_id
                
                # 更新追踪目标
                track.bbox = bbox
                track.center = center
                track.last_seen = current_time  # 更新最后出现时间（用于判断未出现时间）
                track.confidence = face.confidence
                
                matched_tracks.add(track_id)
                matched_detections.add(best_detection_idx)
                self._total_matches += 1
                
                self.logger.info(f"[FaceIDTracker] ✅ 匹配成功: track_id={track_id} ← detection[{best_detection_idx}], "
                               f"IoU={best_iou:.3f}, distance={best_distance:.3f}")
            else:
                # 记录匹配失败的原因
                if len(detections) > 0:
                    # 找到该track_id对应的最佳匹配（即使不满足阈值）
                    best_iou_all = 0.0
                    best_distance_all = float('inf')
                    for i, (face, bbox, center) in enumerate(detections):
                        if i not in matched_detections:
                            iou = self._calculate_iou(track.bbox, bbox)
                            distance = self._distance(track.center, center)
                            if iou > best_iou_all:
                                best_iou_all = iou
                                best_distance_all = distance
                    
                    # 如果 IoU 很高但距离稍大，或者距离很近但 IoU 稍低，记录为警告
                    # 这种情况可能是阈值设置过严导致的
                    if best_iou_all > 0.8 or best_distance_all < 0.15:
                        self.logger.warning(f"[FaceIDTracker] ⚠️ 匹配失败（接近阈值）: track_id={track_id}, "
                                          f"最佳IoU={best_iou_all:.3f} (阈值={self.IOU_THRESHOLD}), "
                                          f"最佳distance={best_distance_all:.3f} (阈值={self.DISTANCE_THRESHOLD}), "
                                          f"可能是阈值过严或检测有微小变化")
                    else:
                        self.logger.debug(f"[FaceIDTracker] ❌ 匹配失败: track_id={track_id}, "
                                        f"最佳IoU={best_iou_all:.3f}, 最佳distance={best_distance_all:.3f}")
        
        # 为未匹配的检测创建新的追踪目标
        for i, (face, bbox, center) in enumerate(detections):
            if i not in matched_detections:
                face_id = self._get_next_id()
                face.face_id = face_id
                
                # 创建新的追踪目标
                self._tracks[face_id] = TrackedFace(
                    face_id=face_id,
                    bbox=bbox,
                    center=center,
                    last_seen=current_time,
                    confidence=face.confidence
                )
                
                self.logger.warning(f"[FaceIDTracker] 🆕 创建新追踪目标: face_id={face_id}, "
                                  f"bbox=({bbox[0]:.3f}, {bbox[1]:.3f}, {bbox[2]:.3f}, {bbox[3]:.3f}), "
                                  f"center=({center[0]:.3f}, {center[1]:.3f}), "
                                  f"原因: 未匹配到现有追踪目标")
        
        # 清理丢失的追踪目标
        # 注意：只有在有检测结果时才清理，避免在检测失败时过早清理
        # 如果 faces 为空，说明可能是检测失败，不应该立即清理追踪目标
        if len(faces) > 0:
            removed_tracks = self._cleanup_lost_tracks(current_time)
            if removed_tracks:
                self.logger.info(f"[FaceIDTracker] 🗑️ 清理丢失追踪目标: {removed_tracks}")
        else:
            # 没有检测到人脸，可能是检测失败，不清理追踪目标
            # 但记录一下，用于调试
            if len(self._tracks) > 0:
                self.logger.debug(f"[FaceIDTracker] 未检测到人脸，保留{len(self._tracks)}个追踪目标")
        
        # 记录最终状态
        if len(faces) > 0 or len(self._tracks) > 0:
            assigned_ids = [f.face_id for f in faces]
            self.logger.debug(f"[FaceIDTracker] 更新完成: 分配IDs={assigned_ids}, 活跃追踪目标数={len(self._tracks)}")
        
        return faces
    
    def _calculate_iou(self, bbox1: Tuple[float, float, float, float],
                       bbox2: Tuple[float, float, float, float]) -> float:
        """计算两个边界框的 IoU"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # 计算交集
        x1_min, y1_min = x1, y1
        x1_max, y1_max = x1 + w1, y1 + h1
        x2_min, y2_min = x2, y2
        x2_max, y2_max = x2 + w2, y2 + h2
        
        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)
        
        if inter_x_max <= inter_x_min or inter_y_max <= inter_y_min:
            return 0.0
        
        inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
        area1 = w1 * h1
        area2 = w2 * h2
        union_area = area1 + area2 - inter_area
        
        if union_area <= 0:
            return 0.0
        
        return inter_area / union_area
    
    def _distance(self, p1: Tuple[float, float], 
                  p2: Tuple[float, float]) -> float:
        """计算两点距离（归一化坐标）"""
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    
    def _cleanup_lost_tracks(self, current_time: float) -> List[int]:
        """清理丢失的追踪目标"""
        tracks_to_remove = []
        
        for track_id, track in self._tracks.items():
            time_since_seen = current_time - track.last_seen
            
            # 清理策略：
            # 基于 last_seen 判断未出现时间，如果超过阈值则清理
            # 注意：如果追踪目标一直在匹配成功，last_seen 会持续更新，不会被清理
            should_remove = False
            reason = ""
            
            if time_since_seen > self.MAX_TIME_SINCE_SEEN:
                should_remove = True
                reason = f"未出现时间={time_since_seen:.3f}s > {self.MAX_TIME_SINCE_SEEN}s"
            
            if should_remove:
                tracks_to_remove.append(track_id)
                self.logger.warning(f"[FaceIDTracker] 标记清理: track_id={track_id}, {reason}")
        
        for track_id in tracks_to_remove:
            del self._tracks[track_id]
        
        return tracks_to_remove
    
    def _get_next_id(self) -> int:
        """获取下一个可用的 face_id"""
        face_id = self._next_id
        self._next_id += 1
        return face_id
    
    def reset(self) -> None:
        """重置追踪器"""
        self._tracks.clear()
        self._next_id = 0
        self._total_detections = 0
        self._total_matches = 0
    
    def get_stats(self) -> dict:
        """获取追踪统计信息"""
        match_rate = (self._total_matches / self._total_detections * 100 
                     if self._total_detections > 0 else 0.0)
        
        return {
            'active_tracks': len(self._tracks),
            'total_detections': self._total_detections,
            'total_matches': self._total_matches,
            'match_rate': match_rate
        }

