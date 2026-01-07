"""
感知数据类型定义

定义人脸信息和锁定目标数据结构
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple
import time


@dataclass
class FaceInfo:
    """人脸信息"""
    # 边界框（归一化坐标 0~1）
    x: float = 0.0          # 左上角 x
    y: float = 0.0          # 左上角 y
    width: float = 0.0      # 宽度
    height: float = 0.0     # 高度
    
    # 计算属性
    @property
    def area(self) -> float:
        """边界框面积"""
        return self.width * self.height
    
    @property
    def center(self) -> Tuple[float, float]:
        """中心点坐标"""
        return (self.x + self.width / 2, self.y + self.height / 2)
    
    @property
    def center_x(self) -> float:
        """中心点 x 坐标"""
        return self.x + self.width / 2
    
    @property
    def center_y(self) -> float:
        """中心点 y 坐标"""
        return self.y + self.height / 2
    
    # 正脸判断
    is_frontal: bool = False        # 是否正脸
    yaw: float = 0.0                # 头部偏航角（度），正值向右
    pitch: float = 0.0              # 头部俯仰角（度），正值向上
    roll: float = 0.0               # 头部翻滚角（度）
    
    # 置信度和 ID
    confidence: float = 0.0         # 检测置信度
    face_id: int = -1               # 人脸 ID（跨帧追踪用，-1 表示未分配）
    
    # 时间戳
    timestamp: float = field(default_factory=time.time)
    
    def is_valid(self) -> bool:
        """检查人脸是否有效"""
        return self.width > 0 and self.height > 0 and self.confidence > 0.5
    
    def is_in_region(self, x_min: float, x_max: float, 
                     y_min: float, y_max: float) -> bool:
        """检查人脸中心是否在指定区域内"""
        cx, cy = self.center
        return x_min <= cx <= x_max and y_min <= cy <= y_max
    
    def distance_to(self, other: 'FaceInfo') -> float:
        """计算与另一个人脸中心的距离"""
        dx = self.center_x - other.center_x
        dy = self.center_y - other.center_y
        return (dx * dx + dy * dy) ** 0.5


@dataclass
class LockedTarget:
    """锁定的目标信息（传递给 TRACKING 状态）"""
    face_id: int = -1                               # 人脸 ID
    bbox: Tuple[float, float, float, float] = (0, 0, 0, 0)  # (x, y, w, h) 归一化
    center: Tuple[float, float] = (0.5, 0.5)        # 中心点 (x, y) 归一化
    lock_time: float = 0.0                          # 锁定时间戳
    
    @classmethod
    def from_face(cls, face: FaceInfo) -> 'LockedTarget':
        """从 FaceInfo 创建锁定目标"""
        return cls(
            face_id=face.face_id,
            bbox=(face.x, face.y, face.width, face.height),
            center=face.center,
            lock_time=time.time()
        )
