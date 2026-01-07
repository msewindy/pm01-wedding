"""
FSM 共享数据结构

包含感知数据、运动数据、音频数据等
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Tuple, TYPE_CHECKING
import numpy as np

from .enums import WeddingEvent

# 避免循环导入
if TYPE_CHECKING:
    from ..perception import FaceInfo, LockedTarget


@dataclass
class PerceptionData:
    """感知数据"""
    # 目标检测（基础）
    face_detected: bool = False
    face_position: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5]))  # 归一化 (x, y)
    face_distance: float = 2.0  # 估计距离 (m)
    face_count: int = 0
    
    # 手势检测
    gesture: str = "none"  # "none", "heart", "v", "ok"
    gesture_confidence: float = 0.0
    
    # 安全检测
    too_close: bool = False
    
    # === SEARCH 状态扩展（简化版） ===
    
    # 人脸列表（所有检测到的人脸）
    faces: List['FaceInfo'] = field(default_factory=list)
    
    # 跟踪状态
    has_target: bool = False                # 是否有跟踪目标
    target_confirmed: bool = False          # 目标是否已确认（正脸持续 ≥ 2s）
    target_position: Tuple[float, float] = (0.5, 0.5)  # 目标位置
    is_face_visible: bool = False           # 正脸是否可见
    face_visible_duration: float = 0.0      # 正脸可见持续时间
    
    # 锁定目标（传递给 TRACKING）
    locked_target: Optional['LockedTarget'] = None
    
    # 当前跟踪的目标 face_id（用于可视化）
    target_face_id: Optional[int] = None
    
    # 当前图像帧（用于录制等功能，OpenCV格式，BGR）
    current_frame: Optional[np.ndarray] = None
    
    def reset(self) -> None:
        """重置单帧数据"""
        self.face_detected = False
        self.gesture = "none"
        self.gesture_confidence = 0.0
        # 注意：faces 不在这里重置，由感知节点更新
    
    def reset_tracking(self) -> None:
        """重置跟踪状态"""
        self.faces.clear()
        self.has_target = False
        self.target_confirmed = False
        self.target_position = (0.5, 0.5)
        self.is_face_visible = False
        self.face_visible_duration = 0.0
        self.locked_target = None
        self.target_face_id = None


@dataclass
class MotionData:
    """运动数据"""
    # 当前关节状态
    joint_positions: np.ndarray = field(default_factory=lambda: np.zeros(24))
    
    # 目标关节命令
    joint_commands: np.ndarray = field(default_factory=lambda: np.zeros(24))
    
    # 当前 Pose 名称
    current_pose: str = "neutral"
    target_pose: str = "neutral"
    
    # 注视目标 (归一化坐标)
    look_at_target: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5]))
    
    # Pose 是否正在执行
    pose_in_progress: bool = False


@dataclass
class AudioData:
    """音频数据"""
    # 语音命令
    voice_command: str = "none"  # "none", "photo", "interview", "stop"
    
    # 当前播放状态
    is_playing: bool = False
    current_speech: str = ""
    
    # 待播放队列
    pending_speech: str = ""


@dataclass
class CameraData:
    """相机数据"""
    # 相机内参（从 camera_info 获取）
    fx: float = 0.0  # 焦距 x（像素）
    fy: float = 0.0  # 焦距 y（像素）
    cx: float = 0.0  # 主点 x（像素）
    cy: float = 0.0  # 主点 y（像素）
    
    # 图像尺寸
    width: int = 0  # 图像宽度（像素）
    height: int = 0  # 图像高度（像素）
    
    # 是否已初始化
    initialized: bool = False
    
    def update_from_camera_info(self, camera_info) -> None:
        """
        从 ROS2 CameraInfo 消息更新相机参数
        
        Args:
            camera_info: sensor_msgs.msg.CameraInfo 消息
        """
        # 从内参矩阵 K 提取参数（K 是 3x3 矩阵，按行主序存储）
        # K = [fx  0  cx]
        #     [0  fy  cy]
        #     [0   0   1]
        if len(camera_info.k) >= 9:
            self.fx = camera_info.k[0]
            self.fy = camera_info.k[4]
            self.cx = camera_info.k[2]
            self.cy = camera_info.k[5]
        
        # 图像尺寸
        self.width = camera_info.width
        self.height = camera_info.height
        
        # 标记为已初始化
        self.initialized = True
    
    def is_valid(self) -> bool:
        """检查相机参数是否有效"""
        return self.initialized and self.fx > 0 and self.fy > 0 and self.width > 0 and self.height > 0


@dataclass 
class WeddingFSMData:
    """
    FSM 共享数据结构
    
    包含所有状态需要访问的共享数据
    参考：engineai_humanoid/FSM_States/ControlFSMData.h
    """
    # 感知数据
    perception: PerceptionData = field(default_factory=PerceptionData)
    
    # 运动数据
    motion: MotionData = field(default_factory=MotionData)
    
    # 音频数据
    audio: AudioData = field(default_factory=AudioData)
    
    # 相机数据
    camera: CameraData = field(default_factory=CameraData)
    
    # 待处理命令（外部输入）
    pending_command: Optional[WeddingEvent] = None
    
    # 安全标志
    safety_triggered: bool = False
    
    # 配置参数
    config: Dict[str, Any] = field(default_factory=dict)
    
    def reset_pending_command(self) -> None:
        """重置待处理命令"""
        self.pending_command = None
    
    def reset_frame_data(self) -> None:
        """重置单帧数据（每帧开始时调用）"""
        self.perception.reset()
        self.audio.pending_speech = ""
