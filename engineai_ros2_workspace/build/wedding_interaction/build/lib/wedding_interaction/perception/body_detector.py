"""
人体检测器

支持 MediaPipe 0.10.x (Tasks API) 和 0.9.x (Solutions API)
以及 OpenCV HOG 作为后备
"""

import time
import logging
import os
from typing import List, Optional
import numpy as np

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# 检测 MediaPipe 版本和可用的 API
MEDIAPIPE_AVAILABLE = False
MEDIAPIPE_LEGACY = False
MEDIAPIPE_TASKS = False

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
    
    if hasattr(mp, 'solutions'):
        MEDIAPIPE_LEGACY = True
    
    if hasattr(mp, 'tasks'):
        MEDIAPIPE_TASKS = True
        
except ImportError:
    pass

from .data_types import BodyInfo, FaceInfo


class BodyDetector:
    """
    人体检测器
    
    自动检测 MediaPipe 版本并使用对应的 API
    支持人脸-人体关联
    """
    
    # 最小检测置信度
    MIN_DETECTION_CONFIDENCE = 0.5
    MIN_TRACKING_CONFIDENCE = 0.5
    
    def __init__(self,
                 min_detection_confidence: float = 0.5,
                 min_tracking_confidence: float = 0.5,
                 model_complexity: int = 1):
        """
        初始化人体检测器
        
        Args:
            min_detection_confidence: 最小检测置信度
            min_tracking_confidence: 最小跟踪置信度
            model_complexity: 模型复杂度 (0: Lite, 1: Full, 2: Heavy)
        """
        self.logger = logging.getLogger("BodyDetector")
        self._min_detection_confidence = min_detection_confidence
        self._min_tracking_confidence = min_tracking_confidence
        self._model_complexity = model_complexity
        
        # 人体 ID 计数器
        self._next_body_id = 0
        
        # 检测器组件
        self._pose = None
        self._pose_landmarker = None
        self._hog_detector = None
        self._upper_body_cascade = None
        self._using_legacy = False
        self._using_hog = False
        
        self._init_detector()
    
    def _init_detector(self) -> None:
        """初始化检测器（自动选择最佳可用方案）"""
        
        if MEDIAPIPE_LEGACY:
            self._init_legacy_api()
        elif MEDIAPIPE_TASKS:
            self._init_tasks_api()
        else:
            self._init_opencv_hog()
    
    def _init_legacy_api(self) -> None:
        """初始化旧版 Solutions API (MediaPipe 0.9.x)"""
        self._using_legacy = True
        
        try:
            mp_pose = mp.solutions.pose
            self._pose = mp_pose.Pose(
                static_image_mode=False,
                model_complexity=self._model_complexity,
                smooth_landmarks=True,
                enable_segmentation=False,
                min_detection_confidence=self._min_detection_confidence,
                min_tracking_confidence=self._min_tracking_confidence
            )
            self.logger.info("MediaPipe Legacy Pose initialized")
        except Exception as e:
            self.logger.error(f"Failed to init Legacy Pose: {e}")
            self._init_opencv_hog()
    
    def _init_tasks_api(self) -> None:
        """初始化新版 Tasks API (MediaPipe 0.10.x)"""
        self._using_legacy = False
        
        try:
            from mediapipe.tasks import python as mp_tasks
            from mediapipe.tasks.python import vision
            
            model_dir = self._get_model_dir()
            model_path = os.path.join(model_dir, "pose_landmarker.task")
            
            if os.path.exists(model_path):
                base_options = mp_tasks.BaseOptions(model_asset_path=model_path)
                options = vision.PoseLandmarkerOptions(
                    base_options=base_options,
                    num_poses=1,  # 单人检测
                    min_pose_detection_confidence=self._min_detection_confidence,
                    min_tracking_confidence=self._min_tracking_confidence
                )
                self._pose_landmarker = vision.PoseLandmarker.create_from_options(options)
                self.logger.info(f"MediaPipe Tasks Pose loaded from {model_path}")
            else:
                self.logger.warning(f"Pose model not found: {model_path}")
                self._init_opencv_hog()
                
        except Exception as e:
            self.logger.warning(f"Failed to init Tasks Pose: {e}")
            self._init_opencv_hog()
    
    def _init_opencv_hog(self) -> None:
        """初始化 OpenCV 检测器（后备方案）"""
        self._using_hog = True
        
        if CV_AVAILABLE:
            # HOG 全身检测器
            self._hog_detector = cv2.HOGDescriptor()
            self._hog_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            
            # 上半身 Haar Cascade 检测器
            upper_body_cascade = cv2.data.haarcascades + "haarcascade_upperbody.xml"
            self._upper_body_cascade = cv2.CascadeClassifier(upper_body_cascade)
            
            self.logger.info("OpenCV HOG + Upper Body Cascade initialized (fallback)")
        else:
            self._upper_body_cascade = None
            self.logger.warning("No body detection available, using mock detector")
    
    def _get_model_dir(self) -> str:
        """获取模型文件目录"""
        project_model_dir = os.path.join(
            os.path.dirname(__file__), 
            "..", "..", "models", "mediapipe"
        )
        if os.path.exists(project_model_dir):
            return project_model_dir
        
        home_model_dir = os.path.expanduser("~/.mediapipe/models")
        os.makedirs(home_model_dir, exist_ok=True)
        return home_model_dir
    
    def detect(self, image: np.ndarray) -> List[BodyInfo]:
        """
        检测图像中的人体
        
        Args:
            image: BGR 格式的图像 (OpenCV 格式)
        
        Returns:
            检测到的人体列表
        """
        if self._using_hog:
            return self._detect_hog(image)
        elif self._using_legacy:
            return self._detect_legacy(image)
        else:
            return self._detect_tasks(image)
    
    def _detect_legacy(self, image: np.ndarray) -> List[BodyInfo]:
        """使用旧版 Solutions API 检测"""
        if self._pose is None:
            return self._mock_detect(image)
        
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        bodies = []
        
        results = self._pose.process(image_rgb)
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            body = self._create_body_from_landmarks_legacy(landmarks)
            if body:
                bodies.append(body)
        
        return bodies
    
    def _detect_tasks(self, image: np.ndarray) -> List[BodyInfo]:
        """使用新版 Tasks API 检测"""
        if self._pose_landmarker is None:
            return self._detect_hog(image)
        
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        bodies = []
        
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
        results = self._pose_landmarker.detect(mp_image)
        
        if results.pose_landmarks:
            for landmarks in results.pose_landmarks:
                body = self._create_body_from_landmarks_tasks(landmarks)
                if body:
                    bodies.append(body)
        
        return bodies
    
    def _detect_hog(self, image: np.ndarray) -> List[BodyInfo]:
        """使用 OpenCV HOG + 上半身检测"""
        if self._hog_detector is None and self._upper_body_cascade is None:
            return self._mock_detect(image)
        
        h, w = image.shape[:2]
        bodies = []
        
        # 1. 首先尝试上半身检测（更适合室内/摄像头场景）
        if self._upper_body_cascade is not None:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            upper_bodies = self._upper_body_cascade.detectMultiScale(
                gray, scaleFactor=1.05, minNeighbors=2, 
                minSize=(40, 40), maxSize=(int(w*0.9), int(h*0.9))
            )
            
            for (x, y, bw, bh) in upper_bodies:
                body = BodyInfo(
                    x=x / w,
                    y=y / h,
                    width=bw / w,
                    height=bh / h,
                    confidence=0.75,
                    body_id=self._get_next_body_id(),
                    timestamp=time.time()
                )
                bodies.append(body)
        
        # 2. 如果上半身没检测到，尝试 HOG 全身检测
        if not bodies and self._hog_detector is not None:
            boxes, weights = self._hog_detector.detectMultiScale(
                image, winStride=(8, 8), padding=(4, 4), scale=1.05
            )
            
            for i, (x, y, bw, bh) in enumerate(boxes):
                body = BodyInfo(
                    x=x / w,
                    y=y / h,
                    width=bw / w,
                    height=bh / h,
                    confidence=float(weights[i]) if i < len(weights) else 0.8,
                    body_id=self._get_next_body_id(),
                    timestamp=time.time()
                )
                bodies.append(body)
        
        return bodies
    
    def estimate_body_from_face(self, face) -> Optional[BodyInfo]:
        """
        基于人脸位置估算上半身区域
        
        假设：人脸占上半身的约 1/3 宽度，1/4 高度
        上半身从人脸上方一点开始，到人脸下方约 2 倍人脸高度
        
        Args:
            face: FaceInfo 对象
        
        Returns:
            估算的上半身 BodyInfo
        """
        if face is None:
            return None
        
        # 估算上半身尺寸
        body_width = face.width * 3.0  # 上半身宽度约为人脸的 3 倍
        body_height = face.height * 4.0  # 上半身高度约为人脸的 4 倍
        
        # 估算上半身位置（人脸在上半身的上 1/4 中心）
        body_x = face.center_x - body_width / 2
        body_y = face.y - face.height * 0.2  # 上半身顶部在人脸上方一点
        
        # 边界限制
        body_x = max(0.0, min(1.0 - body_width, body_x))
        body_y = max(0.0, body_y)
        body_width = min(body_width, 1.0 - body_x)
        body_height = min(body_height, 1.0 - body_y)
        
        return BodyInfo(
            x=body_x,
            y=body_y,
            width=body_width,
            height=body_height,
            confidence=0.6,  # 估算的置信度较低
            body_id=self._get_next_body_id(),
            timestamp=time.time(),
            associated_face_id=face.face_id
        )
    
    def _create_body_from_landmarks_legacy(self, landmarks) -> Optional[BodyInfo]:
        """从旧版 landmarks 创建 BodyInfo"""
        x_coords = []
        y_coords = []
        
        for lm in landmarks:
            if lm.visibility > 0.5:
                x_coords.append(lm.x)
                y_coords.append(lm.y)
        
        if not x_coords or not y_coords:
            return None
        
        x_min = max(0.0, min(x_coords) - 0.05)
        x_max = min(1.0, max(x_coords) + 0.05)
        y_min = max(0.0, min(y_coords) - 0.05)
        y_max = min(1.0, max(y_coords) + 0.05)
        
        confidence = sum(lm.visibility for lm in landmarks) / len(landmarks)
        
        return BodyInfo(
            x=x_min,
            y=y_min,
            width=x_max - x_min,
            height=y_max - y_min,
            confidence=confidence,
            body_id=self._get_next_body_id(),
            timestamp=time.time()
        )
    
    def _create_body_from_landmarks_tasks(self, landmarks) -> Optional[BodyInfo]:
        """从新版 landmarks 创建 BodyInfo"""
        x_coords = [lm.x for lm in landmarks if lm.visibility > 0.5]
        y_coords = [lm.y for lm in landmarks if lm.visibility > 0.5]
        
        if not x_coords or not y_coords:
            return None
        
        x_min = max(0.0, min(x_coords) - 0.05)
        x_max = min(1.0, max(x_coords) + 0.05)
        y_min = max(0.0, min(y_coords) - 0.05)
        y_max = min(1.0, max(y_coords) + 0.05)
        
        confidence = sum(lm.visibility for lm in landmarks) / len(landmarks)
        
        return BodyInfo(
            x=x_min,
            y=y_min,
            width=x_max - x_min,
            height=y_max - y_min,
            confidence=confidence,
            body_id=self._get_next_body_id(),
            timestamp=time.time()
        )
    
    def _mock_detect(self, image: np.ndarray) -> List[BodyInfo]:
        """Mock 检测（用于测试）"""
        return [
            BodyInfo(
                x=0.25,
                y=0.1,
                width=0.3,
                height=0.7,
                confidence=0.9,
                body_id=self._get_next_body_id(),
                timestamp=time.time()
            )
        ]
    
    def _get_next_body_id(self) -> int:
        """获取下一个人体 ID"""
        body_id = self._next_body_id
        self._next_body_id += 1
        return body_id
    
    def associate_faces(self, bodies: List[BodyInfo], 
                        faces: List[FaceInfo]) -> None:
        """
        将人脸与人体关联
        
        逻辑：人脸中心应该在人体边界框的上半部分
        """
        for body in bodies:
            best_face: Optional[FaceInfo] = None
            best_overlap = 0.0
            
            for face in faces:
                fx, fy = face.center
                if body.contains_point(fx, fy, region="upper"):
                    overlap = self._calculate_overlap(face, body)
                    if overlap > best_overlap:
                        best_overlap = overlap
                        best_face = face
            
            if best_face is not None:
                body.associated_face_id = best_face.face_id
    
    def _calculate_overlap(self, face: FaceInfo, body: BodyInfo) -> float:
        """计算人脸与人体的重叠度"""
        fx, fy = face.center
        bx, by = body.center
        
        distance = ((fx - bx) ** 2 + (fy - by) ** 2) ** 0.5
        max_distance = 0.5
        overlap = max(0.0, 1.0 - distance / max_distance)
        
        return overlap
    
    def find_body_for_face(self, face: FaceInfo, 
                           bodies: List[BodyInfo]) -> Optional[BodyInfo]:
        """为人脸找到对应的人体"""
        fx, fy = face.center
        
        for body in bodies:
            if body.contains_point(fx, fy, region="upper"):
                return body
        
        return None
    
    def release(self) -> None:
        """释放资源"""
        if self._pose is not None:
            if hasattr(self._pose, 'close'):
                self._pose.close()
            self._pose = None
        
        if self._pose_landmarker is not None:
            if hasattr(self._pose_landmarker, 'close'):
                self._pose_landmarker.close()
            self._pose_landmarker = None
        
        self._hog_detector = None
        
        self.logger.info("BodyDetector released")
