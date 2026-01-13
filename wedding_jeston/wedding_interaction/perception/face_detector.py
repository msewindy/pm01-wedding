"""
人脸检测器

支持 MediaPipe 0.10.x (Tasks API) 和 0.9.x (Solutions API)
使用人脸检测和头部姿态估计进行正脸判断
"""

import time
import logging
import os
from typing import List, Optional, Tuple
import numpy as np

try:
    import cv2
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# 检测 MediaPipe 版本和可用的 API
MEDIAPIPE_AVAILABLE = False
MEDIAPIPE_LEGACY = False  # 是否使用旧版 solutions API
MEDIAPIPE_TASKS = False   # 是否使用新版 tasks API

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
    
    # 检查是否有 solutions API (旧版 0.9.x)
    if hasattr(mp, 'solutions'):
        MEDIAPIPE_LEGACY = True
    
    # 检查是否有 tasks API (新版 0.10.x)
    if hasattr(mp, 'tasks'):
        MEDIAPIPE_TASKS = True
        
except ImportError:
    pass

from .data_types import FaceInfo
from .face_id_tracker import FaceIDTracker


class FaceDetector:
    """
    人脸检测器
    
    自动检测 MediaPipe 版本并使用对应的 API
    支持人脸检测和正脸判断（基于头部姿态估计）
    """
    
    # 正脸判断阈值（度）
    FRONTAL_YAW_THRESHOLD = 30.0    # yaw 角度阈值
    FRONTAL_PITCH_THRESHOLD = 20.0  # pitch 角度阈值
    
    # 最小检测置信度
    MIN_DETECTION_CONFIDENCE = 0.5
    
    def __init__(self, 
                 use_face_mesh: bool = True,
                 min_detection_confidence: float = 0.5,
                 min_tracking_confidence: float = 0.5,
                 model_selection: int = 0,
                 force_opencv: bool = False,  # 默认使用 MediaPipe
                 enable_tracking: bool = True):  # 启用 ID 追踪
        """
        初始化人脸检测器
        
        Args:
            use_face_mesh: 是否使用 Face Mesh/Landmarker 进行正脸判断
            min_detection_confidence: 最小检测置信度
            min_tracking_confidence: 最小追踪置信度（MediaPipe 追踪模式阈值）
            model_selection: 模型选择 (0: 近距离 2m, 1: 远距离 5m)
            force_opencv: 强制使用 OpenCV Haar Cascade (对小人脸检测更稳定)
            enable_tracking: 是否启用跨帧 ID 追踪（使用 FaceIDTracker）
        """
        self.logger = logging.getLogger("FaceDetector")
        self._use_face_mesh = use_face_mesh
        self._min_detection_confidence = min_detection_confidence
        self._min_tracking_confidence = min_tracking_confidence
        self._model_selection = model_selection
        self._force_opencv = force_opencv
        self._enable_tracking = enable_tracking
        
        # ID 追踪器（用于维护跨帧持久化 ID）
        self._id_tracker = FaceIDTracker() if enable_tracking else None
        
        # MediaPipe 组件
        self._face_detector = None
        self._face_landmarker = None
        self._using_legacy = False
        self._using_opencv = False
        
        # OpenCV Haar Cascade (预加载以提高性能)
        self._haar_cascade = None
        if CV_AVAILABLE:
            # 尝试多种方式找到 haarcascade 文件
            cascade_path = None
            
            # 方法1: 使用 cv2.data.haarcascades (如果可用)
            if hasattr(cv2, 'data') and hasattr(cv2.data, 'haarcascades'):
                cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
                if not os.path.exists(cascade_path):
                    cascade_path = None
            
            # 方法2: 尝试常见的系统路径
            if cascade_path is None or not os.path.exists(cascade_path):
                possible_paths = [
                    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
                    "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
                    "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
                    "/usr/local/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
                ]
                for path in possible_paths:
                    if os.path.exists(path):
                        cascade_path = path
                        break
            
            # 方法3: 尝试从 OpenCV 安装路径查找
            if cascade_path is None or not os.path.exists(cascade_path):
                try:
                    # cv2 已在模块级别导入，直接使用
                    cv2_path = os.path.dirname(cv2.__file__)
                    possible_cascade = os.path.join(cv2_path, "data", "haarcascade_frontalface_default.xml")
                    if os.path.exists(possible_cascade):
                        cascade_path = possible_cascade
                except (AttributeError, OSError):
                    pass
            
            if cascade_path and os.path.exists(cascade_path):
                self._haar_cascade = cv2.CascadeClassifier(cascade_path)
                if self._haar_cascade.empty():
                    self.logger.warning(f"无法加载 Haar Cascade 文件: {cascade_path}")
                    self._haar_cascade = None
            else:
                self.logger.warning("无法找到 haarcascade_frontalface_default.xml 文件，OpenCV 人脸检测将不可用")
        
        # 如果强制使用 OpenCV，直接设置标志
        if force_opencv:
            self._using_opencv = True
            self.logger.info("Using OpenCV Haar Cascade (force_opencv=True)")
        elif MEDIAPIPE_AVAILABLE:
            self._init_mediapipe()
        else:
            self._using_opencv = True
            self.logger.warning("MediaPipe not available, using OpenCV Haar Cascade")
        
        if enable_tracking:
            self.logger.info("Face ID tracking enabled (using FaceIDTracker)")
    
    def _init_mediapipe(self) -> None:
        """初始化 MediaPipe 组件（自动选择 API 版本）"""
        
        if MEDIAPIPE_LEGACY:
            self._init_legacy_api()
        elif MEDIAPIPE_TASKS:
            self._init_tasks_api()
        else:
            self.logger.error("No compatible MediaPipe API found")
    
    def _init_legacy_api(self) -> None:
        """初始化旧版 Solutions API (MediaPipe 0.9.x)"""
        self._using_legacy = True
        
        mp_face_detection = mp.solutions.face_detection
        self._face_detector = mp_face_detection.FaceDetection(
            min_detection_confidence=self._min_detection_confidence,
            model_selection=self._model_selection
        )
        
        if self._use_face_mesh:
            mp_face_mesh = mp.solutions.face_mesh
            self._face_landmarker = mp_face_mesh.FaceMesh(
                static_image_mode=False,  # 启用追踪模式
                max_num_faces=5,
                refine_landmarks=True,
                min_detection_confidence=self._min_detection_confidence,
                min_tracking_confidence=self._min_tracking_confidence  # 使用配置的追踪置信度
            )
            self.logger.info(f"Face Mesh initialized with tracking "
                           f"(min_tracking_confidence={self._min_tracking_confidence})")
        
        self.logger.info("MediaPipe Legacy (Solutions) API initialized")
    
    def _init_tasks_api(self) -> None:
        """初始化新版 Tasks API (MediaPipe 0.10.x)"""
        self._using_legacy = False
        
        try:
            from mediapipe.tasks import python as mp_tasks
            from mediapipe.tasks.python import vision
            
            # 获取模型文件路径
            model_dir = self._get_model_dir()
            
            # Face Landmarker (for head pose estimation) - 优先尝试，因为它同时提供检测和姿态
            if self._use_face_mesh:
                landmarker_model_path = os.path.join(model_dir, "face_landmarker.task")
                if os.path.exists(landmarker_model_path):
                    try:
                        base_options = mp_tasks.BaseOptions(model_asset_path=landmarker_model_path)
                        options = vision.FaceLandmarkerOptions(
                            base_options=base_options,
                            num_faces=5,
                            min_face_detection_confidence=self._min_detection_confidence,
                            min_tracking_confidence=self._min_tracking_confidence  # 使用配置的追踪置信度
                        )
                        self._face_landmarker = vision.FaceLandmarker.create_from_options(options)
                        self.logger.info(f"Face Landmarker loaded from {landmarker_model_path}")
                    except Exception as e:
                        self.logger.warning(f"Face Landmarker failed: {e}")
                else:
                    self.logger.warning(f"Face Landmarker model not found: {landmarker_model_path}")
            
            # Face Detector（如果 Landmarker 失败才尝试）
            if self._face_landmarker is None:
                detector_model_path = os.path.join(model_dir, "blaze_face_short_range.tflite")
                if os.path.exists(detector_model_path):
                    try:
                        base_options = mp_tasks.BaseOptions(model_asset_path=detector_model_path)
                        options = vision.FaceDetectorOptions(
                            base_options=base_options,
                            min_detection_confidence=self._min_detection_confidence
                        )
                        self._face_detector = vision.FaceDetector.create_from_options(options)
                        self.logger.info(f"Face Detector loaded from {detector_model_path}")
                    except Exception as e:
                        self.logger.warning(f"Face Detector failed: {e}")
                else:
                    self.logger.warning(f"Face Detector model not found: {detector_model_path}")
            
            # 如果都失败，标记使用 OpenCV
            if self._face_detector is None and self._face_landmarker is None:
                self.logger.info("MediaPipe models unavailable, using OpenCV Haar Cascade")
                self._using_opencv = True
            else:
                self.logger.info("MediaPipe Tasks API initialized")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize Tasks API: {e}")
            self.logger.info("Falling back to OpenCV detection")
            self._using_opencv = True
    
    def _get_model_dir(self) -> str:
        """获取模型文件目录"""
        # 优先使用项目内的模型目录
        project_model_dir = os.path.join(
            os.path.dirname(__file__), 
            "..", "..", "models", "mediapipe"
        )
        if os.path.exists(project_model_dir):
            return project_model_dir
        
        # 使用用户目录
        home_model_dir = os.path.expanduser("~/.mediapipe/models")
        os.makedirs(home_model_dir, exist_ok=True)
        return home_model_dir
    
    def detect(self, image: np.ndarray) -> List[FaceInfo]:
        """
        检测图像中的人脸（带跨帧 ID 追踪）
        
        Args:
            image: BGR 格式的图像 (OpenCV 格式)
        
        Returns:
            检测到的人脸列表（已分配持久化 ID）
        """
        # 执行检测
        if not MEDIAPIPE_AVAILABLE or self._using_opencv:
            faces = self._detect_opencv(image)
        elif self._using_legacy:
            faces = self._detect_legacy(image)
        else:
            faces = self._detect_tasks(image)
        
        # 记录检测结果（追踪前）
        if len(faces) > 0:
            self.logger.debug(f"[FaceDetector] 检测到{len(faces)}个人脸（追踪前）:")
            for i, face in enumerate(faces):
                self.logger.debug(f"  [{i}] bbox=({face.x:.3f}, {face.y:.3f}, {face.width:.3f}, {face.height:.3f}), "
                                f"center=({face.center_x:.3f}, {face.center_y:.3f}), "
                                f"frontal={face.is_frontal}, confidence={face.confidence:.3f}")
        
        # 使用 ID 追踪器分配持久化 ID
        if self._enable_tracking and self._id_tracker is not None:
            faces = self._id_tracker.update(faces)
            
            # 记录追踪后的结果
            if len(faces) > 0:
                self.logger.debug(f"[FaceDetector] 追踪后分配IDs:")
                for face in faces:
                    self.logger.debug(f"  face_id={face.face_id}, "
                                    f"center=({face.center_x:.3f}, {face.center_y:.3f}), "
                                    f"frontal={face.is_frontal}")
        else:
            self.logger.warning("[FaceDetector] ID追踪未启用或追踪器未初始化")
        
        return faces
    
    def _detect_legacy(self, image: np.ndarray) -> List[FaceInfo]:
        """使用旧版 Solutions API 检测"""
        if self._face_detector is None:
            return self._detect_opencv(image)
        
        # 转换为 RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w = image.shape[:2]
        
        faces = []
        
        # 人脸检测
        results = self._face_detector.process(image_rgb)
        
        if results.detections:
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                
                face = FaceInfo(
                    x=max(0.0, bbox.xmin),
                    y=max(0.0, bbox.ymin),
                    width=min(1.0 - bbox.xmin, bbox.width),
                    height=min(1.0 - bbox.ymin, bbox.height),
                    confidence=detection.score[0] if detection.score else 0.0,
                    face_id=-1,  # 将由 ID 追踪器分配
                    timestamp=time.time()
                )
                
                faces.append(face)
        
        # 使用 Face Mesh 进行正脸判断
        if self._use_face_mesh and self._face_landmarker is not None and faces:
            self._estimate_head_pose_legacy(image_rgb, faces)
        
        return faces
    
    def _detect_tasks(self, image: np.ndarray) -> List[FaceInfo]:
        """使用新版 Tasks API 检测"""
        # 如果没有加载模型，使用 OpenCV fallback
        if self._face_detector is None and self._face_landmarker is None:
            return self._detect_opencv(image)
        
        # 转换为 RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w = image.shape[:2]
        
        faces = []
        
        # 优先使用 Face Landmarker（同时提供检测和姿态）
        if self._face_landmarker is not None:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
            results = self._face_landmarker.detect(mp_image)
            
            if results.face_landmarks:
                for i, landmarks in enumerate(results.face_landmarks):
                    # 从 landmarks 计算 bounding box
                    x_coords = [lm.x for lm in landmarks]
                    y_coords = [lm.y for lm in landmarks]
                    
                    x_min = max(0.0, min(x_coords))
                    x_max = min(1.0, max(x_coords))
                    y_min = max(0.0, min(y_coords))
                    y_max = min(1.0, max(y_coords))
                    
                    face = FaceInfo(
                        x=x_min,
                        y=y_min,
                        width=x_max - x_min,
                        height=y_max - y_min,
                        confidence=0.9,  # Landmarker 不提供置信度
                        face_id=-1,  # 将由 ID 追踪器分配
                        timestamp=time.time()
                    )
                    
                    # 直接计算头部姿态
                    self._compute_head_pose_from_landmarks(face, landmarks)
                    
                    faces.append(face)
        
        # 如果没有 Landmarker，使用 Face Detector
        elif self._face_detector is not None:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
            results = self._face_detector.detect(mp_image)
            
            if results.detections:
                for detection in results.detections:
                    bbox = detection.bounding_box
                    
                    face = FaceInfo(
                        x=bbox.origin_x / w,
                        y=bbox.origin_y / h,
                        width=bbox.width / w,
                        height=bbox.height / h,
                        confidence=detection.categories[0].score if detection.categories else 0.0,
                        face_id=-1,  # 将由 ID 追踪器分配
                        timestamp=time.time()
                    )
                    
                    # 无法估计头部姿态，默认正脸
                    face.is_frontal = True
                    
                    faces.append(face)
        
        return faces
    
    def _detect_opencv(self, image: np.ndarray) -> List[FaceInfo]:
        """使用 OpenCV Haar Cascade 检测（对小人脸更稳定）"""
        if not CV_AVAILABLE or self._haar_cascade is None:
            return self._mock_detect(image)
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h, w = image.shape[:2]
        
        # 使用更严格的参数以减少误检测
        detections = self._haar_cascade.detectMultiScale(
            gray, 
            scaleFactor=1.1, 
            minNeighbors=6,   # 增加以减少误检测
            minSize=(40, 40), # 增大最小尺寸以过滤噪声
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        faces = []
        for (x, y, fw, fh) in detections:
            face = FaceInfo(
                x=x / w,
                y=y / h,
                width=fw / w,
                height=fh / h,
                confidence=0.8,  # Haar 不提供置信度
                face_id=-1,  # 将由 ID 追踪器分配
                timestamp=time.time(),
                is_frontal=True  # Haar frontalface 只检测正脸
            )
            faces.append(face)
        
        return faces
    
    def _estimate_head_pose_legacy(self, image_rgb: np.ndarray, 
                                    faces: List[FaceInfo]) -> None:
        """使用旧版 Face Mesh 估计头部姿态"""
        results = self._face_landmarker.process(image_rgb)
        
        if not results.multi_face_landmarks:
            return
        
        # 为每个 Face Mesh 结果计算 bounding box
        mesh_bboxes = []
        for face_landmarks in results.multi_face_landmarks:
            bbox = self._get_mesh_bbox_legacy(face_landmarks)
            mesh_bboxes.append(bbox)
        
        # 使用 IoU 匹配 Face Detection 和 Face Mesh
        matched_pairs = self._match_faces_by_iou(faces, mesh_bboxes)
        
        # 为匹配到的人脸估计头部姿态
        for face_idx, mesh_idx in matched_pairs:
            face = faces[face_idx]
            face_landmarks = results.multi_face_landmarks[mesh_idx]
            self._compute_head_pose_legacy(face, face_landmarks)
    
    def _get_mesh_bbox_legacy(self, face_landmarks) -> Tuple[float, float, float, float]:
        """从旧版 Face Mesh landmarks 计算 bounding box"""
        x_coords = [lm.x for lm in face_landmarks.landmark]
        y_coords = [lm.y for lm in face_landmarks.landmark]
        
        x_min = min(x_coords)
        x_max = max(x_coords)
        y_min = min(y_coords)
        y_max = max(y_coords)
        
        return (x_min, y_min, x_max - x_min, y_max - y_min)
    
    def _compute_head_pose_legacy(self, face: FaceInfo, face_landmarks) -> None:
        """从旧版 Face Mesh landmarks 计算头部姿态"""
        try:
            # 使用关键点估计头部姿态
            nose_tip = face_landmarks.landmark[1]
            left_eye = face_landmarks.landmark[33]
            right_eye = face_landmarks.landmark[263]
            chin = face_landmarks.landmark[152]
            forehead = face_landmarks.landmark[10]
            
            self._compute_pose_from_points(
                face, nose_tip, left_eye, right_eye, chin, forehead
            )
            
        except (IndexError, AttributeError) as e:
            self.logger.debug(f"Head pose estimation failed: {e}")
            face.is_frontal = True
    
    def _compute_head_pose_from_landmarks(self, face: FaceInfo, landmarks) -> None:
        """从新版 Face Landmarker 结果计算头部姿态"""
        try:
            # 新版 API 的 landmarks 是列表
            nose_tip = landmarks[1]
            left_eye = landmarks[33]
            right_eye = landmarks[263]
            chin = landmarks[152]
            forehead = landmarks[10]
            
            self._compute_pose_from_points(
                face, nose_tip, left_eye, right_eye, chin, forehead
            )
            
        except (IndexError, AttributeError) as e:
            self.logger.debug(f"Head pose estimation failed: {e}")
            face.is_frontal = True
    
    def _compute_pose_from_points(self, face: FaceInfo, 
                                   nose_tip, left_eye, right_eye, 
                                   chin, forehead) -> None:
        """从关键点计算头部姿态"""
        # Yaw 估计：基于鼻尖相对于两眼中点的水平偏移
        eye_center_x = (left_eye.x + right_eye.x) / 2
        eye_distance = abs(right_eye.x - left_eye.x)
        
        if eye_distance > 0.01:
            yaw_offset = (nose_tip.x - eye_center_x) / eye_distance
            face.yaw = yaw_offset * 90
        else:
            face.yaw = 0.0
        
        # Pitch 估计：基于鼻尖和眼睛的垂直关系
        eye_center_y = (left_eye.y + right_eye.y) / 2
        face_height = abs(chin.y - forehead.y)
        
        if face_height > 0.01:
            expected_nose_offset = face_height * 0.3
            actual_nose_offset = nose_tip.y - eye_center_y
            pitch_offset = (actual_nose_offset - expected_nose_offset) / face_height
            face.pitch = pitch_offset * 90
        else:
            face.pitch = 0.0
        
        # Roll 估计：基于两眼的倾斜角度
        if eye_distance > 0.01:
            roll_offset = (right_eye.y - left_eye.y) / eye_distance
            face.roll = np.arctan(roll_offset) * 180 / np.pi
        else:
            face.roll = 0.0
        
        # 正脸判断
        face.is_frontal = (
            abs(face.yaw) < self.FRONTAL_YAW_THRESHOLD and
            abs(face.pitch) < self.FRONTAL_PITCH_THRESHOLD
        )
    
    def _match_faces_by_iou(self, faces: List[FaceInfo], 
                           mesh_bboxes: List[Tuple[float, float, float, float]],
                           iou_threshold: float = 0.3) -> List[Tuple[int, int]]:
        """使用 IoU 匹配 Face Detection 结果和 Face Mesh 结果"""
        if not faces or not mesh_bboxes:
            return []
        
        n_faces = len(faces)
        n_meshes = len(mesh_bboxes)
        iou_matrix = np.zeros((n_faces, n_meshes))
        
        for i, face in enumerate(faces):
            face_bbox = (face.x, face.y, face.width, face.height)
            for j, mesh_bbox in enumerate(mesh_bboxes):
                iou_matrix[i, j] = self._calculate_iou(face_bbox, mesh_bbox)
        
        # 贪婪匹配
        matched_pairs = []
        used_faces = set()
        used_meshes = set()
        
        while True:
            max_iou = iou_threshold
            best_pair = None
            
            for i in range(n_faces):
                if i in used_faces:
                    continue
                for j in range(n_meshes):
                    if j in used_meshes:
                        continue
                    if iou_matrix[i, j] > max_iou:
                        max_iou = iou_matrix[i, j]
                        best_pair = (i, j)
            
            if best_pair is None:
                break
            
            matched_pairs.append(best_pair)
            used_faces.add(best_pair[0])
            used_meshes.add(best_pair[1])
        
        return matched_pairs
    
    def _calculate_iou(self, bbox1: Tuple[float, float, float, float],
                       bbox2: Tuple[float, float, float, float]) -> float:
        """计算两个 bounding box 的 IoU"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
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
    
    def _mock_detect(self, image: np.ndarray) -> List[FaceInfo]:
        """Mock 检测（用于测试）"""
        return [
            FaceInfo(
                x=0.3,
                y=0.2,
                width=0.2,
                height=0.25,
                is_frontal=True,
                yaw=0.0,
                pitch=0.0,
                roll=0.0,
                confidence=0.9,
                face_id=-1,  # Mock 检测不追踪
                timestamp=time.time()
            )
        ]
    
    def get_tracking_stats(self) -> dict:
        """获取追踪统计信息"""
        if self._id_tracker is not None:
            return self._id_tracker.get_stats()
        return {}
    
    def select_best_face(self, faces: List[FaceInfo],
                         valid_region: Tuple[float, float, float, float] = (0.15, 0.85, 0.1, 0.9),
                         min_area: float = 0.01,
                         max_area: float = 0.30) -> Optional[FaceInfo]:
        """
        从多个人脸中选择最佳目标
        
        优先级：正脸 > 有效区域内 > 面积合理 > 面积最大
        """
        x_min, x_max, y_min, y_max = valid_region
        
        valid_faces = [
            f for f in faces
            if f.is_frontal
            and f.is_in_region(x_min, x_max, y_min, y_max)
            and min_area <= f.area <= max_area
        ]
        
        if not valid_faces:
            return None
        
        return max(valid_faces, key=lambda f: f.area)
    
    def release(self) -> None:
        """释放资源"""
        if self._face_detector is not None:
            if hasattr(self._face_detector, 'close'):
                self._face_detector.close()
            self._face_detector = None
        
        if self._face_landmarker is not None:
            if hasattr(self._face_landmarker, 'close'):
                self._face_landmarker.close()
            self._face_landmarker = None
        
        if self._id_tracker is not None:
            self._id_tracker.reset()
        
        self.logger.info("FaceDetector released")
