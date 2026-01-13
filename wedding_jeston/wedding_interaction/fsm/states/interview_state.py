"""
INTERVIEW 状态（采访）

行为：
- 跟随目标（复用基类方法）
- 右手拿麦克风动作
- 录制视频+音频
- 执行固定对话逻辑
"""

from typing import TYPE_CHECKING, Optional
import os

from ..enums import WeddingStateName, WeddingEvent
from ..wedding_state import WeddingState
from .interview_recorder import InterviewRecorder

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM
    from ...perception import FaceInfo


class InterviewState(WeddingState):
    """
    采访状态
    
    行为：
    - 跟随目标（复用基类方法）
    - 右手拿麦克风动作
    - 录制视频+音频
    - 执行固定对话逻辑
    """
    
    # 采访流程阶段
    PHASE_START = "start"
    PHASE_GREETING = "greeting"
    PHASE_QUESTION = "question"
    PHASE_LISTENING = "listening"
    PHASE_ENDING = "ending"
    PHASE_DONE = "done"
    
    def __init__(self, fsm: 'WeddingFSM'):
        super().__init__(fsm, WeddingStateName.INTERVIEW)
        
        # 跟随参数（与 TRACKING 相同）
        self.follow_smooth = 0.25
        
        # 采访流程阶段
        self.phase = self.PHASE_START
        self.phase_start_time = 0.0
        self.listening_start_time = 0.0
        
        # 录制管理
        self._recorder: Optional[InterviewRecorder] = None
        self._recording_path = None
        
        # 对话逻辑
        self._question_index = 0
        self._questions = [
            "您好，我是今天的特约记者，能送给新人一句祝福吗？",
            "还有什么想对新人说的吗？",
            "最后，请对着镜头挥挥手吧！"
        ]
        
        # 时间参数（可从配置读取）
        self.greeting_duration = 3.0      # 开场白时长（秒）
        self.question_duration = 2.0      # 问题播放时长（秒）
        self.listening_timeout = 8.0     # 监听超时（秒）
        self.silence_threshold = 1.5      # 静音检测阈值（秒）
        self.has_spoken = False           # 记录用户是否已开口
        self.ending_duration = 2.0        # 结束语时长（秒）
        self.done_duration = 1.0          # 完成后等待时长（秒）
        self._voice_threshold = 0.02  # 音频能量阈值 (需要根据实际 mic 调整)
        
        # 录制参数
        self.recording_save_path = "/tmp/wedding_blessings/"  # 默认保存路径
        
        # 跟踪目标信息（使用基类的公共属性）
        self._match_distance_threshold = 0.2  # 位置匹配距离阈值
    
    def on_enter(self) -> None:
        """进入采访状态"""
        self.log("Entering INTERVIEW state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 1. 启动录制
        self._start_recording()
        
        # 2. 设置麦克风动作
        self.set_pose("interview_mic_hold")
        
        # 3. 初始化对话状态
        self.phase = self.PHASE_START
        self.phase_start_time = self.enter_time
        self._question_index = 0
        
        # 获取跟踪目标信息（从 TRACKING 状态传递）
        locked_target = self.fsm.data.perception.locked_target
        if locked_target:
            self._tracked_face_id = locked_target.face_id
            self._tracked_position = locked_target.center
            self.fsm.data.perception.target_face_id = locked_target.face_id
            self.log(f"Interview target: face_id={self._tracked_face_id}, "
                    f"pos=({self._tracked_position[0]:.2f}, {self._tracked_position[1]:.2f})")
        else:
            self.log("No locked target provided for interview")
            self._tracked_face_id = None
            self._tracked_position = None
            self.has_target = False
            return
            
        self.has_target = True
        
        # 重置 PID 状态
        if hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
        
        # 初始化语音活动检测
        self._last_voice_time = self.fsm.get_current_time()

    
    def run(self) -> None:
        """执行采访逻辑"""
        # 1. 跟随目标（复用基类方法）
        target_face = self._find_tracked_face(self.fsm.data.perception.faces)
        
        if target_face is not None:
            # 更新跟踪位置
            self._tracked_position = (target_face.center_x, target_face.center_y)
            # 更新可视化用的target_face_id
            self.fsm.data.perception.target_face_id = target_face.face_id
            
            # 使用 PID 控制跟随（与 TRACKING 状态相同）
            self.pid_follow_target_face(target_face, "[INTERVIEW跟随]")
            
            # 更新目标丢失检测
            self.update_target_lost_time(has_target=True)
            self.has_target = True
        else:
            # 未找到目标
            self.fsm.data.perception.target_face_id = None
            self.update_target_lost_time(has_target=False)
            self.has_target = False
        
        # 2. 执行对话流程
        self._execute_interview_flow()
        
        # 3. 录制数据写入（如果使用摄像头模式，不需要手动写入）
        # 如果使用 frame_callback 模式，需要确保回调函数被调用
        # 这里主要用于检查录制状态
        if self._recorder and self._recorder.is_recording:
            # 检查录制时长（防止超时）
            duration = self._recorder.recording_duration
            if duration >= self._recorder.max_duration:
                self.log("Recording reached max duration")
        
        self.iter_count += 1
    
    def _find_tracked_face(self, faces: list) -> Optional['FaceInfo']:
        """
        查找跟踪的目标脸
        
        使用基类的统一匹配方法，通过 face_id 或位置匹配，允许侧脸
        
        Args:
            faces: 当前帧检测到的人脸列表
        
        Returns:
            匹配的目标人脸（可以是侧脸），或 None
        """
        matched = self.find_tracked_face_by_id_and_position(
            faces, self._tracked_face_id, self._tracked_position, self._match_distance_threshold
        )
        
        # 更新 face_id（如果之前没有）
        if matched is not None and (self._tracked_face_id is None or self._tracked_face_id < 0):
            self._tracked_face_id = matched.face_id
        
        return matched
    
    def _execute_interview_flow(self) -> None:
        """执行采访流程"""
        elapsed = self.get_elapsed_time()
        phase_elapsed = self.fsm.get_current_time() - self.phase_start_time
        
        if self.phase == self.PHASE_START:
            # 播放开场白
            self.set_speech("interview_start")
            self._enter_phase(self.PHASE_GREETING)
            
        elif self.phase == self.PHASE_GREETING:
            # 等待开场白播放完成
            if phase_elapsed >= self.greeting_duration:
                self._enter_phase(self.PHASE_QUESTION)
                self._ask_question()
                
        elif self.phase == self.PHASE_QUESTION:
            # 等待问题播放完成
            if phase_elapsed >= self.question_duration:
                self._enter_phase(self.PHASE_LISTENING)
                self.listening_start_time = self.fsm.get_current_time()
                # 重置语音活动计时
                self._last_voice_time = self.fsm.get_current_time()
                self.has_spoken = False
                
        elif self.phase == self.PHASE_LISTENING:
            # 监听用户回答
            # 检测语音停顿或超时
            silence_duration = self._get_silence_duration()
            listening_duration = self.fsm.get_current_time() - self.listening_start_time
            
            # 判断是否完成回答：
            # 1. 用户已经开口 (has_spoken=True) 且 检测到足够长的静音
            # 2. 只有在超时的情况下，才允许没开口直接结束
            
            answered = self.has_spoken and (silence_duration > self.silence_threshold)
            timeout = listening_duration > self.listening_timeout
            
            if answered or timeout:
                if answered:
                    self.log(f"Answer detected (silence: {silence_duration:.2f}s)")
                else:
                    self.log(f"Listening timeout (duration: {listening_duration:.2f}s)")
                    
                # 用户回答完成或超时
                if self._question_index < len(self._questions):
                    # 还有问题，继续提问
                    self._enter_phase(self.PHASE_QUESTION)
                    self._ask_question()
                else:
                    # 问题问完了，结束
                    self._enter_phase(self.PHASE_ENDING)
                    self.set_speech("interview_thanks")
                
        elif self.phase == self.PHASE_ENDING:
            # 等待结束语播放完成
            if phase_elapsed >= self.ending_duration:
                self._enter_phase(self.PHASE_DONE)
                
        elif self.phase == self.PHASE_DONE:
            # 等待一小段时间后退出
            if phase_elapsed >= self.done_duration:
                pass  # check_transition 会处理
    
    def _enter_phase(self, new_phase: str) -> None:
        """进入新阶段"""
        self.phase = new_phase
        self.phase_start_time = self.fsm.get_current_time()
        self.log(f"Interview phase: {new_phase}")
    
    def _ask_question(self) -> None:
        """提问"""
        if self._question_index < len(self._questions):
            question = self._questions[self._question_index]
            # 注意：音频文件是 interview_question_1.mp3, interview_question_2.mp3 等（从1开始）
            # 所以索引要+1
            speech_id = f"interview_question_{self._question_index + 1}"
            self.set_speech(speech_id)
            
            # 根据文本长度动态估算时长
            # 基础 1.5s + 每个字 0.25s
            est_duration = 1.5 + len(question) * 0.25
            self.question_duration = max(2.0, est_duration)
            
            self.log(f"Asking question {self._question_index + 1}: {question} (est_time: {self.question_duration:.2f}s)")
            self._question_index += 1
        else:
            # 所有问题都问完了，直接进入结束阶段
            self._enter_phase(self.PHASE_ENDING)
            self.set_speech("interview_thanks")
    
    def _get_silence_duration(self) -> float:
        """
        获取静音持续时间
        
        TODO: 实现实际的语音活动检测（VAD）
        目前返回一个固定值，表示没有检测到静音
        
        Returns:
            静音持续时间（秒）
        """
        # TODO: 实现 VAD 检测
        # 这里需要从音频数据中检测静音
        # 暂时返回0，表示没有静音
        # 从录制器获取当前能量
        if self._recorder:
            energy = self._recorder.get_current_audio_energy()
            current_time = self.fsm.get_current_time()
            
            # 简单的能量阈值检测
            if energy > self._voice_threshold:
                self._last_voice_time = current_time
                if not self.has_spoken and self.phase == self.PHASE_LISTENING:
                    self.has_spoken = True
                    self.log(f"Voice started: energy={energy:.4f}")
            
            return current_time - self._last_voice_time
            
        return 0.0
    
    def _start_recording(self) -> None:
        """启动录制"""
        try:
            # 确保保存目录存在
            if os.path.isdir(self.recording_save_path):
                os.makedirs(self.recording_save_path, exist_ok=True)
                self.log(f"Recording save directory: {self.recording_save_path}")
            
            # 检查依赖并显示详细信息
            deps_ok = InterviewRecorder.check_dependencies()
            if not deps_ok:
                self.log("Warning: Some dependencies are missing, recording may fail")
            
            # 测试视频帧回调
            test_frame = self._get_video_frame()
            if test_frame is None:
                self.log("Warning: Video frame callback returns None, video recording may fail")
            else:
                self.log(f"Video frame callback OK, frame shape: {test_frame.shape}")
            
            # 创建录制器
            # 使用回调模式从 ROS2 topic 获取视频帧
            # 音频直接从麦克风读取（AudioRecorder内部实现）
            
            # 获取原始图像分辨率（如果可用）
            test_frame = self._get_video_frame()
            if test_frame is not None:
                original_height, original_width = test_frame.shape[:2]
                # 使用原始分辨率，避免不必要的缩放损失
                video_size = (original_width, original_height)
                self.log(f"Using original video resolution: {video_size}")
            else:
                # 回退到默认分辨率
                video_size = (640, 480)
                self.log(f"Using default video resolution: {video_size}")
            
            self._recorder = InterviewRecorder(
                save_path=self.recording_save_path,
                video_size=video_size,  # 使用原始分辨率
                video_fps=30,
                camera_index=None,  # 不使用摄像头，使用回调模式
                frame_callback=self._get_video_frame,  # 从ROS2 topic获取视频帧
                video_quality=18,  # 高质量：CRF 18（23 是中等质量）
                logger=self.logger
            )
            
            # 启动录制
            success = self._recorder.start()
            if success:
                self.log(f"Recording started: {self._recorder._actual_save_path}")
                # 检查实际启动的录制器
                if self._recorder._video_recorder:
                    self.log("Video recorder started successfully")
                else:
                    self.log("Warning: Video recorder not started")
                if self._recorder._audio_recorder:
                    self.log("Audio recorder started successfully")
                else:
                    self.log("Warning: Audio recorder not started (PyAudio may not be available)")
            else:
                self.log("Warning: Failed to start recording")
                self._recorder = None
                
        except Exception as e:
            self.log(f"Error starting recording: {e}")
            import traceback
            self.log(f"Traceback: {traceback.format_exc()}")
            self._recorder = None
    
    def _get_video_frame(self):
        """
        获取视频帧（用于回调模式）
        
        从 FSM 数据中获取最新的图像帧（由 WeddingFSMNode 从 ROS2 topic 更新）
        
        Returns:
            numpy.ndarray: OpenCV BGR 图像，或 None
        """
        # 从 FSM 数据获取最新图像帧
        if hasattr(self.fsm.data.perception, 'current_frame'):
            return self.fsm.data.perception.current_frame
        return None
    
    def _stop_recording(self) -> None:
        """停止录制"""
        if self._recorder:
            self.log("Stopping recording...")
            
            # 记录临时文件路径（用于调试）
            if hasattr(self._recorder, '_temp_video') and self._recorder._temp_video:
                self.log(f"Temp video file: {self._recorder._temp_video}")
            if hasattr(self._recorder, '_temp_audio') and self._recorder._temp_audio:
                self.log(f"Temp audio file: {self._recorder._temp_audio}")
            
            saved_path = self._recorder.stop()
            if saved_path:
                # 检查文件是否存在且大小>0
                if os.path.exists(saved_path):
                    file_size = os.path.getsize(saved_path)
                    file_size_mb = file_size / (1024 * 1024)
                    self.log(f"Interview recording saved: {saved_path} (size: {file_size_mb:.2f} MB)")
                    if file_size == 0:
                        self.log("Warning: Recording file is empty (0 bytes)")
                        # 删除空文件
                        try:
                            os.remove(saved_path)
                            self.log("Empty file removed")
                        except Exception as e:
                            self.log(f"Failed to remove empty file: {e}")
                else:
                    self.log(f"Warning: Recording file does not exist: {saved_path}")
            else:
                self.log("Warning: Recording was not saved (stop() returned None)")
            self._recorder = None
        else:
            self.log("No recorder to stop")
    
    def check_transition(self) -> WeddingStateName:
        """检查状态转换"""
        # 先检查基类的转换逻辑（安全/命令）
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
            
        # 如果没有目标，或者中途丢失目标，回到 IDLE
        if not hasattr(self, 'has_target') or not self.has_target:
            self.log("Target missing or lost, returning to IDLE")
            return WeddingStateName.IDLE
        
        # 采访完成 -> 回到 IDLE
        if self.phase == self.PHASE_DONE:
            phase_elapsed = self.fsm.get_current_time() - self.phase_start_time
            if phase_elapsed >= self.done_duration:
                self.log("Interview complete, returning to IDLE")
                return WeddingStateName.IDLE
        
        # 外部命令中断，也建议回到 IDLE 或者 TRACKING，这里保持原来的逻辑或者随之修改
        # 如果是 STOP 命令，上面基类已经处理了回到 IDLE
        # 如果是其他情况，保持不变

        
        # 外部命令中断
        if self.fsm.data.pending_command == WeddingEvent.CMD_STOP:
            self.log("Interview interrupted by command")
            return WeddingStateName.TRACKING
        
        return self.state_name
    
    def on_exit(self) -> None:
        """退出采访状态"""
        self.log("Exiting INTERVIEW state")
        
        # 1. 停止录制
        self._stop_recording()
        
        # 2. 恢复动作
        self.set_pose("neutral")
        
        # 3. 重置状态
        self.phase = self.PHASE_START
        self._question_index = 0

