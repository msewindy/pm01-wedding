"""
INTERVIEW 状态（采访）

行为：
- 执行采访动作 (interview_hold)
- 跟随目标 (PID)
- 右手拿麦克风动作 (包含在动作包中)
- 录制视频+音频
- 执行固定对话逻辑
"""

import os
from typing import TYPE_CHECKING, Optional

from ..enums import WeddingStateName, WeddingEvent
from ..wedding_state import WeddingState
from .interview_recorder import InterviewRecorder

if TYPE_CHECKING:
    from ..wedding_fsm import WeddingFSM
    from ...perception import FaceInfo


class InterviewState(WeddingState):
    """
    采访状态
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
        
        # 采访流程阶段
        self.phase = self.PHASE_START
        self.phase_start_time = 0.0
        self.listening_start_time = 0.0
        
        # 录制管理
        self._recorder: Optional[InterviewRecorder] = None
        
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
        self.listening_timeout = 8.0      # 监听超时（秒）
        self.silence_threshold = 1.5      # 静音检测阈值（秒）
        self.has_spoken = False           # 记录用户是否已开口
        self.ending_duration = 2.0        # 结束语时长（秒）
        self.done_duration = 1.0          # 完成后等待时长（秒）
        self._voice_threshold = 0.02      # 音频能量阈值
        
        # 录制参数
        self.recording_save_path = "/tmp/wedding_blessings/"
        
        # 跟踪目标信息
        self._match_distance_threshold = 0.2
    
    def on_enter(self) -> None:
        """进入采访状态"""
        self.log("Entering INTERVIEW state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 1. 启动录制
        self._start_recording()
        
        # 2. 执行采访动作 (Pose: mic_hold, Motion: track)
        self.execute_action("interview_hold")
        
        # 3. 初始化对话状态
        self.phase = self.PHASE_START
        self.phase_start_time = self.enter_time
        self._question_index = 0
        
        # 获取跟踪目标信息
        locked_target = self.fsm.data.perception.locked_target
        if locked_target:
            self._tracked_face_id = locked_target.face_id
            self._tracked_position = locked_target.center
            self.fsm.data.perception.target_face_id = locked_target.face_id
            self.has_target = True
        else:
            self.log("No locked target provided for interview")
            self._tracked_face_id = None
            self._tracked_position = None
            self.has_target = False
            return
            
        # 重置 PID 状态
        if hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
        
        # 初始化语音活动检测
        self._last_voice_time = self.fsm.get_current_time()

    
    def run(self) -> None:
        """执行采访逻辑"""
        perception = self.fsm.data.perception
        
        # 1. 跟随目标
        target_face = self._find_tracked_face(perception.faces)
        
        if target_face is not None:
            self._tracked_position = (target_face.center_x, target_face.center_y)
            self.fsm.data.perception.target_face_id = target_face.face_id
            
            # 使用 PID 控制跟随
            self.pid_follow_target_face(target_face, "[INTERVIEW跟随]")
            
            # 确保 motion strategy 正确
            if self.current_motion_strategy != "track":
                 self.execute_action("interview_hold")
            
            self.update_target_lost_time(has_target=True)
            self.has_target = True
        else:
            self.fsm.data.perception.target_face_id = None
            self.update_target_lost_time(has_target=False)
            self.has_target = False
        
        # 2. 执行对话流程
        self._execute_interview_flow()
        
        # 3. 检查录制状态
        if self._recorder and self._recorder.is_recording:
            if self._recorder.recording_duration >= self._recorder.max_duration:
                self.log("Recording reached max duration")
        
        self.iter_count += 1
    
    def _find_tracked_face(self, faces: list) -> Optional['FaceInfo']:
        """查找跟踪的目标脸"""
        matched = self.find_tracked_face_by_id_and_position(
            faces, self._tracked_face_id, self._tracked_position, self._match_distance_threshold
        )
        if matched is not None and (self._tracked_face_id is None or self._tracked_face_id < 0):
            self._tracked_face_id = matched.face_id
        return matched
    
    def _execute_interview_flow(self) -> None:
        """执行采访流程"""
        phase_elapsed = self.fsm.get_current_time() - self.phase_start_time
        
        if self.phase == self.PHASE_START:
            self.set_speech("interview_start")
            self._enter_phase(self.PHASE_GREETING)
            
        elif self.phase == self.PHASE_GREETING:
            if phase_elapsed >= self.greeting_duration:
                self._enter_phase(self.PHASE_QUESTION)
                self._ask_question()
                
        elif self.phase == self.PHASE_QUESTION:
            if phase_elapsed >= self.question_duration:
                self._enter_phase(self.PHASE_LISTENING)
                self.listening_start_time = self.fsm.get_current_time()
                self._last_voice_time = self.fsm.get_current_time()
                self.has_spoken = False
                
        elif self.phase == self.PHASE_LISTENING:
            silence_duration = self._get_silence_duration()
            listening_duration = self.fsm.get_current_time() - self.listening_start_time
            
            answered = self.has_spoken and (silence_duration > self.silence_threshold)
            timeout = listening_duration > self.listening_timeout
            
            if answered or timeout:
                if answered:
                    self.log(f"Answer detected (silence: {silence_duration:.2f}s)")
                else:
                    self.log(f"Listening timeout (duration: {listening_duration:.2f}s)")
                    
                if self._question_index < len(self._questions):
                    self._enter_phase(self.PHASE_QUESTION)
                    self._ask_question()
                else:
                    self._enter_phase(self.PHASE_ENDING)
                    self.set_speech("interview_thanks")
                
        elif self.phase == self.PHASE_ENDING:
            if phase_elapsed >= self.ending_duration:
                self._enter_phase(self.PHASE_DONE)
                
        elif self.phase == self.PHASE_DONE:
            pass
    
    def _enter_phase(self, new_phase: str) -> None:
        self.phase = new_phase
        self.phase_start_time = self.fsm.get_current_time()
        self.log(f"Interview phase: {new_phase}")
    
    def _ask_question(self) -> None:
        if self._question_index < len(self._questions):
            question = self._questions[self._question_index]
            speech_id = f"interview_question_{self._question_index + 1}"
            self.set_speech(speech_id)
            
            est_duration = 1.5 + len(question) * 0.25
            self.question_duration = max(2.0, est_duration)
            
            self.log(f"Asking question {self._question_index + 1}: {question}")
            self._question_index += 1
        else:
            self._enter_phase(self.PHASE_ENDING)
            self.set_speech("interview_thanks")
    
    def _get_silence_duration(self) -> float:
        """获取静音持续时间 (简单能量检测)"""
        if self._recorder:
            energy = self._recorder.get_current_audio_energy()
            current_time = self.fsm.get_current_time()
            if energy > self._voice_threshold:
                self._last_voice_time = current_time
                if not self.has_spoken and self.phase == self.PHASE_LISTENING:
                    self.has_spoken = True
                    self.log(f"Voice started: energy={energy:.4f}")
            return current_time - self._last_voice_time
        return 0.0
            
    def _start_recording(self) -> None:
        try:
            if os.path.isdir(self.recording_save_path):
                os.makedirs(self.recording_save_path, exist_ok=True)
            
            video_size = (640, 480)
            test_frame = self._get_video_frame()
            if test_frame is not None:
                video_size = (test_frame.shape[1], test_frame.shape[0])
            
            self._recorder = InterviewRecorder(
                save_path=self.recording_save_path,
                video_size=video_size,
                video_fps=30,
                camera_index=None,
                frame_callback=self._get_video_frame,
                video_quality=18,
                logger=self.logger
            )
            
            if self._recorder.start():
                self.log(f"Recording started")
            else:
                self.log("Failed to start recording")
                self._recorder = None
                
        except Exception as e:
            self.log(f"Error starting recording: {e}")
            self._recorder = None
    
    def _get_video_frame(self):
        if hasattr(self.fsm.data.perception, 'current_frame'):
            return self.fsm.data.perception.current_frame
        return None
    
    def _stop_recording(self) -> None:
        if self._recorder:
            saved_path = self._recorder.stop()
            if saved_path and os.path.exists(saved_path):
                 file_size = os.path.getsize(saved_path)
                 if file_size == 0:
                     try:
                         os.remove(saved_path)
                     except: pass
                 else:
                     self.log(f"Recording saved: {saved_path}")
            self._recorder = None
    
    def check_transition(self) -> WeddingStateName:
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
            
        if not hasattr(self, 'has_target') or not self.has_target:
            self.log("Target missing or lost, returning to IDLE")
            return WeddingStateName.IDLE
        
        if self.phase == self.PHASE_DONE:
            phase_elapsed = self.fsm.get_current_time() - self.phase_start_time
            if phase_elapsed >= self.done_duration:
                return WeddingStateName.IDLE
        
        if self.fsm.data.pending_command == WeddingEvent.CMD_STOP:
            return WeddingStateName.TRACKING
        
        return self.state_name
    
    def on_exit(self) -> None:
        self.log("Exiting INTERVIEW state")
        self._stop_recording()
        self.stop_action() # Reset to neutral/stop via ActionManager or logic
