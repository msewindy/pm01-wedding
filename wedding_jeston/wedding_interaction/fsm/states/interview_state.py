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
from .interview_recorder import InterviewRecorder, init_hardware_mixer

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
        self._questions = []
        
        # Load questions dynamically from SPEECH_LIBRARY
        from ...audio.speech_resources import SPEECH_LIBRARY
        
        # Find all keys starting with 'interview_question_'
        q_keys = [k for k in SPEECH_LIBRARY.keys() if k.startswith('interview_question_')]
        # Sort by index suffix (e.g. '1', '2')
        try:
            q_keys.sort(key=lambda x: int(x.split('_')[-1]))
        except ValueError:
            # Fallback sort if naming convention fails
            q_keys.sort()
            
        for k in q_keys:
            self._questions.append(SPEECH_LIBRARY[k])
            
        if not self._questions:
            self.log("Warning: No interview questions found in SPEECH_LIBRARY!")
            # Fallback
            self._questions = ["(无问题配置)"]
        
        # 时间参数（可从配置读取）
        config = self.fsm.data.config or {}
        self.greeting_duration = config.get('interview_greeting_duration', 3.0)
        self.question_duration = config.get('interview_question_duration', 2.0)
        
        # New split timeouts
        self.no_speech_timeout = config.get('interview_no_speech_timeout', 8.0)
        self.speech_max_duration = config.get('interview_speech_max_duration', 30.0)
        
        # Backward compatibility for old config
        if 'interview_listening_timeout' in config:
             # If old param exists but new ones don't, use old for no_speech
             if 'interview_no_speech_timeout' not in config:
                 self.no_speech_timeout = config['interview_listening_timeout']

        self.silence_threshold = config.get('interview_silence_threshold', 1.5)
        self.has_spoken = False
        self.ending_duration = config.get('interview_ending_duration', 2.0)
        self.done_duration = config.get('interview_done_duration', 1.0)
        self._voice_threshold = config.get('interview_voice_threshold', 0.02)
        
        # 麦克风增益配置
        self.mic_gain = config.get('interview_mic_gain', 2.0)
        self.auto_init_alsa = config.get('interview_auto_init_alsa', True)
        
        # 丢失超时 (Interview 需要较强的粘性)
        self.lost_timeout = config.get('interview_lost_timeout', 3.0)
        
        # 音频采样率
        self.audio_sample_rate = config.get('interview_audio_sample_rate', 44100)
        
        # 录制保存路径
        default_save_path = os.path.join(os.path.expanduser("~"), "interview_records")
        config_save_path = config.get('interview_recording_save_path', "")
        if config_save_path and config_save_path.strip():
            self.recording_save_path = config_save_path
        else:
            self.recording_save_path = default_save_path
    
    def on_enter(self) -> None:
        """进入采访状态"""
        self.log("Entering INTERVIEW state")
        self.iter_count = 0
        self.enter_time = self.fsm.get_current_time()
        
        # 0. 初始化硬件麦克风 (如果配置启用)
        if self.auto_init_alsa:
            init_hardware_mixer(self.logger)
        
        # 1. 启动录制
        self._start_recording()
        
        # 2. 执行采访动作 (Pose: mic_hold, Motion: track)
        self.execute_action("interview_hold")
        
        # 3. 初始化对话状态
        self.phase = self.PHASE_START
        self.phase_start_time = self.enter_time
        self._question_index = 0
        
        # 获取跟踪目标信息
        
        # 配置全局跟踪器
        # Interview 状态：强容忍 (3.0s)
        self.fsm.face_tracker.configure(lost_timeout=self.lost_timeout)
        
        locked_target = self.fsm.data.perception.locked_target
        if locked_target:
             self.fsm.face_tracker.set_target(locked_target)
             self.fsm.data.perception.target_face_id = locked_target.face_id
             # state used by check_transition implicitly
        else:
            # 如果没有转交的目标 (e.g. 强制进入)，尝试保持现有目标?
            # 这里的逻辑是 Interview 必须有个明确目标。
            # 假设 FaceTracker 已经有目标了? 我们 set_target 会覆盖。
            # 如果 locked_target 是 None (e.g. 语音命令)，我们希望 FaceTracker 继续跟踪当前目标?
            # "fsm.data.perception.locked_target" 只有 SEARCH -> TRACKING 会设置。
            # TRACKING -> INTERVIEW 没有设置 locked_target 吗?
            # 检查 SearchState: fsm.data.perception.locked_target = locked.
            # TrackingState: 读取它, 但没有更新它。
            # 所以 locked_target 是最初的那个。
            # 但是 Tracking 过程中 ID 可能变了 (Reselect/Recovery)。
            # 所以我们应该用 FaceTracker 当前的目标? 
            # FaceTracker 是全局的，如果从 TRACKING 过来，target 就在 Tracker 里。
            # 我们不需要 set_target! !
            # UNLESS we come from IDLE directly with a locked target?
            # 我们的 FSM 设计：TRACKING -> INTERVIEW. Tracker 状态应该保持。
            # Search -> (locked) -> Tracking -> (timeout) -> Interview.
            # 如果我们不调用 set_target, Tracker 保持原样。
            # 但是 TrackingState on_enter 加载了 locked_target.
            # 如果 Tracking 运行了很久，Tracker 里的目标是最新的。
            # 如果我们这里重新 set_target(locked_target)，可能会回滚到旧 ID?
            # NO. locked_target is persistent in data context.
            # 实际上 TrackingState 没有更新 locked_target.
            # 这是一个潜在 BUG。如果我们不更新 locked_target，Interview 用的是旧的。
            # 解决方案: 直接使用 FaceTracker 的当前目标，不需要 set_target。
            # 只要 FaceTracker 不被 Reset。
            # SearchState calls reset(). TrackingState doesn't. InterviewState shouldn't.
            # 但是如果 Interview 直接从 Command 触发 (IDLE -> INTERVIEW)?
            # 这种情况下 FaceTracker 可能是空的。
            # 我们应该检查 Tracker 是否有目标。
            pass
            
        if not self.fsm.face_tracker.has_target:
             self.log("Warning: No target for interview!")
             # check_transition will exit.
            
        # 重置 PID 状态
        if hasattr(self, '_pid_integral_x'):
            self._pid_integral_x = 0.0
            self._pid_integral_y = 0.0
            self._pid_last_error_x = 0.0
            self._pid_last_error_y = 0.0
        
        # 重置平滑状态
        if hasattr(self, '_smoothed_look_at_x'):
            delattr(self, '_smoothed_look_at_x')
            delattr(self, '_smoothed_look_at_y')
        
        # 初始化语音活动检测
        self._last_voice_time = self.fsm.get_current_time()

    
    def run(self) -> None:
        """执行采访逻辑"""
        perception = self.fsm.data.perception
        
        # 1. 更新跟踪器
        state = self.fsm.face_tracker.update(perception.faces)
        
        if state.has_target:
            target_face = state.target_face
            self.fsm.data.perception.target_face_id = target_face.face_id
            
            # 使用 PID 控制跟随
            # 注: 即使在 grace period 也会跟随最后已知位置
            self.pid_follow_target_face(target_face, "[INTERVIEW跟随]")
            
            # 确保 motion strategy 正确
            if self.current_motion_strategy != "track":
                 self.execute_action("interview_hold")
        else:
            self.fsm.data.perception.target_face_id = None
        
        # 2. 执行对话流程
        self._execute_interview_flow()
        
        # 3. 检查录制状态
        if self._recorder and self._recorder.is_recording:
            if self._recorder.recording_duration >= self._recorder.max_duration:
                self.log("Recording reached max duration")
        
        self.iter_count += 1
        
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
            
            answered = False
            timeout = False
            
            if not self.has_spoken:
                # 情况1：用户还没开始说话
                # 检查是否等待超时
                if listening_duration > self.no_speech_timeout:
                    timeout = True
                    self.log(f"No speech detected timeout ({listening_duration:.1f}s)")
            else:
                # 情况2：用户已经开始说话
                # 检查是否回答完成 (VAD silence)
                if silence_duration > self.silence_threshold:
                    answered = True
                    self.log(f"Answer finished (silence: {silence_duration:.2f}s)")
                
                # 检查是否说话时间太长 (Max Speech Duration)
                # 注意：listening_duration 包含了开始前的等待时间，
                # 但只要用户开口了，我们允许的总时长由 speech_max_duration 控制
                # (或者更精确地： max_duration 从 has_spoken=True 开始算? 
                #  简单起见，从 phase start 算比较安全，避免一直不说话还一直占着)
                # 更合理的逻辑：
                #   Start -> First Voice: limit by no_speech_timeout
                #   First Voice -> End: limit by speech_max_duration (counting from First Voice)
                
                # 这里简化实现：如果说话了，总时长限制放宽到 speech_max_duration
                # 因为 speech_max_duration (30s) 通常远大于 no_speech_timeout (8s)
                if listening_duration > self.speech_max_duration:
                    timeout = True # 强行打断
                    self.log(f"Speech max duration exceeded ({listening_duration:.1f}s)")
            
            if answered or timeout:  
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
                audio_sample_rate=self.audio_sample_rate,
                camera_index=None,
                frame_callback=self._get_video_frame,
                video_quality=18,
                audio_gain=self.mic_gain,
                enable_beauty=self.fsm.data.config.get('interview_enable_beauty', True),
                logger=self.logger
            )
            
            if self._recorder.start():
                self.log(f"Recording started. Saving to: {self.recording_save_path}")
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
                     abs_path = os.path.abspath(saved_path)
                     self.log("="*50)
                     self.log(f" [RESULT] Interview Video Saved: {abs_path}")
                     self.log("="*50)
            self._recorder = None
    
    def check_transition(self) -> WeddingStateName:
        base_next = super().check_transition()
        if base_next != self.state_name:
            return base_next
            
        # 更新并获取状态
        state = self.fsm.face_tracker.update(self.fsm.data.perception.faces)
            
        if not state.has_target:
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
