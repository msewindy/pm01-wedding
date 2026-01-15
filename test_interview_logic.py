
import sys
import os
import time
import logging
from unittest.mock import MagicMock, patch

# Add project root to path to allow imports
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(current_dir, 'wedding_jeston')
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Mock ROS2 dependencies BEFORE importing modules that use them
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger("InterviewTest")

# Mock the entire wedding_interaction package structure if needed, 
# but we will try to rely on real files where possible.
# However, to isolate logic, we might need to mock base classes if they depend heavily on ROS.

# Let's import the specific class. 
# We need to handle relative imports by running this as a script with compilation, 
# or by mocking the imported modules.
# Since relative imports are tricky in standalone scripts, we will mock the dependencies 
# that are imported via relative imports in interview_state.py,
# OR we can try to setup the sys.path correctly.

# Strategy: Mock the dependencies that InterviewState imports.
# It imports: 
# from ..enums import WeddingStateName, WeddingEvent
# from ..wedding_state import WeddingState
# from .interview_recorder import InterviewRecorder, init_hardware_mixer
# from .mock_recorder import MockInterviewRecorder

# We will create a dummy environment.

class MockWeddingFSM:
    def __init__(self):
        self.data = MagicMock()
        self.data.config = {
            'interview_greeting_duration': 1.0, # Shorten for test
            'interview_question_duration': 1.0,
            'interview_no_speech_timeout': 5.0,
            'interview_speech_max_duration': 10.0,
            'interview_silence_threshold': 1.5,
            'interview_voice_threshold': 0.02,
            'interview_mic_gain': 1.0,
            'interview_auto_init_alsa': False,
            'interview_lost_timeout': 999.0, # Disable lost timeout
            'use_mock_audio': True,
            'mock_scenario': 'MANUAL', # key for our manual control
            'enable_speech': True
        }
        self.data.perception = MagicMock()
        self.data.perception.faces = []
        self.data.pending_command = None
        self.data.safety_triggered = False
        self.node = MagicMock()
        self.face_tracker = MagicMock()
        
        # Configure Face Mock
        class MockFace:
            def __init__(self):
                self.center_x = 0.5
                self.center_y = 0.5
                self.timestamp = 0.0
                self.is_frontal = True
                self.area = 0.1
                self.face_id = 1
                
            @property
            def center(self):
                return (self.center_x, self.center_y)
                
            def is_in_region(self, *args):
                return True
        
        self.mock_face = MockFace()
        
        # Configure Tracker Result
        class MockTrackResult:
            def __init__(self, target):
                self.target = target
                self.target_face = target
                self.has_target = True
            def get_target_face(self):
                return self.target
                
        self.mock_track_res = MockTrackResult(self.mock_face)
        self.face_tracker.update.return_value = self.mock_track_res
        
        self.action_manager = MagicMock()
        
        # Camera Mock
        self.data.camera = MagicMock()
        self.data.camera.is_valid.return_value = True
        self.data.camera.width = 640
        self.data.camera.height = 480
        self.data.camera.fx = 600.0
        self.data.camera.fy = 600.0
        self.data.motion.look_at_target = [0.5, 0.5]
        
        self.current_time = 0.0
    
    def get_current_time(self):
        return self.current_time

# Mock SPEECH_LIBRARY to avoid importing the huge resource file
sys.modules['wedding_interaction.audio.speech_resources'] = MagicMock()
from wedding_interaction.audio import speech_resources
speech_resources.SPEECH_LIBRARY = {
    'interview_start': 'Hello',
    'interview_question_1': 'Q1',
    'interview_question_2': 'Q2',
    'interview_question_3': 'Q3',
    'interview_thanks': 'Thanks'
}

# Now we can attempt to import InterviewState. 
# But we need to patch the parent references using sys.modules hacking or 
# simply copy the class logic? No, we want to test the REAL code.
# The relative imports in `interview_state.py` will fail if we just `input` it.
# We will import it as `wedding_interaction.fsm.states.interview_state`

try:
    from wedding_interaction.fsm.states.interview_state import InterviewState
    from wedding_interaction.fsm.enums import WeddingStateName
except ImportError as e:
    logger.error(f"Import failed: {e}")
    logger.info("Setting up minimal mocks for imports...")
    # Setup mocks for what we can't import
    # This part is fragile. Better to rely on PYTHONPATH setup above.
    raise

class TestHarness:
    def __init__(self):
        self.fsm = MockWeddingFSM()
        self.state = InterviewState(self.fsm)
        self.state.logger = logger # Override logger
        self.state._use_ros2_logger = False
        
        # Override the recorder with our manual control mock
        self.manual_energy = 0.0
        self.state._recorder = MagicMock()
        self.state._recorder.get_current_audio_energy = lambda: self.manual_energy
        self.state._recorder.is_recording = True
        self.state._recorder.recording_duration = 0.0
        self.state._start_recording = MagicMock() # Disable real recording start
        
    def set_energy(self, energy):
        self.manual_energy = energy
        
    def step(self, dt=0.1):
        self.fsm.current_time += dt
        self.fsm.mock_face.timestamp = self.fsm.current_time # Keep data fresh
        self.state.run()
        # Also simulate recorder time progression if needed (mock doesn't use it for energy, but logic might)
        
    def run_tests(self):
        print("========== TEST 1: Normal Flow (Speak immediately) ==========")
        self.fsm.current_time = 0.0
        self.state.on_enter()
        self.state._recorder = MagicMock() # Re-mock because on_enter might have overwritten it
        self.state._recorder.get_current_audio_energy = lambda: self.manual_energy
        self.state._recorder.is_recording = True
        self.state._recorder.max_duration = 300.0
        self.state._recorder.recording_duration = 0.0
        
        # 1. Start Phase
        print(f"[0.0s] Phase: {self.state.phase}")
        self.step(0.1) 
        # Should be Greeting
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase} (Expected: greeting)")
        
        # Run until Question
        while self.state.phase == 'greeting':
            self.step(0.1)
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase} (Expected: question)")
        
        # Run until Listening
        while self.state.phase == 'question':
            self.step(0.1)
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase} (Expected: listening)")
        
        # Simulate User Speaking
        print("--> User starts speaking (Energy 0.1)")
        self.set_energy(0.1)
        for _ in range(20): # Speak for 2s
            self.step(0.1)
            
        print("--> User stops speaking (Energy 0.0)")
        self.set_energy(0.0)
        
        # Should wait for silence threshold (1.5s)
        wait_start = self.fsm.current_time
        while self.state.phase == 'listening':
            self.step(0.1)
            if self.fsm.current_time - wait_start > 3.0:
                print("!! Timeout waiting for silence detection !!")
                break
                
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase} (Expected: question/ending)")
        print("Test 1 Result: SUCCESS" if self.state.phase != 'listening' else "FAILURE")
        print("\n")

        print("========== TEST 2: No Speech Timeout ==========")
        self.fsm.current_time = 0.0
        self.state.on_enter()
        self.state._recorder = MagicMock()
        self.state._recorder.get_current_audio_energy = lambda: self.manual_energy
        self.state._recorder.is_recording = True
        self.state._recorder.recording_duration = 0.0
        self.state._recorder.max_duration = 300.0
        
        # Fast forward to listening
        self.state._enter_phase(self.state.PHASE_LISTENING)
        self.state.listening_start_time = self.fsm.current_time
        self.state._last_voice_time = self.fsm.current_time
        self.state.has_spoken = False
        
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase}")
        self.set_energy(0.0) # Silence
        
        # Run for 6s (Timeout is 5s)
        for _ in range(60):
            self.step(0.1)
            if self.state.phase != 'listening':
                break
                
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase} (Expected: question/ending)")
        print("Test 2 Result: SUCCESS" if self.state.phase != 'listening' else "FAILURE")
        print("\n")
        
        print("========== TEST 3: Long Speech Interruption ==========")
        self.fsm.current_time = 0.0
        self.state.on_enter()
        self.state._recorder = MagicMock()
        self.state._recorder.get_current_audio_energy = lambda: self.manual_energy
        self.state._recorder.is_recording = True
        self.state._recorder.recording_duration = 0.0
        self.state._recorder.max_duration = 300.0
        
        # Fast forward to listening
        self.state._enter_phase(self.state.PHASE_LISTENING)
        self.state.listening_start_time = self.fsm.current_time
        self.state._last_voice_time = self.fsm.current_time
        self.state.has_spoken = False
        
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase}")
        print("--> User speaks continuously...")
        self.set_energy(0.1)
        
        # Run for 12s (Max duration is 10s)
        start_listen = self.fsm.current_time
        for _ in range(120):
            self.step(0.1)
            if self.state.phase != 'listening':
                print(f"--> Interrupted at {self.fsm.current_time - start_listen:.1f}s")
                break
        
        print(f"[{self.fsm.current_time:.1f}s] Phase: {self.state.phase} (Expected: question/ending)")
        print("Test 3 Result: SUCCESS" if self.state.phase != 'listening' else "FAILURE")


if __name__ == '__main__':
    test = TestHarness()
    test.run_tests()
