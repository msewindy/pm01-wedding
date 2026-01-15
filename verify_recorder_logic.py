import sys
import unittest
from unittest.mock import MagicMock, patch
import logging

# Append path to finding the module
sys.path.append("/home/lfw/Desktop/ZQ_develop/pm01-wedding")

# Mock dependencies that might not exist or we don't want to init
sys.modules['pyaudio'] = MagicMock()
sys.modules['cv2'] = MagicMock()
sys.modules['ctypes'] = MagicMock()

# Now import the class
from wedding_jeston.wedding_interaction.fsm.states.interview_recorder import InterviewRecorder

class TestInterviewRecorder(unittest.TestCase):
    def setUp(self):
        logging.basicConfig(level=logging.INFO)

    @patch('subprocess.run')
    def test_detect_encoder_desktop_nvenc(self, mock_run):
        # Simulate NVENC available
        mock_result = MagicMock()
        mock_result.stdout = " ... h264_nvenc ..."
        mock_run.return_value = mock_result
        
        recorder = InterviewRecorder(save_path="/tmp")
        config = recorder._encoder_config
        print(f"Desktop (NVENC) Detected: {config['name']}")
        self.assertEqual(config['name'], "h264_nvenc")
        self.assertIn("-preset", config['flags'])

    @patch('subprocess.run')
    def test_detect_encoder_jetson_fallback(self, mock_run):
        # Simulate Jetson (no NVENC, only NVMPI/V4L2 which we ignore now)
        mock_result = MagicMock()
        mock_result.stdout = " ... h264_nvmpi ... h264_v4l2m2m ..."
        mock_run.return_value = mock_result
        
        recorder = InterviewRecorder(save_path="/tmp")
        config = recorder._encoder_config
        print(f"Jetson (Fallback) Detected: {config['name']}")
        self.assertEqual(config['name'], "libx264-ultrafast")
        self.assertIn("-preset", config['flags'])
        self.assertIn("ultrafast", config['flags'])
        self.assertIn("-crf", config['flags'])
        
        # Check explicit flags
        idx = config['flags'].index("-crf")
        self.assertEqual(config['flags'][idx+1], "28")
        
        idx2 = config['flags'].index("-threads")
        self.assertEqual(config['flags'][idx2+1], "4")

if __name__ == '__main__':
    unittest.main()
