
import sys
import os

# Mock classes to test VAD logic in isolation
class MockFSM:
    def get_current_time(self):
        return 0.0

class MockState:
    def __init__(self):
        self._voice_threshold = 0.02
        self.current_ambient_noise = 0.0
        self.iter_count = 0
        
    def log(self, msg):
        print(f"[LOG] {msg}")
        
    def _noise_callback(self, msg_data):
        self.current_ambient_noise = msg_data
        # Logic to test
        new_thresh = max(0.05, min(0.15, self.current_ambient_noise * 1.5 + 0.01))
        
        if abs(new_thresh - self._voice_threshold) > 0.005:
            self._voice_threshold = new_thresh
            print(f"Adaptive VAD: Noise={self.current_ambient_noise:.4f}, Threshold={self._voice_threshold:.4f}")
        else:
            self._voice_threshold = new_thresh

def test_vad():
    s = MockState()
    
    print("Test 1: Low Noise (0.01) -> Should clamp to 0.05")
    s._noise_callback(0.01)
    assert abs(s._voice_threshold - 0.05) < 0.001, f"Expected 0.05, got {s._voice_threshold}"
    
    print("Test 2: Medium Noise (0.04) -> 0.04 * 1.5 + 0.01 = 0.07")
    s._noise_callback(0.04)
    assert abs(s._voice_threshold - 0.07) < 0.001, f"Expected 0.07, got {s._voice_threshold}"
    
    print("Test 3: High Noise (0.2) -> Should clamp to 0.15")
    s._noise_callback(0.2)
    assert abs(s._voice_threshold - 0.15) < 0.001, f"Expected 0.15, got {s._voice_threshold}"
    
    print("VAD Logic Verified!")

if __name__ == "__main__":
    test_vad()
