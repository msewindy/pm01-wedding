import time
import logging
from typing import Optional, Dict

class MockInterviewRecorder:
    """
    Mock Interview Recorder for Simulation Testing
    
    Simulates audio energy levels based on predefined scenarios.
    """
    
    SCENARIOS = {
        # Scenario A: Standard Answer
        # Wait 1s -> Speak 5s -> Silence
        "A": [
            {"time": 0.0, "energy": 0.005}, # Silence start
            {"time": 1.0, "energy": 0.1},   # Start speaking
            {"time": 6.0, "energy": 0.005}  # Stop speaking
        ],
        # Scenario B: Hesitation
        # Silence 6s -> Speak
        "B": [
            {"time": 0.0, "energy": 0.005},
            {"time": 6.0, "energy": 0.1},   # Late start
            {"time": 9.0, "energy": 0.005}
        ],
        # Scenario C: Long Answer
        # Speak continuously
        "C": [
            {"time": 0.0, "energy": 0.1},
            {"time": 40.0, "energy": 0.1}   # Keep speaking
        ],
        # Scenario D: Intermittent
        # Speak 2s -> Silence 0.8s -> Speak 2s
        "D": [
            {"time": 0.0, "energy": 0.1},
            {"time": 2.0, "energy": 0.005}, # Short pause
            {"time": 2.8, "energy": 0.1},   # Resume
            {"time": 5.0, "energy": 0.005}
        ],
        # Scenario E: No Answer
        # Silence forever
        "E": [
            {"time": 0.0, "energy": 0.005},
            {"time": 20.0, "energy": 0.005}
        ]
    }

    def __init__(self, save_path: str, scenario: str = "A", logger: Optional[logging.Logger] = None, **kwargs):
        self.logger = logger or logging.getLogger("MockRecorder")
        self.scenario_name = scenario
        self.timeline = self.SCENARIOS.get(scenario, self.SCENARIOS["A"])
        
        self.start_time = 0.0
        self._is_recording = False
        self.max_duration = 300.0
        
        self.logger.warning(f"INITIALIZED MOCK RECORDER with Scenario: {self.scenario_name}")
        
    def start(self) -> bool:
        self.start_time = time.time()
        self._is_recording = True
        self.logger.info("Mock recording started")
        return True
        
    def stop(self) -> Optional[str]:
        self._is_recording = False
        self.logger.info("Mock recording stopped")
        return "/tmp/mock_recording.mp4"
        
    def get_current_audio_energy(self) -> float:
        if not self._is_recording:
            return 0.0
            
        elapsed = time.time() - self.start_time
        
        # Find current energy based on timeline
        current_energy = 0.005 # Default silence
        
        # Iterate to find the interval we are in
        # Timeline is a list of change points: [{"time": t, "energy": e}, ...]
        # We assume the energy persists until the next change point
        
        for point in self.timeline:
            if elapsed >= point["time"]:
                current_energy = point["energy"]
            else:
                break
                
        return current_energy

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    @property
    def recording_duration(self) -> float:
        if not self._is_recording:
            return 0.0
        return time.time() - self.start_time
