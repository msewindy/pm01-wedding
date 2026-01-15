#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pyaudio
import numpy as np
import threading
import time
import math
import struct
import ctypes
import sys
import contextlib
import subprocess

# ========== ALSA Error Suppression (Same as InterviewRecorder) ==========
ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextlib.contextmanager
def no_alsa_err():
    if sys.platform != 'linux':
        yield
        return
    try:
        asound = ctypes.cdll.LoadLibrary('libasound.so.2')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None)
    except:
        yield

from ..utils.audio_utils import find_audio_device, get_alsa_card_index_from_info

def init_hardware_mixer(card_id=0, logger=None):
    """
    Apply ALSA mixer settings (Capture Switch=ON, Vol=100%).
    """
    if logger is None:
        import logging
        logger = logging.getLogger("System")
        
    try:
        # 1. Open Mic Capture Switch (numid=3)
        cmd_switch = ["amixer", "-c", str(card_id), "cset", "numid=3", "on"]
        subprocess.run(cmd_switch, check=True, capture_output=True)
        
        # 2. Set Mic volume to 100%
        cmd_vol = ["amixer", "-c", str(card_id), "sset", "Mic", "100%", "unmute"]
        subprocess.run(cmd_vol, check=True, capture_output=True)
        
        logger.info(f"ALSA hardware mixer initialized for Card {card_id}")
    except Exception as e:
        logger.warning(f"Failed to set ALSA mixer for Card {card_id}: {e}")

class AmbientNoiseNode(Node):
    def __init__(self):
        super().__init__('ambient_noise_node')
        
        self.publisher_ = self.create_publisher(Float32, '/perception/ambient_noise_level', 10)
        
        # Subscribe to FSM state to release audio resource during INTERVIEW
        from std_msgs.msg import String
        self.state_sub = self.create_subscription(String, '/wedding/fsm/state', self._on_fsm_state, 10)
        self.is_paused = False
        
        # Audio params
        self.declare_parameter('audio_device_id', -1)
        self.audio_device_id = self.get_parameter('audio_device_id').value
        
        self.declare_parameter('alsa_card_id', 0)
        self.alsa_card_id = self.get_parameter('alsa_card_id').value
        
        self.rate = 44100
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.gain = 2.0  # Software gain
        
        self.audio = None
        self.stream = None
        self.is_running = False
        
        # Noise calculation
        self.ema_noise = 0.0
        self.alpha = 0.1  # Smoothing factor
        
        # Start thread
        self.start_audio()
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("AmbientNoiseNode started")

    def _on_fsm_state(self, msg):
        state_name = msg.data
        # If transitioning TO InterviewState, pause audio reading
        if "InterviewState" in state_name or "INTERVIEW" in state_name:
            if not self.is_paused:
                self.is_paused = True
                self.get_logger().info("Paused audio monitoring (Interview started)")
        else:
            if self.is_paused:
                self.is_paused = False
                self.get_logger().info("Resumed audio monitoring (Interview ended)")

    def start_audio(self):
        try:
            with no_alsa_err():
                self.audio = pyaudio.PyAudio()
            
            # Determine Input Device using shared utility
            input_device_index, device_info = find_audio_device(
                self.audio, 
                self.audio_device_id, 
                target_name="USB Audio", 
                logger=self.get_logger()
            )
            
            # Initialize Hardware Mixer
            # Smart detection: Use configured card_id, but if default (0) and device info suggests another, use that.
            final_card_id = self.alsa_card_id
            if self.alsa_card_id == 0 and device_info:
                detected_card = get_alsa_card_index_from_info(device_info)
                if detected_card != 0:
                     final_card_id = detected_card
                     self.get_logger().info(f"Auto-detected ALSA Card ID {final_card_id} from device info")
            
            init_hardware_mixer(final_card_id, self.get_logger())

            # Open stream
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                input_device_index=input_device_index,
                frames_per_buffer=self.chunk
            )
            self.is_running = True
            
            self.thread = threading.Thread(target=self.audio_loop)
            self.thread.daemon = True
            self.thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Failed to start audio: {e}")

    def audio_loop(self):
        while self.is_running:
            # If paused (e.g. during Interview), just sleep to save CPU and reduce contention
            if self.is_paused:
                time.sleep(0.1)
                continue

            try:
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                # Ensure 16-bit PCM
                if self.gain != 1.0:
                     # Apply gain using numpy for efficiency
                     audio_data = np.frombuffer(data, dtype=np.int16)
                     amplified = np.clip(audio_data * self.gain, -32768, 32767).astype(np.int16)
                     shorts = amplified.tolist() # Convert back to list for existing logic
                else:
                     shorts = struct.unpack(f"{self.chunk}h", data)
                     
                sum_squares = sum(s**2 for s in shorts)
                rms = math.sqrt(sum_squares / self.chunk)
                normalized_rms = rms / 32768.0
                
                # Update EMA
                if self.ema_noise == 0.0:
                    self.ema_noise = normalized_rms
                else:
                    self.ema_noise = self.alpha * normalized_rms + (1 - self.alpha) * self.ema_noise
                    
                # Publish high frequency or just update internal state?
                # We publish in timer callback at 1Hz to avoid flooding, or maybe 10Hz here?
                # Let's publish here every N chunks if we want faster response
                # But timer is safer for ROS interaction.
                
            except Exception:
                # self.get_logger().warn(f"Audio read error: {e}") # Avoid spamming logs in loop
                time.sleep(0.1)

    def timer_callback(self):
        msg = Float32()
        msg.data = float(self.ema_noise)
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Noise Level: {self.ema_noise:.4f}")

    def destroy_node(self):
        self.is_running = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio:
            self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AmbientNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
