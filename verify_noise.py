#!/usr/bin/env python3
import pyaudio
import numpy as np
import time
import subprocess
import math
import struct
import sys
import argparse

def init_hardware_mixer():
    """
    Apply ALSA hardware mixer settings.
    """
    print("Initializing hardware mixer...")
    try:
        # 1. Open Mic Capture Switch (numid=3)
        # Note: -c 0 assumes USB mic is card 0.
        cmd_switch = ["amixer", "-c", "0", "cset", "numid=3", "on"]
        subprocess.run(cmd_switch, check=True, capture_output=True)
        print("  - Mic Capture Switch enabled.")
        
        # 2. Set Mic volume to 100%
        cmd_vol = ["amixer", "-c", "0", "sset", "Mic", "100%", "unmute"]
        subprocess.run(cmd_vol, check=True, capture_output=True)
        print("  - Mic volume set to 100%.")
        
    except subprocess.CalledProcessError as e:
        print(f"  [WARNING] Failed to set ALSA mixer: {e}")
    except Exception as e:
        print(f"  [WARNING] Failed to set ALSA mixer: {e}")

def verify_noise(gain=2.0, duration=10.0):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    
    p = pyaudio.PyAudio()
    
    print(f"\nStarting audio stream monitoring for {duration} seconds...")
    print(f"Settings: Rate={RATE}, Chunk={CHUNK}, Gain={gain}")
    
    try:
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
        
        print("\nMonitoring... (Ctrl+C to stop early)")
        print(f"{'RMS (Raw)':<15} | {'RMS (Gain)':<15} | {'dB (approx)':<10} | {'Status'}")
        print("-" * 60)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                data = stream.read(CHUNK, exception_on_overflow=False)
                
                # Convert to numpy for calculation
                audio_data = np.frombuffer(data, dtype=np.int16)
                
                # Calculate Raw RMS
                params_raw = audio_data.astype(np.float32)
                rms_raw = np.sqrt(np.mean(params_raw**2))
                norm_rms_raw = rms_raw / 32768.0
                
                # Apply Gain
                amplified_data = np.clip(audio_data * gain, -32768, 32767)
                
                # Calculate Amplified RMS
                params_amp = amplified_data.astype(np.float32)
                rms_amp = np.sqrt(np.mean(params_amp**2))
                norm_rms_amp = rms_amp / 32768.0
                
                # Calculate dB (relative to full scale)
                db = 20 * math.log10(norm_rms_amp + 1e-9) 
                
                # Visual bar
                bar_len = int(norm_rms_amp * 50)
                bar = "#" * bar_len
                
                print(f"{norm_rms_raw:.4f}          | {norm_rms_amp:.4f}          | {db:.1f} dB     | {bar}")
                
            except IOError as e:
                print(f"[Error] Stream read error: {e}")
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'stream' in locals() and stream.is_active():
            stream.stop_stream()
            stream.close()
        p.terminate()
        print("Audio resources released.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Verify ambient noise levels with gain.")
    parser.add_argument("--gain", type=float, default=2.0, help="Software gain to apply (default: 2.0)")
    parser.add_argument("--duration", type=float, default=20.0, help="Duration to run the test (seconds)")
    parser.add_argument("--skip-mixer", action="store_true", help="Skip mixer initialization")
    
    args = parser.parse_args()
    
    if not args.skip_mixer:
        init_hardware_mixer()
    
    verify_noise(gain=args.gain, duration=args.duration)
