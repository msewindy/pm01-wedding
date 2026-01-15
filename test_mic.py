
import pyaudio
import numpy as np
import time
import math

def list_devices():
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    
    print("Available Audio Devices:")
    for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            name = p.get_device_info_by_host_api_device_index(0, i).get('name')
            print(f"Index {i}: {name}")
    
    p.terminate()

def test_recording(device_index=None):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    
    p = pyaudio.PyAudio()
    
    try:
        if device_index is None:
             # Try to find default
             device_index = p.get_default_input_device_info()['index']
        
        print(f"\nOpening stream on Device Index: {device_index}")
        
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        input_device_index=device_index,
                        frames_per_buffer=CHUNK)

        print("Recording... (Press Ctrl+C to stop)")
        
        count = 0
        while count < 100:
            count += 1
            data = stream.read(CHUNK, exception_on_overflow=False)
            shorts = np.frombuffer(data, dtype=np.int16)
            # Cast to int64/float to avoid overflow during square
            rms = np.sqrt(np.mean(shorts.astype(np.float32)**2))
            
            # Simple visualization
            bars = int(rms / 500)
            print(f"RMS: {rms:.2f} | {'#' * bars}")
            
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'stream' in locals():
            stream.stop_stream()
            stream.close()
        p.terminate()

if __name__ == "__main__":
    list_devices()
    
    print("\nPlease enter the Device Index to test (or press Enter for default):")
    try:
        val = input().strip()
        idx = int(val) if val else None
        test_recording(idx)
    except Exception as e:
        print(f"Invalid input: {e}") 
