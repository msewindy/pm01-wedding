
import pyaudio
import wave
import time
import struct
import math
import os

def list_devices(p):
    print("\n=== Audio Devices ===")
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    
    try:
        default_input = p.get_default_input_device_info()
        print(f"Default Input Device: ID {default_input['index']} - {default_input['name']}")
    except:
        print("No Default Input Device found")
    
    input_devices = []
    for i in range(0, numdevices):
        device = p.get_device_info_by_index(i)
        if device.get('maxInputChannels') > 0:
            print(f"ID {i}: {device.get('name')} (Channels: {device.get('maxInputChannels')}, Rate: {device.get('defaultSampleRate')})")
            input_devices.append(i)
    return input_devices

def test_recording(p, device_index=None):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    # IMPORTANT: Changing rate to 16000Hz to match the hardware
    RATE = 16000 
    RECORD_SECONDS = 5
    WAVE_OUTPUT_FILENAME = "debug_audio_16k.wav"

    print(f"\n=== Testing Recording (5 seconds) @ 16000Hz ===")
    if device_index is not None:
        device_name = p.get_device_info_by_index(device_index)['name']
        print(f"Device: {device_name} (ID: {device_index})")
    else:
        print("Device: Default")
    
    try:
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        input_device_index=device_index,
                        frames_per_buffer=CHUNK)

        print("* Recording started... Please speak into the microphone.")
        
        frames = []
        max_energy = 0.0
        
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            
            # Calculate energy
            shorts = struct.unpack(f"{len(data)//2}h", data)
            sum_squares = sum(s**2 for s in shorts)
            rms = math.sqrt(sum_squares / len(shorts)) / 32768.0
            if rms > max_energy:
                max_energy = rms
                
            # Simple visualization
            bars = "#" * int(rms * 100) # Increased scale for visibility
            print(f"\rEnergy: {rms:.4f} [{bars:<20}]", end="")

        print("\n* Recording done.")

        stream.stop_stream()
        stream.close()

        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        print(f"Saved to: {os.path.abspath(WAVE_OUTPUT_FILENAME)}")
        print(f"Max Energy Detected: {max_energy:.4f}")
        
        if max_energy < 0.005:
            print("❌ WARNING: Max energy is still very low.")
        else:
            print("✅ Energy detected! 16000Hz seems to work.")

    except Exception as e:
        print(f"❌ Error during recording: {e}")

if __name__ == "__main__":
    p = pyaudio.PyAudio()
    try:
        input_ids = list_devices(p)
        
        # Test Default Device with 16k
        test_recording(p, None)
            
    finally:
        p.terminate()
