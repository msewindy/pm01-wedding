
import pyaudio
import wave
import struct
import math
import os
import numpy as np

def test_multichannel_recording():
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 8  # XF_AIUI 应该是 8 通道
    RATE = 16000
    RECORD_SECONDS = 5
    OUTPUT_DIR = "debug_channels"
    
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    p = pyaudio.PyAudio()

    print(f"\n=== Testing Multichannel Recording (5s) @ 16000Hz, 8 Channels ===")
    
    try:
        # Find default device index if possible, or let PyAudio choose default
        # Assuming the system default is set to the USB mic as per previous debug
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

        print("* Recording started... Please speak into the microphone.")
        
        frames = []
        
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            
            # Simple visualization of Ch 0 energy
            # Data is interleaved: [sample0_ch0, sample0_ch1, ..., sample0_ch7, sample1_ch0...]
            # We just peek at the raw bytes for a rough energy check
            pass

        print("* Recording done.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Convert to numpy array for easy de-interleaving
        # 16-bit PCM = 2 bytes. 
        buffer = b''.join(frames)
        audio_data = np.frombuffer(buffer, dtype=np.int16)
        
        # Reshape: [Total Samples, Channels]
        # Total valid frames = len(audio_data) / CHANNELS
        num_frames = len(audio_data) // CHANNELS
        audio_data = audio_data[:num_frames * CHANNELS]
        audio_data = audio_data.reshape(-1, CHANNELS)
        
        print(f"\nAnalyzing {CHANNELS} channels:")
        
        for ch in range(CHANNELS):
            ch_data = audio_data[:, ch]
            
            # Calculate RMS
            rms = np.sqrt(np.mean(ch_data.astype(float)**2)) / 32768.0
            db = 20 * math.log10(rms) if rms > 0 else -99
            
            bars = "#" * int(rms * 200)
            print(f"Channel {ch}: Energy={rms:.4f} ({db:.1f} dB) [{bars}]")
            
            # Save individual channel
            filename = os.path.join(OUTPUT_DIR, f"channel_{ch}.wav")
            wf = wave.open(filename, 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(ch_data.tobytes())
            wf.close()
        
        print(f"\n✅ Separate channel files saved to: {os.path.abspath(OUTPUT_DIR)}")
        print("Please listen to the files to find which one has clear audio.")

    except Exception as e:
        print(f"❌ Error during recording: {e}")
        p.terminate()

if __name__ == "__main__":
    try:
        import numpy
        test_multichannel_recording()
    except ImportError:
        print("Error: numpy is required. Please install it: pip3 install numpy")
