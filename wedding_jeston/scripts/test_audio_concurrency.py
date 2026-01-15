import pyaudio
import time
import multiprocessing
import os

def record_process(name, duration):
    try:
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=44100,
                        input=True,
                        frames_per_buffer=1024)
        print(f"[{name}] Stream opened successfully. Recording for {duration}s...")
        
        start = time.time()
        chunks = 0
        while time.time() - start < duration:
            data = stream.read(1024, exception_on_overflow=False)
            chunks += 1
            time.sleep(0.01)
            
        print(f"[{name}] Finished. Captured {chunks} chunks.")
        stream.stop_stream()
        stream.close()
        p.terminate()
    except Exception as e:
        print(f"[{name}] Failed: {e}")

if __name__ == "__main__":
    print("Starting Process A (Long running)...")
    p1 = multiprocessing.Process(target=record_process, args=("ProcA", 5))
    p1.start()
    
    time.sleep(1.0)
    
    print("Starting Process B (Short running)...")
    p2 = multiprocessing.Process(target=record_process, args=("ProcB", 2))
    p2.start()
    
    p1.join()
    p2.join()
    print("Test complete.")
