
import threading
import time
import queue
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

# Copying the FFMPEGProcessor class for testing (simulation)
class FFMPEGProcessor:
    _instance = None
    _lock = threading.Lock()
    
    def __init__(self):
        self.job_queue = queue.Queue()
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True, name="FFMPEG_Worker")
        self.worker_thread.start()
        self.logger = logging.getLogger("FFMPEGProcessor")
        self.processed_jobs = [] # For verification
        self.lock = threading.Lock()
        
    @classmethod
    def get_instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
        return cls._instance
        
    def submit_job(self, func, *args, **kwargs):
        self.logger.info(f"Submitting job. Queue size before put: {self.job_queue.qsize()}")
        self.job_queue.put((func, args, kwargs))
        
    def _worker_loop(self):
        while self.running:
            try:
                func, args, kwargs = self.job_queue.get(timeout=0.1)
                try:
                    func(*args, **kwargs)
                except Exception as e:
                    self.logger.error(f"Job failed: {e}")
                finally:
                    self.job_queue.task_done()
            except queue.Empty:
                continue

    def get_processed_count(self):
        return len(self.processed_jobs)

ffmpeg_processor = FFMPEGProcessor.get_instance()

def mock_ffmpeg_job(job_id, duration):
    print(f"Job {job_id} started. Simulating {duration}s processing...")
    time.sleep(duration)
    print(f"Job {job_id} finished.")
    with ffmpeg_processor.lock:
        ffmpeg_processor.processed_jobs.append(job_id)

def test_queue():
    print("Submitting Job 1 (Heavy)")
    ffmpeg_processor.submit_job(mock_ffmpeg_job, 1, 2.0)
    
    print("Submitting Job 2 (Quick)")
    ffmpeg_processor.submit_job(mock_ffmpeg_job, 2, 0.5)
    
    print("Submitting Job 3 (Medium)")
    ffmpeg_processor.submit_job(mock_ffmpeg_job, 3, 1.0)
    
    print("Waiting for jobs to complete...")
    
    # Wait enough time
    time.sleep(4.0)
    
    processed = ffmpeg_processor.processed_jobs
    print(f"Processed jobs: {processed}")
    
    assert processed == [1, 2, 3], f"Jobs executed out of order or failed! {processed}"
    print("FFMPEG Queue Logic Verified!")

if __name__ == "__main__":
    test_queue()
