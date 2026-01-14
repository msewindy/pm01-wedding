#!/usr/bin/env python3
"""
Recording Cleanup Script

Monitors the recording directory and keeps only the N most recent files.
Purpose: Prevent disk fill-up during long-duration testing.

Usage:
    python3 cleanup_records.py [--path /path/to/dir] [--keep 10] [--interval 60]
"""

import os
import time
import argparse
import glob
from datetime import datetime

def cleanup_files(directory, keep_count):
    """
    Check directory and remove oldest files if count > keep_count
    """
    if not os.path.exists(directory):
        print(f"[{datetime.now()}] Directory not found: {directory}")
        return

    # Find all files (assuming mp4/mkv/avi or just all files?)
    # User said "video files", but safer to clean all files in that specific folder 
    # if it's a dedicated record folder. Let's filter for common video extensions just in case.
    extensions = ['*.mp4', '*.avi', '*.mkv', '*.mov', '*.wav']
    files = []
    for ext in extensions:
        files.extend(glob.glob(os.path.join(directory, ext)))
    
    # Remove duplicates if any matching multiple patterns (unlikely with extensions)
    files = list(set(files))
    
    if len(files) <= keep_count:
        return

    # Sort by modification time (newest first)
    files.sort(key=os.path.getmtime, reverse=True)
    
    # Keep the first N
    files_to_keep = files[:keep_count]
    files_to_delete = files[keep_count:]
    
    print(f"[{datetime.now()}] Found {len(files)} files. Keeping {keep_count}. Deleting {len(files_to_delete)}.")
    
    for f in files_to_delete:
        try:
            os.remove(f)
            print(f"  Deleted: {os.path.basename(f)}")
        except Exception as e:
            print(f"  Error deleting {f}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Periodically cleanup old recordings.")
    parser.add_argument("--path", default="/home/lfw/interview_records", help="Directory to monitor")
    parser.add_argument("--keep", type=int, default=10, help="Number of recent files to keep")
    parser.add_argument("--interval", type=int, default=60, help="Check interval in seconds")
    
    args = parser.parse_args()
    
    print(f"Starting cleanup monitor...")
    print(f"  Target: {args.path}")
    print(f"  Keep: {args.keep} newest files")
    print(f"  Check Interval: {args.interval}s")
    
    try:
        while True:
            cleanup_files(args.path, args.keep)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping cleanup monitor.")

if __name__ == "__main__":
    main()
