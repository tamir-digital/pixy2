#!/usr/bin/env python3

import sys
import time
import logging
from ctypes import *
import pixy
from pixy import *
import os
from datetime import datetime
import signal
import atexit

# Configure logging
logging.basicConfig(level=logging.DEBUG,
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Frame dimensions
FRAME_WIDTH = 316
FRAME_HEIGHT = 208

class TimingTest:
    def __init__(self):
        self.frame_count = 0
        self.running = True
        
        # Register cleanup handlers
        atexit.register(self.cleanup)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C"""
        logger.info("\nStopping test...")
        self.running = False
        
    def cleanup(self):
        """Cleanup resources"""
        try:
            # Resume program
            pixy.resume()
        except:
            pass
        
    def init_pixy(self):
        """Initialize Pixy2 camera"""
        logger.info("Initializing Pixy2 camera...")
        
        # Initialize Pixy2
        ret = pixy.init()
        if ret < 0:
            logger.error(f"Pixy2 initialization failed with code: {ret}")
            return False
            
        return True
        
    def run_test(self, duration=10):
        """Test raw frame capture timing"""
        if not self.init_pixy():
            return
            
        logger.info("Testing raw frame capture...")
        
        # Switch to video mode and stop for raw frames
        pixy.change_prog("video")
        pixy.stop()
        
        # Allocate frame buffer
        try:
            frame_buffer = pixy.RawFrame(FRAME_WIDTH * FRAME_HEIGHT)
        except Exception as e:
            logger.error(f"Error allocating frame buffer: {str(e)}")
            return
            
        start_time = time.time()
        frame_times = []
        
        while self.running and time.time() - start_time < duration:
            frame_start = time.time()
            frame_result = pixy.get_raw_frame(frame_buffer)
            if frame_result < 0:
                logger.error(f"Frame capture failed with code: {frame_result}")
                break
            frame_time = (time.time() - frame_start) * 1000
            frame_times.append(frame_time)
            self.frame_count += 1
            
            # Log progress every second
            if self.frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = self.frame_count / elapsed
                logger.info(f"Progress: {elapsed:.1f}s, FPS: {fps:.1f}")
                
        # Calculate statistics
        if frame_times:
            avg_time = sum(frame_times) / len(frame_times)
            max_time = max(frame_times)
            min_time = min(frame_times)
            fps = self.frame_count / duration if duration > 0 else 0
            
            logger.info("\n=== RAW FRAME CAPTURE RESULTS ===")
            logger.info(f"Frames captured: {self.frame_count}")
            logger.info(f"Average time: {avg_time:.2f}ms")
            logger.info(f"Maximum time: {max_time:.2f}ms")
            logger.info(f"Minimum time: {min_time:.2f}ms")
            logger.info(f"Average FPS: {fps:.1f}")
            
        # Resume program
        pixy.resume()

if __name__ == "__main__":
    try:
        test = TimingTest()
        test.run_test()
    except Exception as e:
        logger.error(f"Test failed: {str(e)}")
        sys.exit(1) 