#!/usr/bin/env python3

import sys
import numpy as np
import cv2
import pixy
import ctypes

print("Pixy2 Python SWIG Example -- Get Raw Frames")
print(f"OpenCV version: {cv2.__version__}")
print(f"NumPy version: {np.__version__}")

# Initialize Pixy2
print("Initializing Pixy2...")
ret = pixy.init()
print(f"init() returned: {ret}")

if ret < 0:
    print("Error initializing Pixy2")
    sys.exit(1)

# Get frame dimensions
FRAME_WIDTH = pixy.get_raw_frame_width()
FRAME_HEIGHT = pixy.get_raw_frame_height()

print("Switching to video mode...")
print(f"Frame dimensions: {FRAME_WIDTH}x{FRAME_HEIGHT}")

# Create video writer
print(f"Creating video writer with dimensions: {FRAME_WIDTH}x{FRAME_HEIGHT}")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 30.0, (FRAME_WIDTH, FRAME_HEIGHT))

# Allocate raw frame buffer
print("Allocating raw frame buffer...")
try:
    frame_buffer = pixy.RawFrame(FRAME_WIDTH * FRAME_HEIGHT)
except Exception as e:
    print(f"Error allocating raw frame buffer: {str(e)}")
    sys.exit(1)

def bayer_to_rgb(bayer_frame):
    """Convert Bayer BGGR format to RGB"""
    # Get the raw pointer as an integer
    ptr = int(bayer_frame.cast())
    # Create a ctypes array from the pointer
    buf = (ctypes.c_uint8 * (FRAME_WIDTH * FRAME_HEIGHT)).from_address(ptr)
    # Convert to numpy array
    bayer_data = np.frombuffer(buf, dtype=np.uint8, count=FRAME_WIDTH * FRAME_HEIGHT)
    # Reshape the flat array to 2D
    bayer = bayer_data.reshape((FRAME_HEIGHT, FRAME_WIDTH))
    # Convert BGGR to RGB using OpenCV
    rgb = cv2.cvtColor(bayer, cv2.COLOR_BayerBG2RGB)
    return rgb

try:
    # Stop current program
    pixy.stop()
    
    while True:
        # Get raw frame
        ret = pixy.get_raw_frame(frame_buffer)
        if ret < 0:
            print(f"Error getting frame: {ret}")
            break
            
        # Convert Bayer to RGB
        frame = bayer_to_rgb(frame_buffer)
        
        # Write frame
        out.write(frame)
        
        # Display frame
        cv2.imshow('Pixy2 Raw Frame', frame)
        
        # Break on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
finally:
    # Cleanup
    pixy.resume()  # Resume Pixy2
    out.release()
    cv2.destroyAllWindows()

print("Saved video to output.avi") 