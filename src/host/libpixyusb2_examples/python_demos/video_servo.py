#!/usr/bin/env python3

from __future__ import print_function
import pixy
from ctypes import *
from pixy import *
import sys
import tty
import termios
import time
import select
import numpy as np
import cv2

# Constants
PIXY_RCS_MAXIMUM_POSITION = 1000
PIXY_RCS_MINIMUM_POSITION = 0
PIXY_RCS_CENTER_POSITION = ((PIXY_RCS_MAXIMUM_POSITION - PIXY_RCS_MINIMUM_POSITION) / 2)
PAN_STEP = 100
TILT_STEP = 100

# PID Controller Constants
PID_MAXIMUM_INTEGRAL = 2000
PID_MINIMUM_INTEGRAL = -2000
PAN_GAIN = 400
TILT_GAIN = 500
UPDATE_INTERVAL = 0.016  # 60Hz update rate

class PID_Controller:
    def __init__(self, proportion_gain, integral_gain, derivative_gain, servo):
        self.proportion_gain = proportion_gain
        self.integral_gain = integral_gain
        self.derivative_gain = derivative_gain
        self.servo = servo
        self.reset()

    def reset(self):
        self.previous_error = 0x80000000
        self.integral_value = 0
        self.command = PIXY_RCS_CENTER_POSITION if self.servo else 0

    def update(self, error):
        if self.previous_error != 0x80000000:
            self.integral_value = self.integral_value + error
            self.integral_value = min(max(self.integral_value, PID_MINIMUM_INTEGRAL), PID_MAXIMUM_INTEGRAL)
            pid = int(error * self.proportion_gain + 
                     (int(self.integral_value * self.integral_gain) >> 4) + 
                     (error - self.previous_error) * self.derivative_gain) >> 10
            if self.servo:
                self.command = min(max(self.command + pid, PIXY_RCS_MINIMUM_POSITION), PIXY_RCS_MAXIMUM_POSITION)
        self.previous_error = error
        return self.command

def is_data():
    """Check if there is data waiting on stdin"""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getch_non_blocking():
    """Non-blocking getch"""
    if is_data():
        return sys.stdin.read(1)
    return None

def init_terminal():
    """Initialize terminal for non-blocking input"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
    except:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return fd, old_settings

def restore_terminal(fd, old_settings):
    """Restore terminal settings"""
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def bayer_to_rgb(bayer_frame, width, height):
    """Convert Bayer BGGR format to RGB"""
    ptr = int(bayer_frame.cast())
    buf = (c_uint8 * (width * height)).from_address(ptr)
    bayer_data = np.frombuffer(buf, dtype=np.uint8, count=width * height)
    bayer = bayer_data.reshape((height, width))
    rgb = cv2.cvtColor(bayer, cv2.COLOR_BayerBG2RGB)
    return rgb

def main():
    print("Pixy2 Python SWIG Example -- Video Recording with Pan/Tilt Control")
    print(f"OpenCV version: {cv2.__version__}")
    print(f"NumPy version: {np.__version__}")

    # Initialize Pixy2
    print("Initializing Pixy2...")
    init_result = pixy.init()
    if init_result < 0:
        print("Error: Pixy2 initialization failed with code:", init_result)
        sys.exit(1)
    print("Pixy2 initialized successfully!")

    # Switch to video mode
    print("Switching to video mode...")
    prog_result = pixy.change_prog("video")
    if prog_result < 0:
        print("Error: Failed to change to video mode. Error code:", prog_result)
        sys.exit(1)
    print("Successfully changed to video mode!")

    # Stop current program to allow raw frame capture
    pixy.stop()

    # Get frame dimensions
    FRAME_WIDTH = pixy.get_raw_frame_width()
    FRAME_HEIGHT = pixy.get_raw_frame_height()
    print(f"Frame dimensions: {FRAME_WIDTH}x{FRAME_HEIGHT}")

    # Create video writer
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 30.0, (FRAME_WIDTH, FRAME_HEIGHT))

    # Allocate raw frame buffer
    try:
        frame_buffer = pixy.RawFrame(FRAME_WIDTH * FRAME_HEIGHT)
    except Exception as e:
        print(f"Error allocating raw frame buffer: {str(e)}")
        sys.exit(1)

    # Initialize PID controllers
    pan_controller = PID_Controller(PAN_GAIN, 0, PAN_GAIN, True)
    tilt_controller = PID_Controller(TILT_GAIN, 0, TILT_GAIN, True)

    # Initialize positions
    current_pan = PIXY_RCS_CENTER_POSITION
    current_tilt = PIXY_RCS_CENTER_POSITION
    target_pan = PIXY_RCS_CENTER_POSITION
    target_tilt = PIXY_RCS_CENTER_POSITION

    # Set initial positions
    print("Setting initial servo positions...")
    try:
        pixy.set_servos(int(current_pan), int(current_tilt))
        print("Servos initialized successfully!")
    except Exception as e:
        print("Error: Failed to set initial servo positions:", str(e))
        sys.exit(1)

    print("\nControls:")
    print("a/d : Pan left/right")
    print("w/s : Tilt up/down")
    print("c   : Center servos")
    print("q   : Quit")

    # Initialize terminal for non-blocking input
    fd, old_settings = init_terminal()

    try:
        last_update = time.time()
        
        while True:
            # Handle input
            char = getch_non_blocking()
            if char:
                char = char.lower()
                if char == 'a':  # left
                    target_pan = max(PIXY_RCS_MINIMUM_POSITION, target_pan - PAN_STEP)
                elif char == 'd':  # right
                    target_pan = min(PIXY_RCS_MAXIMUM_POSITION, target_pan + PAN_STEP)
                elif char == 'w':  # up
                    target_tilt = min(PIXY_RCS_MAXIMUM_POSITION, target_tilt + TILT_STEP)
                elif char == 's':  # down
                    target_tilt = max(PIXY_RCS_MINIMUM_POSITION, target_tilt - TILT_STEP)
                elif char == 'c':  # center
                    target_pan = PIXY_RCS_CENTER_POSITION
                    target_tilt = PIXY_RCS_CENTER_POSITION
                elif char == 'q':  # quit
                    break

            # Get raw frame and process it
            ret = pixy.get_raw_frame(frame_buffer)
            if ret >= 0:
                frame = bayer_to_rgb(frame_buffer, FRAME_WIDTH, FRAME_HEIGHT)
                out.write(frame)
                cv2.imshow('Pixy2 Raw Frame', frame)
                cv2.waitKey(1)

            # Update movement at fixed interval
            current_time = time.time()
            if current_time - last_update >= UPDATE_INTERVAL:
                # Calculate errors
                pan_error = target_pan - current_pan
                tilt_error = target_tilt - current_tilt

                # Update PID controllers
                new_pan = pan_controller.update(pan_error)
                new_tilt = tilt_controller.update(tilt_error)

                try:
                    pixy.set_servos(int(new_pan), int(new_tilt))
                    current_pan = new_pan
                    current_tilt = new_tilt
                    print(f"\rCurrent position - Pan: {current_pan:.0f}, Tilt: {current_tilt:.0f}", end='')
                    sys.stdout.flush()
                except Exception as e:
                    print("\nError updating servos:", str(e))

                last_update = current_time

            # Small sleep to prevent CPU hogging
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print("\nError occurred:", str(e))

    finally:
        # Cleanup
        restore_terminal(fd, old_settings)
        
        # Resume Pixy2 program
        pixy.resume()
        
        # Center the servos before exiting
        print("\nCentering servos...")
        try:
            pixy.set_servos(int(PIXY_RCS_CENTER_POSITION), int(PIXY_RCS_CENTER_POSITION))
            print("Servos centered successfully!")
        except Exception as e:
            print("Error centering servos:", str(e))
        
        # Cleanup video resources
        out.release()
        cv2.destroyAllWindows()
        print("Saved video to output.avi")

if __name__ == "__main__":
    main() 