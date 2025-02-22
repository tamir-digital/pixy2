from __future__ import print_function
import pixy
from ctypes import *
from pixy import *
import sys
import tty
import termios
import time

# Pixy2 Python SWIG manual pan/tilt demo #

# Constants #
PIXY_RCS_MAXIMUM_POSITION = 1000
PIXY_RCS_MINIMUM_POSITION = 0
PIXY_RCS_CENTER_POSITION = ((PIXY_RCS_MAXIMUM_POSITION - PIXY_RCS_MINIMUM_POSITION) / 2)
PAN_STEP = 20   # Amount to move per key press
TILT_STEP = 20  # Amount to move per key press

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print("Pixy2 Python SWIG Example -- Manual Pan/Tilt Control")

# Initialize Pixy2
print("Initializing Pixy2...")
init_result = pixy.init()
if init_result < 0:
    print("Error: Pixy2 initialization failed with code:", init_result)
    sys.exit(1)
print("Pixy2 initialized successfully!")

# Change program to video mode
print("Changing to video mode...")
prog_result = pixy.change_prog("video")
if prog_result < 0:
    print("Error: Failed to change to video mode. Error code:", prog_result)
    sys.exit(1)
print("Successfully changed to video mode!")

# Initialize servo positions to center
pan_pos = PIXY_RCS_CENTER_POSITION
tilt_pos = PIXY_RCS_CENTER_POSITION

# Set initial positions
print("Setting initial servo positions...")
try:
    pixy.set_servos(int(pan_pos), int(tilt_pos))
    print("Servos initialized successfully!")
except Exception as e:
    print("Error: Failed to set initial servo positions:", str(e))
    sys.exit(1)

print("\nControls:")
print("a/d : Pan left/right")
print("w/s : Tilt up/down")
print("c   : Center servos")
print("q   : Quit")
print("\nCurrent position - Pan: {:.0f}, Tilt: {:.0f}".format(pan_pos, tilt_pos))

try:
    while True:
        char = getch().lower()
        
        old_pan = pan_pos
        old_tilt = tilt_pos
        
        if char == 'a':  # left
            pan_pos = max(PIXY_RCS_MINIMUM_POSITION, pan_pos - PAN_STEP)
        elif char == 'd':  # right
            pan_pos = min(PIXY_RCS_MAXIMUM_POSITION, pan_pos + PAN_STEP)
        elif char == 'w':  # up
            tilt_pos = min(PIXY_RCS_MAXIMUM_POSITION, tilt_pos + TILT_STEP)
        elif char == 's':  # down
            tilt_pos = max(PIXY_RCS_MINIMUM_POSITION, tilt_pos - TILT_STEP)
        elif char == 'c':  # center
            pan_pos = PIXY_RCS_CENTER_POSITION
            tilt_pos = PIXY_RCS_CENTER_POSITION
        elif char == 'q':  # quit
            break
        
        # Only update servos if position changed
        if old_pan != pan_pos or old_tilt != tilt_pos:
            try:
                pixy.set_servos(int(pan_pos), int(tilt_pos))
                print("\rCurrent position - Pan: {:.0f}, Tilt: {:.0f}".format(pan_pos, tilt_pos), end='')
                sys.stdout.flush()
            except Exception as e:
                print("\nError: Failed to set servo positions:", str(e))

except KeyboardInterrupt:
    print("\nExiting...")
except Exception as e:
    print("\nError occurred:", str(e))

finally:
    # Center the servos before exiting
    print("\nCentering servos...")
    try:
        pixy.set_servos(int(PIXY_RCS_CENTER_POSITION), int(PIXY_RCS_CENTER_POSITION))
        print("Servos centered successfully!")
    except Exception as e:
        print("Error centering servos:", str(e)) 