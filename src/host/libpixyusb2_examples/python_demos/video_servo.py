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
import os
from datetime import datetime
import logging
import json
from contextlib import contextmanager
import io
import fcntl
import platform
import atexit

# Permanently redirect stderr to /dev/null
stderr_dev_null = open(os.devnull, 'w')
os.dup2(stderr_dev_null.fileno(), sys.stderr.fileno())

# Register cleanup to prevent file descriptor leaks
def cleanup_stderr():
    stderr_dev_null.close()
atexit.register(cleanup_stderr)

# Immediately suppress system messages on macOS
if platform.system() == 'Darwin':
    # Set environment variables
    os.environ['OBJC_DEBUG_MISSING_POOLS'] = 'NO'
    os.environ['OBJC_DISABLE_GC'] = 'YES'
    os.environ['LIBUSB_DEBUG'] = '0'  # Disable libusb debug output
    os.environ['LIBUSB_LOG_LEVEL'] = '0'  # Set libusb log level to none

# Default configuration
DEFAULT_CONFIG = {
    "video": {
        "output_dir": "output",
        "codec": "XVID",
        "fps": 30,
        "show_preview": True
    },
    "servo": {
        "pan_step": 100,
        "tilt_step": 100,
        "update_interval": 0.016,  # 60Hz
        "pan_gain": 400,
        "tilt_gain": 500,
        "integral": {
            "enabled": True,
            "decay_factor": 0.95,  # Integral memory decay (1.0 = no decay)
            "max_limit": 2000,     # Maximum integral accumulation
            "min_limit": -2000,    # Minimum integral accumulation
            "anti_windup": True    # Enable back-calculation anti-windup
        },
        "deadband": {
            "enabled": True,
            "error": 2.0,        # Ignore errors smaller than this
            "output": 1.0,       # Don't move if command change smaller than this
            "integral_error": 0.5 # Smaller deadband for integral accumulation
        }
    },
    "debug": {
        "log_level": "INFO",
        "show_fps": True,
        "suppress_pixy_debug": True,
        "pid_debug": False
    }
}

@contextmanager
def suppress_stdout_stderr():
    """Context manager to capture and suppress stdout and stderr"""
    # Save the original stdout/stderr
    old_stdout = sys.stdout
    old_stderr = sys.stderr
    
    # Create file descriptors for stdout/stderr
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save the original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open null device
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Replace file descriptors
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        # Create string buffers for Python-level capture
        stdout_buffer = io.StringIO()
        stderr_buffer = io.StringIO()
        
        # Replace Python-level stdout/stderr
        sys.stdout = stdout_buffer
        sys.stderr = stderr_buffer
        
        yield
    finally:
        # Restore Python-level stdout/stderr
        sys.stdout = old_stdout
        sys.stderr = old_stderr
        
        # Restore file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)
        
        # Get the captured output
        stdout_value = stdout_buffer.getvalue()
        stderr_value = stderr_buffer.getvalue()
        
        # If there's any stderr content, log it as error
        if stderr_value:
            logging.error(stderr_value.strip())
        
        # If there's any stdout content, log it as debug
        if stdout_value:
            logging.debug(stdout_value.strip())

def load_config():
    """Load configuration from file or create default"""
    config_path = os.path.join(os.path.dirname(__file__), 'video_servo_config.json')
    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
                # Merge with defaults to ensure all keys exist
                merged = DEFAULT_CONFIG.copy()
                merged.update(config)
                return merged
        else:
            # Save default config
            with open(config_path, 'w') as f:
                json.dump(DEFAULT_CONFIG, f, indent=4)
            return DEFAULT_CONFIG
    except Exception as e:
        logging.warning(f"Error loading config, using defaults: {str(e)}")
        return DEFAULT_CONFIG

# Load configuration
CONFIG = load_config()

# Constants
PIXY_RCS_MAXIMUM_POSITION = 1000
PIXY_RCS_MINIMUM_POSITION = 0
PIXY_RCS_CENTER_POSITION = ((PIXY_RCS_MAXIMUM_POSITION - PIXY_RCS_MINIMUM_POSITION) / 2)
PAN_STEP = CONFIG['servo']['pan_step']
TILT_STEP = CONFIG['servo']['tilt_step']

# PID Controller Constants
PID_MAXIMUM_INTEGRAL = 2000
PID_MINIMUM_INTEGRAL = -2000
PAN_GAIN = CONFIG['servo']['pan_gain']
TILT_GAIN = CONFIG['servo']['tilt_gain']
UPDATE_INTERVAL = CONFIG['servo']['update_interval']

class PID_Controller:
    def __init__(self, proportion_gain, integral_gain, derivative_gain, servo):
        # Convert gains to float equivalents of the bit-shifted values
        self.proportion_gain = float(proportion_gain) / 1024.0  # Equivalent of >> 10
        self.integral_gain = float(integral_gain) / 16384.0    # Equivalent of >> 14 (>> 10 and >> 4)
        self.derivative_gain = float(derivative_gain) / 1024.0 * UPDATE_INTERVAL  # Scale for time-based calculation
        self.servo = servo
        self.debug = CONFIG['debug'].get('pid_debug', False)
        
        # Get integral configuration
        integral_config = CONFIG['servo'].get('integral', {})
        self.integral_enabled = integral_config.get('enabled', True)
        self.integral_decay = integral_config.get('decay_factor', 1.0)
        self.integral_max = float(integral_config.get('max_limit', PID_MAXIMUM_INTEGRAL))
        self.integral_min = float(integral_config.get('min_limit', PID_MINIMUM_INTEGRAL))
        self.anti_windup = integral_config.get('anti_windup', True)
        
        # Get deadband configuration
        deadband_config = CONFIG['servo'].get('deadband', {})
        self.deadband_enabled = deadband_config.get('enabled', True)
        self.error_deadband = float(deadband_config.get('error', 2.0))
        self.output_deadband = float(deadband_config.get('output', 1.0))
        self.integral_error_deadband = float(deadband_config.get('integral_error', 0.5))
        
        # Safety checks
        if self.integral_gain == 0:
            self.integral_enabled = False
            self.anti_windup = False
            if self.debug:
                logging.warning("Integral control disabled due to zero gain")
        
        self.reset()

    def reset(self):
        self.previous_error = None
        self.previous_time = None
        self.integral_value = 0.0
        self.command = float(PIXY_RCS_CENTER_POSITION if self.servo else 0)
        self.previous_command = self.command
        self.last_pid = 0.0  # Store last PID output for anti-windup

    def apply_deadband(self, value, deadband):
        """Apply deadband to a value"""
        if abs(value) <= deadband:
            return 0.0
        return value

    def update(self, error):
        current_time = time.time()
        
        if self.previous_error is not None and self.previous_time is not None:
            # Calculate time delta
            dt = current_time - self.previous_time
            if dt <= 0:  # Avoid division by zero or negative time
                dt = UPDATE_INTERVAL
            
            # Apply error deadband if enabled
            working_error = error
            if self.deadband_enabled:
                working_error = self.apply_deadband(error, self.error_deadband)
            
            # Calculate PID terms with float math
            p_term = working_error * self.proportion_gain
            
            # Enhanced integral term with time-based integration and decay
            if self.integral_enabled and self.integral_gain != 0:  # Extra safety check
                # Apply decay to existing integral
                self.integral_value *= self.integral_decay
                
                # Time-based integration (using smaller deadband for integral)
                integral_error = working_error if not self.deadband_enabled else self.apply_deadband(working_error, self.integral_error_deadband)
                self.integral_value += integral_error * dt
                
                # Anti-windup using clamping and back-calculation
                if self.anti_windup:
                    if self.command != self.previous_command:
                        command_delta = self.command - self.previous_command
                        if abs(command_delta) < abs(self.last_pid):
                            windup_scale = 0.9  # Reduce integral by 10% when limited
                            self.integral_value *= windup_scale
                
                # Apply limits
                self.integral_value = min(max(self.integral_value, self.integral_min), self.integral_max)
                
                i_term = self.integral_value * self.integral_gain
            else:
                i_term = 0.0
            
            # Time-based derivative (using main error deadband)
            if self.previous_error is not None:
                prev_working_error = self.previous_error
                if self.deadband_enabled:
                    prev_working_error = self.apply_deadband(self.previous_error, self.error_deadband)
                d_term = ((working_error - prev_working_error) / dt) * self.derivative_gain
            else:
                d_term = 0.0
            
            # Combine terms
            pid = p_term + i_term + d_term
            self.last_pid = pid  # Store for anti-windup
            
            if self.debug:
                logging.debug(f"PID terms - P: {p_term:.4f}, I: {i_term:.4f}, D: {d_term:.4f}, "
                            f"Total: {pid:.4f}, dt: {dt:.4f}, Integral: {self.integral_value:.4f}, "
                            f"Error: {error:.1f}, Working Error: {working_error:.1f}")
            
            if self.servo:
                self.previous_command = self.command
                # Calculate new command
                new_command = self.command + pid
                
                # Apply output deadband if enabled
                if self.deadband_enabled:
                    if abs(new_command - self.command) <= self.output_deadband:
                        new_command = self.command
                
                # Apply limits and convert to integer
                self.command = min(max(new_command, float(PIXY_RCS_MINIMUM_POSITION)), float(PIXY_RCS_MAXIMUM_POSITION))
        
        self.previous_error = float(error)
        self.previous_time = current_time
        return int(round(self.command))  # Convert to integer for servo

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

def setup_logging():
    """Setup logging configuration"""
    # Create logs directory if it doesn't exist
    logs_dir = "logs"
    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)
    
    # Generate log filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(logs_dir, f"pixy2_session_{timestamp}.log")
    
    # Remove any existing handlers
    root = logging.getLogger()
    for handler in root.handlers[:]:
        root.removeHandler(handler)
    
    # Create logger
    root.setLevel(logging.DEBUG)  # Capture everything at root level
    
    # File handler - gets everything
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    root.addHandler(file_handler)
    
    # Console handler - only gets important messages
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.WARNING)  # Only show warnings and errors
    console_handler.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))
    
    # Add filter to capture Pixy2 debug messages
    class Pixy2DebugFilter(logging.Filter):
        def filter(self, record):
            # Filter out debug messages and Pixy2-related messages from console
            if "Debug:" in record.msg or "Pixy2" in record.msg:
                return False
            return True

    # Add filter to console handler only
    console_handler.addFilter(Pixy2DebugFilter())
    root.addHandler(console_handler)
    
    # Configure other loggers to be quiet
    for name in ['pixy', 'opencv', 'PIL', 'OpenCV']:
        logger = logging.getLogger(name)
        logger.setLevel(logging.WARNING)
        logger.propagate = False
    
    # Create terminal handler class
    class TerminalHandler:
        def __init__(self):
            self.status_line = ""
            self.interface_shown = False
            self._suppress_output = False
            
        def suppress_output(self, suppress=True):
            """Temporarily suppress output"""
            self._suppress_output = suppress
            
        def clear_screen(self):
            """Clear the terminal screen"""
            if not self._suppress_output:
                print('\033[2J\033[H', end='', flush=True)  # Clear screen and move cursor to home
            self.interface_shown = False
            
        def show_interface(self, log_file):
            """Show the initial interface"""
            if not self.interface_shown and not self._suppress_output:
                self.clear_screen()
                lines = [
                    "\r=== Pixy2 Video Servo Control ===",
                    f"\rSession log: {os.path.basename(log_file)}",
                    "\r",
                    "\rControls (work in both terminal and video window):",
                    "\r  a/d : Pan left/right",
                    "\r  w/s : Tilt up/down", 
                    "\r  c   : Center servos",
                    "\r  r   : Toggle recording",
                    "\r  p   : Toggle preview",
                    "\r  q   : Quit",
                    "\r",
                    "\rStatus:"
                ]
                print('\n'.join(lines), flush=True)
                self.interface_shown = True
                
        def update_status(self, status):
            """Update the status line"""
            if not self._suppress_output:
                if self.status_line:
                    # Move up one line, to left margin, and clear line
                    print('\033[F\r\033[K', end='', flush=True)
                print('\r' + status, flush=True)
                self.status_line = status
            
        def show_message(self, message):
            """Show a temporary message without disturbing status"""
            if not self._suppress_output:
                if self.status_line:
                    # Save cursor, move up and left, clear line, show message, restore position
                    print(f'\033[s\033[F\r\033[K{message}\033[u', end='', flush=True)
                else:
                    print('\r' + message, flush=True)

    return log_file, TerminalHandler()

def ensure_output_dir():
    """Ensure output directory exists"""
    output_dir = CONFIG['video']['output_dir']
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    return output_dir

def get_video_path():
    """Generate timestamped video filename"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(ensure_output_dir(), f"pixy2_recording_{timestamp}.avi")

def handle_key_event(key, target_pan, target_tilt):
    """Handle keyboard input from both terminal and OpenCV window"""
    if key in [ord('a'), ord('A')]:  # left
        return max(PIXY_RCS_MINIMUM_POSITION, target_pan - PAN_STEP), target_tilt
    elif key in [ord('d'), ord('D')]:  # right
        return min(PIXY_RCS_MAXIMUM_POSITION, target_pan + PAN_STEP), target_tilt
    elif key in [ord('s'), ord('S')]:  # up
        return target_pan, min(PIXY_RCS_MAXIMUM_POSITION, target_tilt + TILT_STEP)
    elif key in [ord('w'), ord('W')]:  # down
        return target_pan, max(PIXY_RCS_MINIMUM_POSITION, target_tilt - TILT_STEP)
    elif key in [ord('c'), ord('C')]:  # center
        return PIXY_RCS_CENTER_POSITION, PIXY_RCS_CENTER_POSITION
    return target_pan, target_tilt

def init_pixy_with_suppression():
    """Initialize Pixy2 with all debug output suppressed"""
    # Try to access and modify the libusb context directly
    try:
        if hasattr(pixy, '_pixy'):
            # Get the underlying libusb context
            ctx = cast(pixy._pixy.get_libusb_context(), POINTER(c_void_p))
            if ctx:
                # Set libusb debug level to none (0)
                pixy._pixy.libusb_set_debug(ctx, 0)
    except:
        pass
    
    # Initialize Pixy2
    with suppress_stdout_stderr():
        return pixy.init()

def main():
    try:
        # Setup logging and get terminal handler
        log_file, terminal = setup_logging()
        
        # Disable debug output from Pixy2
        pixy.set_debug(False)
        
        # Initialize Pixy2 with all debug suppressed
        init_result = init_pixy_with_suppression()
        if init_result < 0:
            print(f"Failed to initialize Pixy2 (code: {init_result})")
            sys.exit(1)

        # Switch to video mode
        with suppress_stdout_stderr():
            prog_result = pixy.change_prog("video")
        if prog_result < 0:
            print(f"Failed to switch to video mode (code: {prog_result})")
            sys.exit(1)

        # Stop current program to allow raw frame capture
        with suppress_stdout_stderr():
            pixy.stop()

        # Get frame dimensions
        with suppress_stdout_stderr():
            FRAME_WIDTH = pixy.get_raw_frame_width()
            FRAME_HEIGHT = pixy.get_raw_frame_height()
        print(f"Frame dimensions: {FRAME_WIDTH}x{FRAME_HEIGHT}")

        # Create video writer with timestamped filename
        fourcc = cv2.VideoWriter_fourcc(*CONFIG['video']['codec'])
        video_path = get_video_path()
        out = cv2.VideoWriter(video_path, fourcc, CONFIG['video']['fps'], (FRAME_WIDTH, FRAME_HEIGHT))
        print(f"Recording video to: {video_path}")

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
        print("r   : Toggle recording")
        print("p   : Toggle preview")
        print("q   : Quit")

        # Initialize terminal for non-blocking input
        fd, old_settings = init_terminal()

        last_update = time.time()
        frames_processed = 0
        start_time = time.time()
        recording = True
        
        # Show interface
        terminal.show_interface(log_file)
        
        while True:
            # Handle terminal input
            char = getch_non_blocking()
            if char:
                key = ord(char.lower())
                if key == ord('q'):  # quit
                    break
                elif key == ord('r'):  # toggle recording
                    recording = not recording
                    if recording:
                        video_path = get_video_path()
                        out = cv2.VideoWriter(video_path, 
                                           cv2.VideoWriter_fourcc(*CONFIG['video']['codec']), 
                                           CONFIG['video']['fps'], 
                                           (FRAME_WIDTH, FRAME_HEIGHT))
                        print(f"Started recording to: {video_path}")
                    else:
                        out.release()
                        print(f"Stopped recording: {video_path}")
                elif key == ord('p'):  # toggle preview
                    CONFIG['video']['show_preview'] = not CONFIG['video']['show_preview']
                    if not CONFIG['video']['show_preview']:
                        cv2.destroyAllWindows()
                else:
                    # Handle pan/tilt commands
                    target_pan, target_tilt = handle_key_event(key, target_pan, target_tilt)

            # Get raw frame and process it
            ret = -1
            with suppress_stdout_stderr():
                ret = pixy.get_raw_frame(frame_buffer)
            if ret >= 0:
                frame = bayer_to_rgb(frame_buffer, FRAME_WIDTH, FRAME_HEIGHT)
                if recording:
                    out.write(frame)
                if CONFIG['video']['show_preview']:
                    cv2.imshow('Pixy2 Raw Frame', frame)
                    # Handle OpenCV window input
                    key = cv2.waitKey(1) & 0xFF
                    if key != 255:  # Key was pressed
                        if key == ord('q'):
                            break
                        elif key == ord('r'):
                            recording = not recording
                            if recording:
                                video_path = get_video_path()
                                out = cv2.VideoWriter(video_path, 
                                                   cv2.VideoWriter_fourcc(*CONFIG['video']['codec']), 
                                                   CONFIG['video']['fps'], 
                                                   (FRAME_WIDTH, FRAME_HEIGHT))
                                print(f"Started recording to: {video_path}")
                            else:
                                out.release()
                                print(f"Stopped recording: {video_path}")
                        elif key == ord('p'):
                            CONFIG['video']['show_preview'] = False
                            cv2.destroyAllWindows()
                        else:
                            # Handle pan/tilt commands
                            target_pan, target_tilt = handle_key_event(key, target_pan, target_tilt)
                frames_processed += 1

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
                    with suppress_stdout_stderr():
                        pixy.set_servos(int(new_pan), int(new_tilt))
                    current_pan = new_pan
                    current_tilt = new_tilt
                    
                    # Calculate FPS
                    elapsed_time = current_time - start_time
                    fps = frames_processed / elapsed_time if elapsed_time > 0 else 0
                    
                    # Update status display with fixed-width formatting
                    status = f"[{('REC' if recording else 'PAUSE'):4s}] "
                    status += f"Pan: {int(current_pan):4d} Tilt: {int(current_tilt):4d}"
                    if CONFIG['debug']['show_fps']:
                        status += f" FPS: {fps:5.1f}"
                    if recording:
                        status += f"  →  {os.path.basename(video_path)}"
                    terminal.update_status(status)
                    
                    # Log detailed status to file only
                    logging.debug(f"Status - Pan: {current_pan:.2f}, Tilt: {current_tilt:.2f}, "
                                f"FPS: {fps:.2f}, Recording: {recording}")
                    
                except Exception as e:
                    logging.error(f"Error updating servos: {str(e)}")
                    terminal.show_message("Error updating servos. Check log file for details.")

                last_update = current_time

            # Small sleep to prevent CPU hogging
            time.sleep(0.001)

    except KeyboardInterrupt:
        terminal.show_message("Exiting...")
    except Exception as e:
        logging.error(f"Error occurred: {str(e)}")
        terminal.show_message("Error occurred. Check log file for details.")

    finally:
        # First, cleanup video resources
        if 'out' in locals():
            out.release()
        cv2.destroyAllWindows()

        # Calculate final statistics
        elapsed_time = time.time() - start_time
        fps = frames_processed/elapsed_time if elapsed_time > 0 else 0
        
        # Suppress all Pixy2-related cleanup operations
        with suppress_stdout_stderr():
            try:
                # Center the servos before exiting
                pixy.set_servos(int(PIXY_RCS_CENTER_POSITION), int(PIXY_RCS_CENTER_POSITION))
                
                # Resume Pixy2 program
                pixy.resume()
                
                # Small delay to ensure commands are processed
                time.sleep(0.05)
            except Exception as e:
                logging.error(f"Error during Pixy2 cleanup: {str(e)}")

        # Restore terminal
        restore_terminal(fd, old_settings)
        
        # Print final status with a small delay to let any pending output finish
        time.sleep(0.1)  # Short delay
        sys.stdout.flush()  # Flush any pending output
        
        # Use ANSI escape sequences to clear from cursor to end of screen
        print("\033[J", end='')
        
        # Print final status
        print("\nFinal Status:")
        print(f"Duration: {elapsed_time:.1f} seconds")
        print(f"Frames Processed: {frames_processed}")
        print(f"Average FPS: {fps:.1f}")
        if recording:
            print(f"Last Recording: {video_path}")
        print("\nSession completed. Check log file for detailed statistics.")

if __name__ == "__main__":
    main() 