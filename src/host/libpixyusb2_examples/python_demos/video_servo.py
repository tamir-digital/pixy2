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

"""
Movement System Architecture
==========================

Overview
--------
The movement system provides smooth, precise servo control through a multi-layered approach:

Core Components
--------------
1. PID Controller (PID_Controller class)
   - Handles precise positioning and error correction
   - Features anti-windup and deadband handling
   - Configurable gains and limits
   - Integrates with motion profiling

2. Motion Profile Generator (MotionProfile class)
   - Generates smooth acceleration/deceleration curves
   - S-curve profiling for smooth transitions
   - Configurable smoothing and duration
   - Prevents mechanical stress through controlled motion

3. Velocity Control System
   - Manages speed and acceleration limits
   - Provides natural-feeling motion
   - Handles both manual control and automated movements
   - Smoothing and minimum speed thresholds

Movement State Management
-----------------------
- Key state tracking for user input
- Separate pan/tilt movement states
- Velocity and position history
- Configurable update intervals

Configuration System
------------------
The movement system is highly configurable through the CONFIG dictionary:
- PID gains and limits
- Motion profile parameters
- Velocity and acceleration limits
- Smoothing factors
- Debug options

Data Flow
---------
1. User Input → Key State
2. Key State → Target Velocity/Position
3. Motion Profile → Smooth Position Trajectory
4. PID Controller → Final Servo Commands

This architecture ensures:
- Smooth motion across different speeds
- Precise positioning
- Prevention of oscillation
- Configurable behavior for different use cases
"""

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

# Constants
PIXY_RCS_MAXIMUM_POSITION = 1000
PIXY_RCS_MINIMUM_POSITION = 0
PIXY_RCS_CENTER_POSITION = ((PIXY_RCS_MAXIMUM_POSITION - PIXY_RCS_MINIMUM_POSITION) / 2)

# Global velocity scale (can be adjusted at runtime)
VELOCITY_SCALE = 1.0
VELOCITY_SCALE_STEP = 0.25  # How much to change scale per keypress
VELOCITY_SCALE_MAX = 5.0    # Maximum allowed scale
VELOCITY_SCALE_MIN = 0.25   # Minimum allowed scale

# Default configuration
DEFAULT_CONFIG = {
    "video": {
        "output_dir": "output",
        "codec": "XVID",
        "fps": 30,
        "show_preview": True
    },
    "servo": {
        "update_interval": 0.016,  # 60Hz
        "pan_gain": 200,          # Reduced from 250
        "tilt_gain": 240,         # Reduced from 300
        "velocity": {
            "enabled": True,       # Enable velocity-based movement by default
            "max_speed": 50,       # Reduced from 100 - units per second
            "acceleration": 200,   # Reduced from 500 - units per second^2
            "ramp_time": 0.3,     # Increased from 0.2 - Time to reach full speed
            "min_speed": 2,       # Reduced from 5 - Minimum speed to maintain smooth motion
            "smoothing": {
                "enabled": True,
                "factor": 0.8,    # Higher smoothing (0-1)
                "window": 3       # Number of samples for smoothing
            },
            "phases": {
                "accelerating": {
                    "smoothing": 0.8,
                    "min_duration": 0.1,  # Minimum time in acceleration phase
                    "accel_scale": 1.0
                },
                "sustained": {
                    "smoothing": 0.2,
                    "velocity_maintain_factor": 0.95,  # Maintain at least 95% of target velocity
                    "accel_scale": 0.9
                },
                "decelerating": {
                    "smoothing": 0.7,
                    "min_duration": 0.05,  # Minimum deceleration time
                    "accel_scale": 0.8
                }
            }
        },
        "integral": {
            "enabled": False,     # Disabled integral term
            "decay_factor": 0.98,
            "max_limit": 1000,
            "min_limit": -1000,
            "anti_windup": True
        },
        "deadband": {
            "enabled": True,
            "error": 2.0,        # Reduced from 3.0
            "output": 1.5,       # Reduced from 2.0
            "integral_error": 1.0
        },
        "motion": {
            "enabled": True,
            "move_duration": 0.1,  # 100ms for smooth steps
            "debug": True,        # Enable motion profiling debug
            "acceleration": {
                "enabled": True,   # Enable acceleration control
                "max_accel": 1000, # Reduced from 5000 - Maximum acceleration
                "smoothing": 0.85  # Increased from 0.7 - Smoothing factor
            },
            "velocity": {
                "max": 500,       # Reduced from 2000 - Maximum velocity
                "min": 50         # Reduced from 100 - Minimum velocity
            }
        }
    },
    "movement_presets": {
        "current": "balanced",  # Default preset
        "presets": {
            "smooth": {
                "description": "Smooth, slower movements with high precision",
                "servo": {
                    "pan_gain": 180,
                    "tilt_gain": 220,
                    "velocity": {
                        "max_speed": 30,
                        "acceleration": 150,
                        "ramp_time": 0.4,
                        "min_speed": 1,
                        "smoothing": {
                            "factor": 0.9
                        }
                    },
                    "motion": {
                        "move_duration": 0.15,
                        "acceleration": {
                            "max_accel": 800,
                            "smoothing": 0.9
                        }
                    },
                    "deadband": {
                        "error": 1.5,
                        "output": 1.0
                    }
                }
            },
            "responsive": {
                "description": "Quick movements with less smoothing",
                "servo": {
                    "pan_gain": 220,
                    "tilt_gain": 260,
                    "velocity": {
                        "max_speed": 70,
                        "acceleration": 300,
                        "ramp_time": 0.2,
                        "min_speed": 3,
                        "smoothing": {
                            "factor": 0.7
                        }
                    },
                    "motion": {
                        "move_duration": 0.08,
                        "acceleration": {
                            "max_accel": 1200,
                            "smoothing": 0.7
                        }
                    },
                    "deadband": {
                        "error": 2.5,
                        "output": 2.0
                    }
                }
            },
            "precise": {
                "description": "High precision with balanced speed",
                "servo": {
                    "pan_gain": 200,
                    "tilt_gain": 240,
                    "velocity": {
                        "max_speed": 40,
                        "acceleration": 200,
                        "ramp_time": 0.3,
                        "min_speed": 1.5,
                        "smoothing": {
                            "factor": 0.85
                        }
                    },
                    "motion": {
                        "move_duration": 0.12,
                        "acceleration": {
                            "max_accel": 1000,
                            "smoothing": 0.85
                        }
                    },
                    "deadband": {
                        "error": 1.8,
                        "output": 1.2
                    }
                }
            },
            "balanced": {
                "description": "Default balanced behavior",
                "servo": {
                    "pan_gain": 200,
                    "tilt_gain": 240,
                    "velocity": {
                        "max_speed": 50,
                        "acceleration": 200,
                        "ramp_time": 0.3,
                        "min_speed": 2,
                        "smoothing": {
                            "factor": 0.8
                        }
                    },
                    "motion": {
                        "move_duration": 0.1,
                        "acceleration": {
                            "max_accel": 1000,
                            "smoothing": 0.85
                        }
                    },
                    "deadband": {
                        "error": 2.0,
                        "output": 1.5
                    }
                }
            }
        }
    },
    "debug": {
        "log_level": "DEBUG",  # Changed from INFO to DEBUG
        "show_fps": True,
        "suppress_pixy_debug": True,
        "pid_debug": True     # Enable PID debugging
    }
}

# Global key state tracking
key_states = {
    'a': {'pressed': False, 'press_start': 0, 'velocity': 0.0, 'target_velocity': 0.0},
    'd': {'pressed': False, 'press_start': 0, 'velocity': 0.0, 'target_velocity': 0.0},
    'w': {'pressed': False, 'press_start': 0, 'velocity': 0.0, 'target_velocity': 0.0},
    's': {'pressed': False, 'press_start': 0, 'velocity': 0.0, 'target_velocity': 0.0}
}

# Movement state tracking
movement_state = {
    'pan': {
        'velocity': 0.0,
        'last_update': 0,
        'last_position': PIXY_RCS_CENTER_POSITION,
        'movement_phase': 'idle',  # One of: idle, accelerating, sustained, decelerating
        'phase_start_time': 0,
        'initial_velocity': 0.0
    },
    'tilt': {
        'velocity': 0.0,
        'last_update': 0,
        'last_position': PIXY_RCS_CENTER_POSITION,
        'movement_phase': 'idle',
        'phase_start_time': 0,
        'initial_velocity': 0.0
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
    
    def deep_merge(source, destination):
        """Deep merge two dictionaries, ensuring all keys are preserved"""
        for key, value in source.items():
            if key in destination:
                if isinstance(value, dict) and isinstance(destination[key], dict):
                    deep_merge(value, destination[key])
                else:
                    # Always take source value if not a dict
                    destination[key] = value
            else:
                # Key not in destination, add it
                destination[key] = value
        return destination
    
    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                loaded_config = json.load(f)
                # Start with loaded config and merge defaults on top
                merged = loaded_config.copy()
                deep_merge(DEFAULT_CONFIG, merged)
                return merged
        else:
            # Save default config
            with open(config_path, 'w') as f:
                json.dump(DEFAULT_CONFIG, f, indent=4)
            return DEFAULT_CONFIG.copy()
    except Exception as e:
        logging.warning(f"Error loading config, using defaults: {str(e)}")
        return DEFAULT_CONFIG.copy()

# Load configuration
CONFIG = load_config()

def apply_movement_preset(preset_name, pan_controller=None, tilt_controller=None):
    """
    Apply a movement preset configuration.
    
    Args:
        preset_name (str): Name of the preset to apply ('smooth', 'responsive', 'precise', 'balanced')
        pan_controller (PID_Controller, optional): Pan axis controller to update
        tilt_controller (PID_Controller, optional): Tilt axis controller to update
        
    Returns:
        bool: True if preset was applied successfully, False otherwise
    """
    if preset_name not in CONFIG['movement_presets']['presets']:
        logging.error(f"Unknown movement preset: {preset_name}")
        return False
    
    try:
        # Get preset configuration
        preset = CONFIG['movement_presets']['presets'][preset_name]
        
        # Store previous state for logging
        prev_preset = CONFIG['movement_presets']['current']
        
        # Deep merge servo configuration
        def merge_config(source, dest):
            """Deep merge while preserving existing keys"""
            for key, value in source.items():
                if key in dest and isinstance(value, dict) and isinstance(dest[key], dict):
                    merge_config(value, dest[key])
                else:
                    dest[key] = value
            return dest
        
        # Create a copy of the current servo config
        new_servo_config = CONFIG['servo'].copy()
        
        # Merge preset servo config while preserving existing structure
        merge_config(preset['servo'], new_servo_config)
        
        # Ensure velocity configuration is complete
        if 'velocity' in new_servo_config:
            # Preserve enabled state from current config
            enabled_state = CONFIG['servo']['velocity'].get('enabled', True)
            new_servo_config['velocity']['enabled'] = enabled_state
        
        # Update servo configuration
        CONFIG['servo'].update(new_servo_config)
        
        # Update current preset
        CONFIG['movement_presets']['current'] = preset_name
        
        # Update controllers if provided
        if pan_controller is not None:
            pan_controller.proportion_gain = float(CONFIG['servo']['pan_gain']) / 1024.0
            pan_controller.derivative_gain = pan_controller.proportion_gain * UPDATE_INTERVAL
            
            # Reset integral term as gains changed
            if pan_controller.integral_enabled:
                pan_controller.integral_value = 0.0
            
            # Update deadband settings
            pan_controller.error_deadband = float(CONFIG['servo']['deadband']['error'])
            pan_controller.output_deadband = float(CONFIG['servo']['deadband']['output'])
            
            # Update motion profile
            pan_controller.motion = MotionProfile(CONFIG['servo']['motion'])
        
        if tilt_controller is not None:
            tilt_controller.proportion_gain = float(CONFIG['servo']['tilt_gain']) / 1024.0
            tilt_controller.derivative_gain = tilt_controller.proportion_gain * UPDATE_INTERVAL
            
            # Reset integral term as gains changed
            if tilt_controller.integral_enabled:
                tilt_controller.integral_value = 0.0
            
            # Update deadband settings
            tilt_controller.error_deadband = float(CONFIG['servo']['deadband']['error'])
            tilt_controller.output_deadband = float(CONFIG['servo']['deadband']['output'])
            
            # Update motion profile
            tilt_controller.motion = MotionProfile(CONFIG['servo']['motion'])
        
        logging.info(f"Applied movement preset: {preset_name} ({preset['description']})")
        if CONFIG['debug'].get('pid_debug', False):
            logging.debug(
                f"Preset change: {prev_preset} → {preset_name}\n"
                f"Pan gain: {CONFIG['servo']['pan_gain']}, "
                f"Tilt gain: {CONFIG['servo']['tilt_gain']}\n"
                f"Max speed: {CONFIG['servo']['velocity']['max_speed']}, "
                f"Acceleration: {CONFIG['servo']['velocity']['acceleration']}, "
                f"Enabled: {CONFIG['servo']['velocity']['enabled']}"
            )
        
        return True
        
    except Exception as e:
        logging.error(f"Failed to apply movement preset {preset_name}: {str(e)}")
        return False

# PID Controller Constants
PID_MAXIMUM_INTEGRAL = 2000
PID_MINIMUM_INTEGRAL = -2000
PAN_GAIN = CONFIG['servo']['pan_gain']
TILT_GAIN = CONFIG['servo']['tilt_gain']
UPDATE_INTERVAL = CONFIG['servo']['update_interval']

class MotionProfile:
    """
    Motion Profile Generator for Smooth Servo Movements
    
    Generates smooth motion profiles for servo movements using S-curve acceleration
    profiles. This ensures smooth transitions between positions while respecting
    velocity and acceleration limits.

    Features:
    ---------
    - S-curve acceleration profiles
    - Configurable smoothing factors
    - Velocity and acceleration limits
    - Real-time position updates
    - Movement state tracking
    - Debug logging capabilities

    Configuration:
    -------------
    The profile behavior is configured through the CONFIG dictionary:
    - Movement duration
    - Maximum acceleration
    - Velocity limits
    - Smoothing factors
    - Debug options

    The profile ensures:
    - Smooth starts and stops
    - Controlled acceleration
    - Minimal mechanical stress
    - Natural-feeling motion
    """
    
    def __init__(self, config=None):
        """
        Initialize motion profile generator with configuration.
        
        Args:
            config (dict, optional): Configuration dictionary overriding defaults
                                   from CONFIG['servo']['motion']
        
        The initialization:
        1. Validates and processes configuration
        2. Sets up movement parameters
        3. Initializes state tracking
        4. Configures debug logging
        
        Raises:
            ValueError: If configuration is missing required fields
            TypeError: If configuration is not a dictionary
            RuntimeError: If initialization fails
        """
        try:
            # Ensure config is a dictionary
            if config is None:
                if 'servo' not in CONFIG or 'motion' not in CONFIG['servo']:
                    raise ValueError("Missing motion configuration in CONFIG")
                config = CONFIG['servo']['motion']
            elif not isinstance(config, dict):
                raise TypeError(f"Expected dict for config, got {type(config)}")
            
            self.config = config
            self.enabled = bool(self.config.get('enabled', True))
            self.move_duration = float(self.config.get('move_duration', 0.1))
            self.debug = bool(self.config.get('debug', False))
            
            # Get acceleration configuration
            accel_config = self.config.get('acceleration', {})
            self.accel_enabled = bool(accel_config.get('enabled', True))
            self.max_accel = float(accel_config.get('max_accel', 1000))
            self.smoothing = float(accel_config.get('smoothing', 0.85))
            
            # Get velocity configuration
            vel_config = self.config.get('velocity', {})
            self.max_vel = float(vel_config.get('max', 500))
            self.min_vel = float(vel_config.get('min', 50))
            
            # Movement state
            self.is_moving = False
            self.start_pos = 0
            self.target_pos = 0
            self.current_pos = 0
            self.start_time = None
            self.current_vel = 0
            self.distance = 0
            
            if self.debug:
                logging.info("Motion Profile initialized - "
                            f"enabled: {self.enabled}, "
                            f"duration: {self.move_duration:.3f}s, "
                            f"accel: {self.max_accel:.1f}, "
                            f"smoothing: {self.smoothing:.2f}")
                            
        except Exception as e:
            logging.error(f"Failed to initialize MotionProfile: {str(e)}, Config: {config}")
            raise RuntimeError(f"Motion Profile initialization failed: {str(e)}") from e
    
    def _s_curve_progress(self, t):
        """
        Calculate S-curve progress with configurable smoothing.
        
        Implements a sigmoid-based S-curve for smooth acceleration and deceleration.
        The curve is scaled and adjusted based on smoothing factors to ensure
        exact start and end positions.
        
        Args:
            t (float): Time progress from 0 to 1
            
        Returns:
            float: Position progress from 0 to 1
            
        The S-curve:
        1. Applies smoothing factor
        2. Uses sigmoid function for smooth transitions
        3. Scales output to exact range
        4. Handles boundary conditions
        """
        if t <= 0:
            return 0
        if t >= 1:
            return 1
            
        # Apply smoothing factor to create S-curve
        t = t * (1 + (1 - self.smoothing) * 2) - (1 - self.smoothing)
        progress = 1 / (1 + np.exp(-t * 12 + 6))  # Sigmoid function
        
        # Scale progress to ensure it reaches exactly 0 at t=0 and 1 at t=1
        progress = (progress - 1/(1 + np.exp(6))) / (1/(1 + np.exp(-6)) - 1/(1 + np.exp(6)))
        
        return float(progress)
    
    def start_move(self, from_pos, to_pos):
        """
        Start a new movement from current position to target.
        
        Args:
            from_pos (float): Starting position
            to_pos (float): Target position
            
        Returns:
            float: Initial position for the movement
            
        The method:
        1. Validates and stores movement parameters
        2. Initializes timing and state tracking
        3. Calculates movement distance
        4. Begins motion profile generation
        """
        if not self.enabled:
            return to_pos
            
        self.start_pos = float(from_pos)
        self.target_pos = float(to_pos)
        self.current_pos = self.start_pos
        self.start_time = time.time()
        self.is_moving = True
        self.current_vel = 0
        self.distance = abs(to_pos - from_pos)
        
        if self.debug:
            logging.debug(f"Starting move: {self.start_pos:.1f} → {self.target_pos:.1f}")
        
        return self.current_pos
    
    def update(self):
        """
        Update motion profile and return next position.
        
        Calculates the next position in the motion profile based on elapsed time
        and profile parameters. Handles acceleration, velocity, and position
        calculations.
        
        Returns:
            float or None: Next position in the profile, or None if movement complete
            
        The update process:
        1. Check if profile is active
        2. Calculate elapsed time
        3. Generate base progress
        4. Apply S-curve if acceleration enabled
        5. Calculate new position
        6. Update movement state
        7. Handle movement completion
        """
        if not self.enabled or not self.is_moving:
            return None
        
        elapsed = time.time() - self.start_time
        if elapsed >= self.move_duration:
            self.is_moving = False
            self.current_pos = self.target_pos
            self.current_vel = 0
            if self.debug:
                logging.debug(f"Move complete: {self.current_pos:.1f}")
            return self.current_pos
        
        # Calculate base progress
        progress = elapsed / self.move_duration
        
        if self.accel_enabled:
            # Use S-curve for smooth acceleration/deceleration
            progress = self._s_curve_progress(progress)
            
            # Calculate velocity for debugging
            if self.debug and elapsed > 0:
                self.current_vel = (progress - self._s_curve_progress((elapsed - 0.001) / self.move_duration)) * self.distance / 0.001
        else:
            # Linear interpolation if acceleration control is disabled
            if self.debug and elapsed > 0:
                self.current_vel = self.distance / self.move_duration
        
        # Calculate new position
        direction = 1 if self.target_pos > self.start_pos else -1
        self.current_pos = self.start_pos + direction * self.distance * progress
        
        if self.debug:
            logging.debug(f"Move progress: {progress:.3f}, pos: {self.current_pos:.1f}, "
                         f"vel: {self.current_vel:.1f}")
        
        return self.current_pos
    
    def is_profile_active(self):
        """
        Check if profile is currently generating positions.
        
        Returns:
            bool: True if profile is active and generating positions
        """
        return self.enabled and self.is_moving
    
    def get_target(self):
        """
        Get the final target position.
        
        Returns:
            float or None: Target position if profile is enabled, None otherwise
        """
        return self.target_pos if self.enabled else None

class PID_Controller:
    """
    Advanced PID Controller with Motion Profile Integration
    
    This controller provides precise positioning through PID control while incorporating
    motion profiling for smooth movements. It features anti-windup protection,
    deadband handling, and configurable gains.

    Features:
    ---------
    - Time-based derivative calculation
    - Integral anti-windup protection
    - Configurable deadband for error and output
    - Motion profile integration for smooth transitions
    - Decay factor for integral term
    - Automatic gain scaling

    Configuration:
    -------------
    The controller behavior can be tuned through the CONFIG dictionary:
    - PID gains (proportion, integral, derivative)
    - Integral limits and decay
    - Deadband thresholds
    - Motion profile parameters

    Args:
        proportion_gain (float): The proportional gain (scaled by 1024)
        integral_gain (float): The integral gain (scaled by 16384)
        derivative_gain (float): The derivative gain (scaled by 1024)
        servo (bool): Whether this controller is for a servo (affects center position)
    """
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
        
        # Initialize motion profile
        motion_config = CONFIG['servo'].get('motion', {})
        self.motion = MotionProfile(config=motion_config)
        self.target_position = PIXY_RCS_CENTER_POSITION if servo else 0
        
        # Safety checks
        if self.integral_gain == 0:
            self.integral_enabled = False
            self.anti_windup = False
            if self.debug:
                logging.warning("Integral control disabled due to zero gain")
        
        self.reset()

    def reset(self):
        """
        Reset the controller state.
        
        Resets all internal state variables to their initial values:
        - Previous error and time
        - Integral accumulator
        - Command and target values
        - PID output history
        """
        self.previous_error = None
        self.previous_time = None
        self.integral_value = 0.0
        self.command = float(PIXY_RCS_CENTER_POSITION if self.servo else 0)
        self.previous_command = self.command
        self.last_pid = 0.0  # Store last PID output for anti-windup
        self.target_position = self.command

    def set_target(self, new_target):
        """
        Set a new target position with motion profile integration.
        
        Args:
            new_target (float): The desired target position
            
        The method:
        1. Calculates the required movement size
        2. Determines if motion profiling should be used
        3. Either starts a new motion profile or updates target directly
        4. Updates internal target state
        """
        # Calculate step size
        step_size = abs(new_target - self.target_position)
        
        # Only use motion profile for significant changes
        if self.motion.enabled and step_size > self.error_deadband:
            # Start a new motion profile from current command to new target
            self.motion.start_move(self.command, new_target)
            if self.debug:
                logging.debug(f"Starting profiled move: {self.command:.1f} → {new_target:.1f}")
        else:
            # Small change, update target directly
            if self.debug:
                logging.debug(f"Direct target update: {self.target_position:.1f} → {new_target:.1f}")
        
        self.target_position = new_target

    def apply_deadband(self, value, deadband):
        """
        Apply deadband to a value to prevent small oscillations.
        
        Args:
            value (float): The input value to apply deadband to
            deadband (float): The deadband threshold
            
        Returns:
            float: The value after deadband application (0 if within deadband)
        """
        if abs(value) <= deadband:
            return 0.0
        return value

    def update(self, error):
        """
        Update the controller state and calculate new output.
        """
        current_time = time.time()
        
        # Check if we're following a motion profile
        if self.motion.is_profile_active():
            # Only use motion profile for transitional movements (not continuous)
            if any(k['pressed'] for k in key_states.values()):
                # During continuous movement, bypass motion profile
                self.motion.is_moving = False
                error = self.target_position - self.command
            else:
                # Get next position from profile for non-continuous movements
                profile_pos = self.motion.update()
                if profile_pos is not None:
                    error = profile_pos - self.command
                else:
                    error = self.target_position - self.command
        
        if self.previous_error is not None and self.previous_time is not None:
            # Calculate time delta
            dt = current_time - self.previous_time
            if dt <= 0:  # Avoid division by zero or negative time
                dt = UPDATE_INTERVAL
            
            # Apply error deadband if enabled
            working_error = error
            if self.deadband_enabled:
                working_error = self.apply_deadband(error, self.error_deadband)
            
            # Get velocity from movement state instead of calculating it
            if self.servo:
                axis = 'pan' if self.proportion_gain == CONFIG['servo']['pan_gain'] else 'tilt'
                current_velocity = movement_state[axis]['velocity']
                
                # Check if we're in continuous movement
                is_continuous = any(k['pressed'] for k in key_states.values())
                if is_continuous:
                    # During continuous movement, use velocity-based control
                    # Reduce derivative term influence to prevent fighting the velocity system
                    d_term = -current_velocity * (self.derivative_gain * 0.25)  # Further reduced gain
                else:
                    # Normal derivative term for transitional movements
                    d_term = -current_velocity * (self.derivative_gain * 0.5)
            else:
                current_velocity = 0.0
                d_term = 0.0
            
            # Calculate PID terms with float math
            p_term = working_error * self.proportion_gain
            
            # Enhanced integral term with time-based integration and decay
            if self.integral_enabled and self.integral_gain != 0:
                prev_integral = self.integral_value
                
                # Apply decay to existing integral
                self.integral_value *= self.integral_decay
                
                # Time-based integration
                integral_error = working_error if not self.deadband_enabled else self.apply_deadband(working_error, self.integral_error_deadband)
                self.integral_value += integral_error * dt
                
                # Anti-windup using clamping and back-calculation
                if self.anti_windup:
                    if self.command != self.previous_command:
                        command_delta = self.command - self.previous_command
                        if abs(command_delta) < abs(self.last_pid):
                            windup_scale = 0.9
                            self.integral_value *= windup_scale
                
                # Apply limits
                self.integral_value = min(max(self.integral_value, self.integral_min), self.integral_max)
                
                i_term = self.integral_value * self.integral_gain
            else:
                i_term = 0.0
            
            # Combine terms
            pid = p_term + i_term + d_term
            self.last_pid = pid
            
            if CONFIG['debug'].get('pid_debug', False):
                logging.debug(
                    f"PID calculation:\n"
                    f"  Mode: {'continuous' if any(k['pressed'] for k in key_states.values()) else 'transitional'}\n"
                    f"  Target: {self.target_position:.1f}, Current: {self.command:.1f}\n"
                    f"  Raw Error: {error:.2f}, Working Error: {working_error:.2f}\n"
                    f"  Movement Velocity: {current_velocity:.2f}\n"
                    f"  P: {p_term:.4f}, I: {i_term:.4f}, D: {d_term:.4f}\n"
                    f"  Total: {pid:.4f}, dt: {dt:.4f}"
                )
            
            if self.servo:
                self.previous_command = self.command
                new_command = self.command + pid
                
                # Apply output deadband if enabled
                if self.deadband_enabled:
                    command_delta = new_command - self.command
                    if abs(command_delta) <= self.output_deadband:
                        new_command = self.command
                
                # Apply limits and convert to integer
                self.command = min(max(new_command, float(PIXY_RCS_MINIMUM_POSITION)), float(PIXY_RCS_MAXIMUM_POSITION))
        
        self.previous_error = float(error)
        self.previous_time = current_time
        return int(round(self.command))

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

def validate_performance(stats):
    """
    Validate basic performance metrics.
    Returns a dict with test results and recommendations.
    """
    results = {
        'fps': {
            'status': 'OK',
            'value': 0,
            'target': 30,
            'message': ''
        },
        'timing': {
            'status': 'OK',
            'value': 0,
            'target': 16,  # Target 16ms (60Hz)
            'message': ''
        },
        'movement': {
            'status': 'OK',
            'value': 0,
            'message': ''
        }
    }
    
    # FPS Check
    if 'fps' in stats:
        fps = stats['fps']
        results['fps']['value'] = fps
        if fps < 20:  # Below 20 FPS is problematic
            results['fps']['status'] = 'WARNING'
            results['fps']['message'] = f"Low FPS ({fps:.1f}). Target is 30+ FPS."
    
    # Timing Check
    if 'timing' in stats:
        avg_dt = stats['timing'].get('avg_dt', 0) * 1000  # Convert to ms
        results['timing']['value'] = avg_dt
        if avg_dt > 20:  # More than 20ms average delay
            results['timing']['status'] = 'WARNING'
            results['timing']['message'] = f"High average delay ({avg_dt:.1f}ms). Target is <16ms."
    
    # Movement Check
    if 'movements' in stats:
        pan_moves = stats['movements']['pan']['transitions']
        tilt_moves = stats['movements']['tilt']['transitions']
        total_moves = pan_moves + tilt_moves
        results['movement']['value'] = total_moves
        
        if total_moves > 0:
            # Calculate successful moves ratio
            success_ratio = 1.0  # Assume all moves successful unless we find issues
            
            # Check for timing issues during movement
            if 'timing' in stats and stats['timing'].get('max_dt', 0) > 0.1:  # >100ms spikes
                success_ratio *= 0.8
            
            # Check for minimum speed triggers (indicates potential stutter)
            min_speed_triggers = (stats['movements']['pan']['min_speed_triggers'] + 
                                stats['movements']['tilt']['min_speed_triggers'])
            if min_speed_triggers > total_moves * 0.2:  # More than 20% of moves triggered min speed
                success_ratio *= 0.9
            
            if success_ratio < 0.9:
                results['movement']['status'] = 'WARNING'
                results['movement']['message'] = "Movement quality issues detected. Check for stuttering or delays."
    
    return results

def generate_summary(stats):
    """Generate a simple, readable summary of servo performance"""
    results = validate_performance(stats)
    
    summary = [
        "=== SERVO PERFORMANCE SUMMARY ===",
        "",
        "Performance Metrics:",
        f"1. Frame Rate: {results['fps']['value']:.1f} FPS",
        f"   Target: {results['fps']['target']} FPS",
        f"   Status: {results['fps']['status']}",
        f"   {results['fps']['message']}" if results['fps']['message'] else "",
        "",
        f"2. Response Time: {results['timing']['value']:.1f}ms average",
        f"   Target: {results['timing']['target']}ms",
        f"   Status: {results['timing']['status']}",
        f"   {results['timing']['message']}" if results['timing']['message'] else "",
        "",
        f"3. Movement Quality",
        f"   Total Movements: {results['movement']['value']}",
        f"   Status: {results['movement']['status']}",
        f"   {results['movement']['message']}" if results['movement']['message'] else "",
        "",
        "Configuration:",
        f"  Pan Gain: {CONFIG['servo']['pan_gain']}",
        f"  Tilt Gain: {CONFIG['servo']['tilt_gain']}",
        f"  Max Speed: {CONFIG['servo']['velocity']['max_speed']}",
        f"  Current Preset: {CONFIG['movement_presets']['current']}",
        "",
        "=== END SUMMARY ===\n"
    ]
    
    return '\n'.join(line for line in summary if line is not None)

def analyze_log_file(log_file_path):
    """Analyze log file and generate diagnostic summary"""
    try:
        with open(log_file_path, 'r') as f:
            lines = f.readlines()
        
        # Initialize statistics
        stats = {
            'fps': 0,
            'timing': {
                'max_dt': 0.0,
                'min_dt': float('inf'),
                'avg_dt': 0.0,
                'samples': 0
            },
            'movements': {
                'pan': {'transitions': 0, 'min_speed_triggers': 0},
                'tilt': {'transitions': 0, 'min_speed_triggers': 0}
            }
        }
        
        # Process log lines
        fps_values = []
        for line in lines:
            # Extract FPS
            if "FPS:" in line:
                try:
                    fps = float(line.split("FPS:")[-1].split(',')[0])
                    fps_values.append(fps)
                except:
                    pass
            
            # Extract timing
            elif "dt:" in line:
                try:
                    dt = float(line.split("dt:")[-1].split()[0])
                    stats['timing']['max_dt'] = max(stats['timing']['max_dt'], dt)
                    stats['timing']['min_dt'] = min(stats['timing']['min_dt'], dt)
                    stats['timing']['avg_dt'] += dt
                    stats['timing']['samples'] += 1
                except:
                    pass
            
            # Extract movement data
            elif "Movement state transition" in line:
                if "pan" in line.lower():
                    stats['movements']['pan']['transitions'] += 1
                elif "tilt" in line.lower():
                    stats['movements']['tilt']['transitions'] += 1
            
            # Extract minimum speed triggers
            elif "min_speed_applied: True" in line:
                if "pan" in line.lower():
                    stats['movements']['pan']['min_speed_triggers'] += 1
                elif "tilt" in line.lower():
                    stats['movements']['tilt']['min_speed_triggers'] += 1
        
        # Calculate average FPS
        if fps_values:
            # Use the last 10 seconds of FPS values for current performance
            recent_fps = fps_values[-300:] if len(fps_values) > 300 else fps_values
            stats['fps'] = sum(recent_fps) / len(recent_fps)
        
        # Calculate average timing
        if stats['timing']['samples'] > 0:
            stats['timing']['avg_dt'] /= stats['timing']['samples']
        
        return generate_summary(stats)
        
    except Exception as e:
        return f"Error analyzing log file: {str(e)}\n"

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
    root.setLevel(logging.DEBUG)
    
    # File handler - gets everything
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    root.addHandler(file_handler)
    
    # Console handler - only gets important messages
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.WARNING)
    console_handler.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))
    
    # Add filter to capture Pixy2 debug messages
    class Pixy2DebugFilter(logging.Filter):
        def filter(self, record):
            return "Debug:" not in record.msg and "Pixy2" not in record.msg

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
    
    # Write initial summary placeholder
    with open(log_file, 'w') as f:
        f.write("### LOG SUMMARY WILL BE INSERTED HERE ###\n\n")
    
    return log_file, TerminalHandler()

def finalize_logging(log_file):
    """Insert summary at the start of the log file"""
    if os.path.exists(log_file):
        # Generate summary
        summary = analyze_log_file(log_file)
        
        # Read existing log
        with open(log_file, 'r') as f:
            log_content = f.read()
        
        # Write summary and log content
        with open(log_file, 'w') as f:
            f.write(summary + "\n" + log_content)

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

def calculate_velocity(current_velocity, target_velocity, dt):
    """
    Calculate new velocity with improved smoothing logic and better sustained movement.
    """
    global VELOCITY_SCALE
    velocity_config = CONFIG['servo']['velocity']
    max_accel = velocity_config['acceleration'] * VELOCITY_SCALE
    min_speed = velocity_config['min_speed']
    
    # Scale target velocity
    scaled_target = target_velocity * VELOCITY_SCALE
    
    # Calculate maximum velocity change for this time step
    max_delta = max_accel * dt
    
    # Calculate desired velocity change
    desired_delta = scaled_target - current_velocity
    
    # Detect movement phase with more aggressive sustained threshold
    is_accelerating = abs(current_velocity) < abs(scaled_target) * 0.95  # Increased from 0.8
    is_sustained = abs(current_velocity) >= abs(scaled_target) * 0.95
    
    # Special handling for sustained movement to prevent slowdown
    if is_sustained:
        # During sustained movement, actively maintain target velocity
        maintain_factor = velocity_config['phases']['sustained']['velocity_maintain_factor']
        current_ratio = abs(current_velocity) / abs(scaled_target)
        
        if current_ratio < maintain_factor:
            # If velocity has dropped below maintain factor, boost it back up
            direction = 1 if scaled_target > 0 else -1
            new_velocity = direction * abs(scaled_target)  # Snap directly to target
            
            if CONFIG['debug'].get('pid_debug', False):
                logging.debug(
                    f"Velocity maintenance activated:\n"
                    f"  Ratio: {current_ratio:.3f} < {maintain_factor:.3f}\n"
                    f"  Boosting: {current_velocity:.2f} → {new_velocity:.2f}"
                )
            return new_velocity
        
        # For sustained movement, use minimal smoothing and high acceleration
        smoothing_factor = 0.1  # Reduced from 0.2
        accel_scale = 1.0  # Increased from 0.9
    elif is_accelerating:
        # Initial acceleration phase
        smoothing_factor = velocity_config['phases']['accelerating']['smoothing']
        accel_scale = min(1.0, abs(desired_delta) / abs(scaled_target) if scaled_target != 0 else 0)
        accel_scale = accel_scale * (3 - 2 * accel_scale) * smoothing_factor
    else:
        # Transitional movement
        smoothing_factor = 0.5
        accel_scale = 0.7
    
    # Calculate acceleration with reduced time-based decay for sustained movement
    if is_sustained:
        smoothing_decay = 1.0  # No decay during sustained movement
    else:
        smoothing_decay = np.exp(-dt / 0.1)  # Normal decay for other phases
    
    effective_smoothing = smoothing_factor * smoothing_decay
    
    # Apply smoothing to acceleration with higher minimum during sustained movement
    min_accel_factor = 0.6 if is_sustained else 0.4  # Increased minimum acceleration factor
    smoothed_accel = max_delta * (min_accel_factor + (1.0 - min_accel_factor) * accel_scale)
    
    # Limit velocity change
    actual_delta = max(-smoothed_accel, min(smoothed_accel, desired_delta))
    new_velocity = current_velocity + actual_delta
    
    # More aggressive snapping to target velocity during sustained movement
    if is_sustained:
        snap_threshold = 0.02 if is_sustained else 0.05  # Tighter threshold for sustained
        if abs(new_velocity - scaled_target) < abs(scaled_target) * snap_threshold:
            new_velocity = scaled_target
    
    # Apply minimum speed threshold
    if abs(new_velocity) < min_speed:
        if abs(scaled_target) > 0:
            new_velocity = min_speed * (1 if scaled_target > 0 else -1)
        else:
            new_velocity = 0.0
    
    # Enhanced debug logging
    if CONFIG['debug'].get('pid_debug', False):
        logging.debug(
            f"Velocity calculation:\n"
            f"  Phase: {'sustained' if is_sustained else 'accelerating' if is_accelerating else 'other'}\n"
            f"  Current: {current_velocity:.2f} → Target: {scaled_target:.2f}\n"
            f"  Smoothing: factor={smoothing_factor:.2f}, decay={smoothing_decay:.2f}\n"
            f"  Acceleration: scale={accel_scale:.2f}, smoothed={smoothed_accel:.2f}\n"
            f"  Delta: desired={desired_delta:.2f}, actual={actual_delta:.2f}\n"
            f"  Result: {new_velocity:.2f}"
        )
    
    return new_velocity

def update_movement_state(axis_state, target_velocity, dt):
    """
    Update movement state with improved sustained movement handling.
    """
    # Store previous state for debugging
    prev_velocity = axis_state['velocity']
    prev_position = axis_state['last_position']
    prev_time = axis_state.get('last_update', time.time())
    prev_phase = axis_state['movement_phase']
    
    # Calculate actual time delta
    current_time = time.time()
    actual_dt = current_time - prev_time
    
    # More aggressive phase detection for sustained movement
    if target_velocity == 0:
        new_phase = 'decelerating' if abs(prev_velocity) > 0 else 'idle'
    elif abs(prev_velocity) >= abs(target_velocity) * 0.95:  # Increased from 0.8
        new_phase = 'sustained'
    else:
        new_phase = 'accelerating'
    
    # Handle phase transitions with priority for sustained movement
    if new_phase != prev_phase:
        phase_config = CONFIG['servo']['velocity']['phases'].get(new_phase, {})
        min_duration = phase_config.get('min_duration', 0)
        phase_duration = current_time - axis_state['phase_start_time']
        
        # More permissive transition to sustained phase
        should_transition = (
            new_phase == 'idle' or
            prev_phase == 'idle' or
            new_phase == 'sustained' or  # Always allow transition to sustained
            phase_duration >= min_duration
        )
        
        if should_transition:
            axis_state['movement_phase'] = new_phase
            axis_state['phase_start_time'] = current_time
            if new_phase == 'accelerating':
                axis_state['initial_velocity'] = prev_velocity
            
            if CONFIG['debug'].get('pid_debug', False):
                logging.debug(
                    f"Movement phase transition:\n"
                    f"  {prev_phase} → {new_phase}\n"
                    f"  Previous phase duration: {phase_duration:.3f}s\n"
                    f"  Velocity: {prev_velocity:.2f}\n"
                    f"  Target: {target_velocity:.2f}"
                )
    
    # Calculate phase duration
    phase_duration = current_time - axis_state['phase_start_time']
    
    # Update velocity with phase awareness
    if target_velocity == 0 and new_phase == 'idle':
        axis_state['velocity'] = 0.0
        new_velocity = 0.0
    else:
        # Calculate new velocity
        new_velocity = calculate_velocity(
            prev_velocity,
            target_velocity,
            dt
        )
        axis_state['velocity'] = new_velocity
    
    # Calculate position change
    position_delta = axis_state['velocity'] * dt
    new_position = axis_state['last_position'] + position_delta
    
    # Update state
    axis_state['last_position'] = new_position
    axis_state['last_update'] = current_time
    
    if CONFIG['debug'].get('pid_debug', False):
        if abs(new_velocity - prev_velocity) > 0.1 or abs(position_delta) > 0.1:
            logging.debug(
                f"Movement state update:\n"
                f"  Phase: {axis_state['movement_phase']} (duration: {phase_duration:.3f}s)\n"
                f"  Position: {prev_position:.1f} → {new_position:.1f} (Δ{position_delta:+.1f})\n"
                f"  Velocity: {prev_velocity:.2f} → {new_velocity:.2f}\n"
                f"  Target velocity: {target_velocity:.2f}\n"
                f"  dt: {dt:.4f}"
            )
    
    return new_position

def get_transition_type(prev_vel, target_vel):
    """Helper to classify movement transitions"""
    if prev_vel == 0 and target_vel != 0:
        return "Starting movement"
    elif prev_vel != 0 and target_vel == 0:
        return "Stopping movement"
    elif prev_vel * target_vel < 0:
        return "Direction change"
    else:
        return "Continuous movement"

def process_held_keys(pan_controller, tilt_controller):
    """
    Process currently held keys and update movement accordingly.
    
    Implements the main movement logic:
    - Velocity-based movement
    - Time-based updates
    - Controller updates
    - State management
    
    Args:
        pan_controller (PID_Controller): Controller for pan axis
        tilt_controller (PID_Controller): Controller for tilt axis
        
    Returns:
        tuple: (target_pan, target_tilt) positions
    """
    global key_states, movement_state
    current_time = time.time()
    
    # Calculate time delta
    dt = current_time - movement_state['pan']['last_update']
    if dt <= 0:  # Avoid division by zero
        dt = CONFIG['servo']['update_interval']
    
    # Calculate target velocities
    pan_velocity = 0.0
    tilt_velocity = 0.0
    max_speed = CONFIG['servo']['velocity']['max_speed']
    
    # Process pan movement
    if key_states['a']['pressed']:
        pan_velocity = -max_speed
    elif key_states['d']['pressed']:
        pan_velocity = max_speed
    else:
        # No pan keys pressed, ensure velocity is zeroed
        movement_state['pan']['velocity'] = 0.0
        
    # Process tilt movement
    if key_states['w']['pressed']:
        tilt_velocity = -max_speed
    elif key_states['s']['pressed']:
        tilt_velocity = max_speed
    else:
        # No tilt keys pressed, ensure velocity is zeroed
        movement_state['tilt']['velocity'] = 0.0
    
    # Update movement state and calculate new positions
    new_pan = update_movement_state(movement_state['pan'], pan_velocity, dt)
    new_tilt = update_movement_state(movement_state['tilt'], tilt_velocity, dt)
    
    # Apply limits
    new_pan = max(PIXY_RCS_MINIMUM_POSITION, min(PIXY_RCS_MAXIMUM_POSITION, new_pan))
    new_tilt = max(PIXY_RCS_MINIMUM_POSITION, min(PIXY_RCS_MAXIMUM_POSITION, new_tilt))
    
    # Update controllers
    pan_controller.set_target(new_pan)
    tilt_controller.set_target(new_tilt)
    
    if CONFIG['debug'].get('pid_debug', False) and (pan_velocity != 0 or tilt_velocity != 0):
        logging.debug(
            f"Velocity movement:\n"
            f"  Pan: velocity={pan_velocity:.2f}, keys=[a:{key_states['a']['pressed']}, d:{key_states['d']['pressed']}]\n"
            f"  Tilt: velocity={tilt_velocity:.2f}, keys=[w:{key_states['w']['pressed']}, s:{key_states['s']['pressed']}]"
        )
    
    return pan_controller.target_position, tilt_controller.target_position

def handle_key_event(key, pan_controller, tilt_controller, terminal=None, is_new_press=False):
    """
    Handle key press events and update movement state.
    """
    global key_states, movement_state, VELOCITY_SCALE
    current_time = time.time()
    
    # Handle preset switching
    preset_keys = {
        ord('1'): 'smooth',
        ord('2'): 'responsive',
        ord('3'): 'precise',
        ord('4'): 'balanced'
    }
    
    if key in preset_keys:
        preset_name = preset_keys[key]
        if apply_movement_preset(preset_name, pan_controller, tilt_controller):
            if terminal:
                terminal.show_message(f"Switched to {preset_name} preset")
        return pan_controller.target_position, tilt_controller.target_position
    
    # Handle velocity scale adjustment
    if key == ord('['):  # Decrease scale
        VELOCITY_SCALE = max(VELOCITY_SCALE_MIN, VELOCITY_SCALE - VELOCITY_SCALE_STEP)
        if terminal:
            terminal.show_message(f"Velocity scale: {VELOCITY_SCALE:.2f}x")
        return pan_controller.target_position, tilt_controller.target_position
    elif key == ord(']'):  # Increase scale
        VELOCITY_SCALE = min(VELOCITY_SCALE_MAX, VELOCITY_SCALE + VELOCITY_SCALE_STEP)
        if terminal:
            terminal.show_message(f"Velocity scale: {VELOCITY_SCALE:.2f}x")
        return pan_controller.target_position, tilt_controller.target_position
    
    # Update key states and set target velocities
    max_speed = CONFIG['servo']['velocity']['max_speed']
    debug_enabled = CONFIG['debug'].get('pid_debug', False)
    
    def update_key_state(key_name, pressed, target_vel, opposing_key=None, is_new_press=False):
        """Helper to update key state with debug logging"""
        prev_state = key_states[key_name].copy()  # Store previous state for comparison
        
        if pressed:
            if is_new_press:  # Only update press_start on initial press
                key_states[key_name]['press_start'] = current_time
            key_states[key_name]['target_velocity'] = target_vel
            key_states[key_name]['pressed'] = True
            if debug_enabled:
                hold_duration = current_time - key_states[key_name]['press_start']
                logging.debug(
                    f"Key '{key_name}' {'initial press' if is_new_press else 'held'}:\n"
                    f"  Time: {current_time:.3f}\n"
                    f"  Target velocity: {target_vel:.1f}\n"
                    f"  Previous state: {prev_state}"
                )
        else:
            key_states[key_name]['pressed'] = False
            key_states[key_name]['target_velocity'] = 0.0
            
        # Handle opposing key
        if opposing_key and pressed:
            if key_states[opposing_key]['pressed']:
                if debug_enabled:
                    logging.debug(f"Releasing opposing key '{opposing_key}'")
            key_states[opposing_key]['pressed'] = False
            key_states[opposing_key]['target_velocity'] = 0.0
            
        # Log state changes
        if debug_enabled and key_states[key_name] != prev_state:
            logging.debug(
                f"Key '{key_name}' state updated:\n"
                f"  Previous: {prev_state}\n"
                f"  Current: {key_states[key_name]}"
            )
    
    if key in [ord('a'), ord('A')]:
        update_key_state('a', True, -max_speed, opposing_key='d', is_new_press=is_new_press)
    elif key in [ord('d'), ord('D')]:
        update_key_state('d', True, max_speed, opposing_key='a', is_new_press=is_new_press)
    elif key in [ord('w'), ord('W')]:
        update_key_state('w', True, -max_speed, opposing_key='s', is_new_press=is_new_press)
    elif key in [ord('s'), ord('S')]:
        update_key_state('s', True, max_speed, opposing_key='w', is_new_press=is_new_press)
    elif key in [ord('c'), ord('C')]:  # center
        if debug_enabled:
            logging.debug("Center command received - resetting all states")
        # Reset all key states
        for k in key_states:
            key_states[k]['pressed'] = False
            key_states[k]['press_start'] = 0
            key_states[k]['velocity'] = 0.0
            key_states[k]['target_velocity'] = 0.0
        
        # Reset movement state
        movement_state['pan']['velocity'] = 0.0
        movement_state['pan']['last_position'] = PIXY_RCS_CENTER_POSITION
        movement_state['pan']['last_update'] = current_time
        
        movement_state['tilt']['velocity'] = 0.0
        movement_state['tilt']['last_position'] = PIXY_RCS_CENTER_POSITION
        movement_state['tilt']['last_update'] = current_time
        
        # Reset PID controllers
        pan_controller.reset()
        tilt_controller.reset()
        
        # Set center targets
        pan_controller.set_target(PIXY_RCS_CENTER_POSITION)
        tilt_controller.set_target(PIXY_RCS_CENTER_POSITION)
        
        if debug_enabled:
            logging.debug("All states reset, servos centering")
    
    if debug_enabled:
        active_keys = [k for k in key_states if key_states[k]['pressed']]
        if active_keys:
            logging.debug(
                f"Active keys summary:\n"
                f"  Keys: {active_keys}\n"
                f"  Pan velocities: [a: {key_states['a']['target_velocity']:.1f}, d: {key_states['d']['target_velocity']:.1f}]\n"
                f"  Tilt velocities: [w: {key_states['w']['target_velocity']:.1f}, s: {key_states['s']['target_velocity']:.1f}]\n"
                f"  Movement state: {movement_state}"
            )
    
    # Process movement immediately to avoid delay
    return process_held_keys(pan_controller, tilt_controller)

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

        # Initialize movement state
        movement_state['pan']['last_position'] = current_pan
        movement_state['pan']['last_update'] = time.time()
        movement_state['tilt']['last_position'] = current_tilt
        movement_state['tilt']['last_update'] = time.time()

        # Set initial positions
        print("Setting initial servo positions...")
        try:
            pixy.set_servos(int(current_pan), int(current_tilt))
            print("Servos initialized successfully!")
        except Exception as e:
            print("Error: Failed to set initial servo positions:", str(e))
            sys.exit(1)

        print("\nControls:")
        print("Movement:")
        print("  a/d : Pan left/right")
        print("  w/s : Tilt up/down")
        print("  c   : Center servos")
        print("\nPresets:")
        print("  1   : Smooth (slower, precise)")
        print("  2   : Responsive (quick)")
        print("  3   : Precise (balanced speed)")
        print("  4   : Balanced (default)")
        print("\nSpeed Control:")
        print("  [   : Decrease velocity scale")
        print("  ]   : Increase velocity scale")
        print("\nVideo:")
        print("  r   : Toggle recording")
        print("  p   : Toggle preview")
        print("  q   : Quit")

        # Initialize terminal for non-blocking input
        fd, old_settings = init_terminal()

        last_update = time.time()
        frames_processed = 0
        start_time = time.time()
        recording = True
        
        # Track which keys were pressed in the previous iteration
        previously_pressed_keys = set()
        
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
                    # Only treat as initial press if key wasn't pressed before
                    is_new_press = key not in previously_pressed_keys
                    previously_pressed_keys.add(key)
                    # Handle pan/tilt commands
                    target_pan, target_tilt = handle_key_event(key, pan_controller, tilt_controller, terminal, is_new_press)

            # Check for key releases by comparing with previous state
            current_keys = set()
            if is_data():  # Keys are being pressed
                char = getch_non_blocking()
                while char:  # Get all currently pressed keys
                    current_keys.add(ord(char.lower()))
                    char = getch_non_blocking()
            
            # Handle key releases
            released_keys = previously_pressed_keys - current_keys
            for key in released_keys:
                if chr(key) in key_states:
                    # Fully reset the key state
                    key_states[chr(key)] = {
                        'pressed': False,
                        'press_start': 0,
                        'velocity': 0.0,
                        'target_velocity': 0.0
                    }
                    if CONFIG['debug'].get('pid_debug', False):
                        hold_duration = time.time() - key_states[chr(key)]['press_start']
                        logging.debug(f"Key '{chr(key)}' released after {hold_duration:.3f}s - State fully reset")
            
            # Clear previously_pressed_keys if no keys are currently pressed
            if not current_keys:
                previously_pressed_keys.clear()
                # Also ensure all key states are reset
                for k in key_states:
                    key_states[k] = {
                        'pressed': False,
                        'press_start': 0,
                        'velocity': 0.0,
                        'target_velocity': 0.0
                    }
            else:
                previously_pressed_keys = current_keys

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
                            target_pan, target_tilt = handle_key_event(key, pan_controller, tilt_controller, terminal)
                frames_processed += 1

            # Update movement at fixed interval
            current_time = time.time()
            if current_time - last_update >= UPDATE_INTERVAL:
                # Process any held keys first
                if any(k['pressed'] for k in key_states.values()):
                    target_pan, target_tilt = process_held_keys(pan_controller, tilt_controller)
                
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
                    status += f"Pan: {int(current_pan):4d} Tilt: {int(current_tilt):4d} Speed: {VELOCITY_SCALE:.1f}x"
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
        import traceback
        error_details = traceback.format_exc()
        logging.error(f"Error occurred: {str(e)}\nTraceback:\n{error_details}")
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

        # Cleanup and finalize log
        finalize_logging(log_file)

if __name__ == "__main__":
    main() 