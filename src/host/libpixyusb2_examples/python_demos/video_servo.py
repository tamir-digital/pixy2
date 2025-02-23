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
PAN_STEP = 25
TILT_STEP = 25

# Default configuration
DEFAULT_CONFIG = {
    "video": {
        "output_dir": "output",
        "codec": "XVID",
        "fps": 30,
        "show_preview": True
    },
    "servo": {
        "pan_step": 25,
        "tilt_step": 25,
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
            "debug": False,        # Separate debug flag for motion profiling
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
        "log_level": "INFO",
        "show_fps": True,
        "suppress_pixy_debug": True,
        "pid_debug": True        # Enable PID debugging
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
        'last_position': PIXY_RCS_CENTER_POSITION
    },
    'tilt': {
        'velocity': 0.0,
        'last_update': 0,
        'last_position': PIXY_RCS_CENTER_POSITION
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
        
        This is the main control loop that:
        1. Handles motion profile integration
        2. Calculates PID terms
        3. Applies anti-windup protection
        4. Manages deadband
        5. Produces final output command
        
        Args:
            error (float): Current position error
            
        Returns:
            int: The calculated command value for the servo
            
        The update process:
        1. Check and update motion profile if active
        2. Apply deadband to error if enabled
        3. Calculate each PID term:
           - Proportional: direct error scaling
           - Integral: time-based integration with decay
           - Derivative: based on movement state velocity
        4. Apply anti-windup protection
        5. Combine terms and apply limits
        6. Convert to integer command
        """
        current_time = time.time()
        
        # Check if we're following a motion profile
        if self.motion.is_profile_active():
            # Get next position from profile
            profile_pos = self.motion.update()
            if profile_pos is not None:
                # Use profile position as intermediate target
                error = profile_pos - self.command
                if self.debug:
                    logging.debug(f"Profile active - target: {profile_pos:.1f}, error: {error:.1f}")
            else:
                # Profile completed, use final target
                error = self.target_position - self.command
                if self.debug:
                    logging.debug(f"Profile complete - target: {self.target_position:.1f}, error: {error:.1f}")
        
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
            else:
                current_velocity = 0.0
            
            # Calculate PID terms with float math
            p_term = working_error * self.proportion_gain
            
            # Enhanced integral term with time-based integration and decay
            if self.integral_enabled and self.integral_gain != 0:  # Extra safety check
                # Store previous integral for debug
                prev_integral = self.integral_value
                
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
                            if self.debug:
                                logging.debug(f"Anti-windup active - scale: {windup_scale}, "
                                            f"command_delta: {command_delta:.2f}, last_pid: {self.last_pid:.2f}")
                
                # Apply limits
                self.integral_value = min(max(self.integral_value, self.integral_min), self.integral_max)
                
                i_term = self.integral_value * self.integral_gain
                
                if self.debug:
                    integral_change = self.integral_value - prev_integral
                    logging.debug(f"Integral update - "
                                f"prev: {prev_integral:.2f}, new: {self.integral_value:.2f}, "
                                f"change: {integral_change:+.2f}, decay: {self.integral_decay:.3f}, "
                                f"error: {integral_error:.2f}")
            else:
                i_term = 0.0
            
            # Improved derivative term using movement state velocity
            # Note: current_velocity is already in the correct direction
            d_term = -current_velocity * (self.derivative_gain * 0.5)  # Reduced gain
            
            # Combine terms (no feedforward)
            pid = p_term + i_term + d_term
            self.last_pid = pid  # Store for anti-windup
            
            if self.debug:
                logging.debug(
                    f"PID calculation - "
                    f"Target: {self.target_position:.1f}, Current: {self.command:.1f}, "
                    f"Raw Error: {error:.2f}, Working Error: {working_error:.2f}, "
                    f"Movement Velocity: {current_velocity:.2f}, "
                    f"P: {p_term:.4f}, I: {i_term:.4f}, D: {d_term:.4f}, "
                    f"Total: {pid:.4f}, dt: {dt:.4f}"
                )
            
            if self.servo:
                self.previous_command = self.command
                # Calculate new command
                new_command = self.command + pid
                
                # Apply output deadband if enabled
                if self.deadband_enabled:
                    command_delta = new_command - self.command
                    if abs(command_delta) <= self.output_deadband:
                        if self.debug:
                            logging.debug(f"Output deadband active - delta: {command_delta:.2f} ≤ {self.output_deadband:.2f}")
                        new_command = self.command
                
                # Apply limits and convert to integer
                prev_command = self.command
                self.command = min(max(new_command, float(PIXY_RCS_MINIMUM_POSITION)), float(PIXY_RCS_MAXIMUM_POSITION))
                
                if self.debug and self.command != new_command:
                    logging.debug(f"Command limited - raw: {new_command:.1f}, limited: {self.command:.1f}")
        
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

def calculate_velocity(current_velocity, target_velocity, dt):
    """
    Calculate new velocity based on acceleration limits and smoothing.
    
    Implements sophisticated velocity control with:
    - Acceleration limiting
    - Smoothing for natural motion
    - Minimum speed thresholds
    - Scaled acceleration curves
    
    Args:
        current_velocity (float): Current velocity
        target_velocity (float): Desired target velocity
        dt (float): Time delta since last update
        
    Returns:
        float: New calculated velocity respecting limits and smoothing
        
    The calculation:
    1. Applies acceleration limits
    2. Uses smoothing for natural feel
    3. Handles minimum speed threshold
    4. Provides smooth deceleration
    """
    velocity_config = CONFIG['servo']['velocity']
    max_accel = velocity_config['acceleration']
    min_speed = velocity_config['min_speed']
    smoothing_config = velocity_config.get('smoothing', {})
    
    # Calculate maximum velocity change for this time step
    max_delta = max_accel * dt
    
    # Calculate desired velocity change
    desired_delta = target_velocity - current_velocity
    
    # Apply smooth acceleration curve
    if abs(desired_delta) > 0:
        # Scale acceleration based on how close we are to target velocity
        accel_scale = min(1.0, abs(desired_delta) / abs(target_velocity) if target_velocity != 0 else 0)
        
        # Apply enhanced smoothing if enabled
        if smoothing_config.get('enabled', True):
            smoothing_factor = float(smoothing_config.get('factor', 0.8))
            # Apply sigmoid-like smoothing to acceleration
            accel_scale = accel_scale * (3 - 2 * accel_scale) * smoothing_factor
        
        # Apply smoothing to acceleration
        smoothed_accel = max_delta * (0.2 + 0.8 * accel_scale)  # Never less than 20% acceleration
        # Limit velocity change by smoothed acceleration
        actual_delta = max(-smoothed_accel, min(smoothed_accel, desired_delta))
    else:
        actual_delta = 0
    
    # Calculate new velocity
    new_velocity = current_velocity + actual_delta
    
    # Apply minimum speed threshold to avoid tiny movements
    if abs(new_velocity) < min_speed:
        if abs(target_velocity) > 0:
            # If we have a target, maintain minimum speed
            new_velocity = min_speed * (1 if target_velocity > 0 else -1)
        else:
            # If no target, come to a stop with smooth deceleration
            decel_factor = 0.8  # Smooth deceleration
            new_velocity = current_velocity * decel_factor
            # Stop completely if very slow
            if abs(new_velocity) < min_speed * 0.5:
                new_velocity = 0.0
    
    if CONFIG['debug'].get('pid_debug', False):
        logging.debug(
            f"Velocity calculation - "
            f"current: {current_velocity:.2f}, "
            f"target: {target_velocity:.2f}, "
            f"dt: {dt:.4f}, "
            f"max_delta: {max_delta:.2f}, "
            f"actual_delta: {actual_delta:.2f}, "
            f"new: {new_velocity:.2f}"
        )
    
    return new_velocity

def update_movement_state(axis_state, target_velocity, dt):
    """
    Update movement state for an axis (pan or tilt).
    
    Manages the complete state of an axis including:
    - Velocity calculations
    - Position tracking
    - Timing management
    - Debug logging
    
    Args:
        axis_state (dict): Current state of the axis
        target_velocity (float): Desired velocity
        dt (float): Time delta since last update
        
    Returns:
        float: New position for the axis
        
    The update process:
    1. Stores previous state
    2. Updates velocity
    3. Calculates position change
    4. Updates timing
    5. Logs debug information
    """
    # Store previous state for debugging
    prev_velocity = axis_state['velocity']
    prev_position = axis_state['last_position']
    
    # Update velocity
    axis_state['velocity'] = calculate_velocity(
        axis_state['velocity'],
        target_velocity,
        dt
    )
    
    # Calculate position change
    position_delta = axis_state['velocity'] * dt
    new_position = axis_state['last_position'] + position_delta
    
    # Update state
    axis_state['last_position'] = new_position
    axis_state['last_update'] = time.time()
    
    if CONFIG['debug'].get('pid_debug', False):
        velocity_change = axis_state['velocity'] - prev_velocity
        position_change = new_position - prev_position
        logging.debug(
            f"Movement state update - "
            f"Position: {prev_position:.1f} → {new_position:.1f} (Δ{position_change:+.1f}), "
            f"Velocity: {prev_velocity:.1f} → {axis_state['velocity']:.1f} (Δ{velocity_change:+.1f}), "
            f"dt: {dt:.4f}"
        )
    
    return new_position

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
        
    The process:
    1. Calculates time delta
    2. Determines target velocities
    3. Updates movement state
    4. Updates controller targets
    5. Applies position limits
    """
    global key_states, movement_state
    current_time = time.time()
    
    if not CONFIG['servo']['velocity']['enabled']:
        # Use existing step-based movement if velocity mode is disabled
        return process_held_keys_legacy(pan_controller, tilt_controller)
    
    # Calculate time delta
    dt = current_time - movement_state['pan']['last_update']
    if dt <= 0:  # Avoid division by zero
        dt = CONFIG['servo']['update_interval']
    
    # Calculate target velocities
    pan_velocity = 0.0
    tilt_velocity = 0.0
    max_speed = CONFIG['servo']['velocity']['max_speed']
    
    if key_states['a']['pressed']:
        pan_velocity = -max_speed
    elif key_states['d']['pressed']:
        pan_velocity = max_speed
        
    if key_states['w']['pressed']:
        tilt_velocity = -max_speed
    elif key_states['s']['pressed']:
        tilt_velocity = max_speed
    
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
        logging.debug(f"Velocity movement - pan: {pan_velocity:.2f}, tilt: {tilt_velocity:.2f}")
    
    return pan_controller.target_position, tilt_controller.target_position

def process_held_keys_legacy(pan_controller, tilt_controller):
    """Legacy step-based movement (kept as fallback)"""
    global key_states
    current_time = time.time()
    
    # Get step sizes from config
    pan_step = CONFIG['servo']['pan_step']
    tilt_step = CONFIG['servo']['tilt_step']
    
    # Calculate hold duration and scale step size
    def get_scaled_step(key, base_step):
        if not key_states[key]['pressed']:
            return 0
        hold_duration = current_time - key_states[key]['press_start']
        # Start with smaller steps and ramp up
        scale = min(1.0, hold_duration * 5.0)  # Reach full speed in 0.2 seconds
        return base_step * scale
    
    # Calculate pan movement
    pan_step_size = 0
    if key_states['a']['pressed']:
        pan_step_size = -get_scaled_step('a', pan_step)
    elif key_states['d']['pressed']:
        pan_step_size = get_scaled_step('d', pan_step)
        
    # Calculate tilt movement
    tilt_step_size = 0
    if key_states['w']['pressed']:
        tilt_step_size = -get_scaled_step('w', tilt_step)
    elif key_states['s']['pressed']:
        tilt_step_size = get_scaled_step('s', tilt_step)
    
    # Apply movements if any keys are held
    if pan_step_size != 0:
        new_pan = max(PIXY_RCS_MINIMUM_POSITION, 
                     min(PIXY_RCS_MAXIMUM_POSITION, 
                         pan_controller.command + pan_step_size))
        pan_controller.set_target(new_pan)
        
    if tilt_step_size != 0:
        new_tilt = max(PIXY_RCS_MINIMUM_POSITION,
                      min(PIXY_RCS_MAXIMUM_POSITION,
                          tilt_controller.command + tilt_step_size))
        tilt_controller.set_target(new_tilt)
    
    if CONFIG['debug'].get('pid_debug', False) and (pan_step_size != 0 or tilt_step_size != 0):
        logging.debug(f"Legacy movement - pan_step: {pan_step_size:.2f}, tilt_step: {tilt_step_size:.2f}")
    
    return pan_controller.target_position, tilt_controller.target_position

def handle_key_event(key, pan_controller, tilt_controller, terminal=None):
    """
    Handle key press events and update movement state.
    
    Provides the interface between user input and movement system:
    - Key state management
    - Velocity targeting
    - Movement initialization
    - Debug logging
    - Preset switching
    
    Args:
        key (int): ASCII value of pressed key
        pan_controller (PID_Controller): Controller for pan axis
        tilt_controller (PID_Controller): Controller for tilt axis
        terminal (TerminalHandler, optional): Terminal interface for messages
        
    Returns:
        tuple: (target_pan, target_tilt) positions
        
    Key mappings:
    - a/A: Pan left
    - d/D: Pan right
    - w/W: Tilt up
    - s/S: Tilt down
    - c/C: Center both axes
    - 1: Smooth preset
    - 2: Responsive preset
    - 3: Precise preset
    - 4: Balanced preset
    """
    global key_states, movement_state
    current_time = time.time()
    
    if not CONFIG['servo']['velocity']['enabled']:
        # Use legacy step-based movement if velocity mode is disabled
        return handle_key_event_legacy(key, pan_controller, tilt_controller)
    
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
    
    # Update key states and set target velocities
    max_speed = CONFIG['servo']['velocity']['max_speed']
    debug_enabled = CONFIG['debug'].get('pid_debug', False)
    
    def update_key_state(key_name, pressed, target_vel, opposing_key=None):
        """Helper to update key state with debug logging"""
        if pressed and not key_states[key_name]['pressed']:
            # Key just pressed
            key_states[key_name]['press_start'] = current_time
            key_states[key_name]['target_velocity'] = target_vel
            if debug_enabled:
                logging.debug(f"Key '{key_name}' pressed - target velocity: {target_vel:.1f}")
        
        key_states[key_name]['pressed'] = pressed
        
        # Handle opposing key
        if opposing_key and pressed:
            if key_states[opposing_key]['pressed']:
                if debug_enabled:
                    logging.debug(f"Releasing opposing key '{opposing_key}'")
            key_states[opposing_key]['pressed'] = False
            key_states[opposing_key]['target_velocity'] = 0.0
    
    if key in [ord('a'), ord('A')]:
        update_key_state('a', True, -max_speed, opposing_key='d')
    elif key in [ord('d'), ord('D')]:
        update_key_state('d', True, max_speed, opposing_key='a')
    elif key in [ord('w'), ord('W')]:
        update_key_state('w', True, -max_speed, opposing_key='s')
    elif key in [ord('s'), ord('S')]:
        update_key_state('s', True, max_speed, opposing_key='w')
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
            logging.debug(f"Active keys: {active_keys}, "
                        f"Pan velocity targets: [a: {key_states['a']['target_velocity']:.1f}, "
                        f"d: {key_states['d']['target_velocity']:.1f}], "
                        f"Tilt velocity targets: [w: {key_states['w']['target_velocity']:.1f}, "
                        f"s: {key_states['s']['target_velocity']:.1f}]")
    
    # Process movement immediately to avoid delay
    return process_held_keys(pan_controller, tilt_controller)

def handle_key_event_legacy(key, pan_controller, tilt_controller):
    """Legacy step-based key handling"""
    global key_states
    current_pan = pan_controller.command
    current_tilt = tilt_controller.command
    
    # Get step sizes from config
    pan_step = CONFIG['servo']['pan_step']
    tilt_step = CONFIG['servo']['tilt_step']
    
    # Update key states
    current_time = time.time()
    if key in [ord('a'), ord('A')]:
        if not key_states['a']['pressed']:
            key_states['a']['press_start'] = current_time
        key_states['a']['pressed'] = True
        new_pan = max(PIXY_RCS_MINIMUM_POSITION, current_pan - pan_step)
        pan_controller.set_target(new_pan)
    elif key in [ord('d'), ord('D')]:
        if not key_states['d']['pressed']:
            key_states['d']['press_start'] = current_time
        key_states['d']['pressed'] = True
        new_pan = min(PIXY_RCS_MAXIMUM_POSITION, current_pan + pan_step)
        pan_controller.set_target(new_pan)
    elif key in [ord('s'), ord('S')]:
        if not key_states['s']['pressed']:
            key_states['s']['press_start'] = current_time
        key_states['s']['pressed'] = True
        new_tilt = min(PIXY_RCS_MAXIMUM_POSITION, current_tilt + tilt_step)
        tilt_controller.set_target(new_tilt)
    elif key in [ord('w'), ord('W')]:
        if not key_states['w']['pressed']:
            key_states['w']['press_start'] = current_time
        key_states['w']['pressed'] = True
        new_tilt = max(PIXY_RCS_MINIMUM_POSITION, current_tilt - tilt_step)
        tilt_controller.set_target(new_tilt)
    elif key in [ord('c'), ord('C')]:
        for k in key_states:
            key_states[k]['pressed'] = False
            key_states[k]['press_start'] = 0
        pan_controller.set_target(PIXY_RCS_CENTER_POSITION)
        tilt_controller.set_target(PIXY_RCS_CENTER_POSITION)
    
    return pan_controller.target_position, tilt_controller.target_position

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
                    target_pan, target_tilt = handle_key_event(key, pan_controller, tilt_controller, terminal)

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

            # Check for key releases
            if not is_data():  # No keys being pressed
                for k in key_states:
                    if key_states[k]['pressed']:
                        key_states[k]['pressed'] = False
                        key_states[k]['press_start'] = 0
                        if CONFIG['debug'].get('pid_debug', False):
                            logging.debug(f"Key {k} released")

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

if __name__ == "__main__":
    main() 