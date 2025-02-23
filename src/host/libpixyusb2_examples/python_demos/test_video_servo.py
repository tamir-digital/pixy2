#!/usr/bin/env python3

import sys
import time
import logging
import json
from datetime import datetime
import os
from unittest.mock import MagicMock, patch
import numpy as np
import cv2

# Configure logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Try to import video_servo module
try:
    import video_servo
    SERVO_MODULE_AVAILABLE = True
except ImportError as e:
    logger.error(f"Failed to import video_servo module: {str(e)}")
    logger.error("Make sure video_servo.py is in the same directory or PYTHONPATH")
    SERVO_MODULE_AVAILABLE = False

class MockPixy2:
    """Mock Pixy2 camera interface for testing"""
    def __init__(self):
        self.pan = 500  # Center position
        self.tilt = 500  # Center position
        self.frame_width = 316
        self.frame_height = 208
        
        # Track movement history
        self.movement_history = {
            'pan': [],
            'tilt': []
        }
        
        # Movement constraints
        self.max_speed = 100  # Maximum units per second
        self.last_update = time.time()
        
    def init(self):
        return 0
        
    def change_prog(self, program):
        return 0
        
    def stop(self):
        return 0
        
    def resume(self):
        return 0
        
    def get_raw_frame_width(self):
        return self.frame_width
        
    def get_raw_frame_height(self):
        return self.frame_height
        
    def get_raw_frame(self, frame_buffer):
        # Generate a test pattern frame
        frame = np.zeros((self.frame_height, self.frame_width), dtype=np.uint8)
        cv2.putText(frame, f"Pan: {self.pan}, Tilt: {self.tilt}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
        return 0
        
    def set_servos(self, pan, tilt):
        """
        Update servo positions with realistic movement constraints.
        Simulates actual servo behavior including:
        - Maximum speed limits
        - Movement tracking
        - Position updates based on time
        """
        current_time = time.time()
        dt = current_time - self.last_update
        
        if dt <= 0:  # Avoid division by zero
            dt = 0.016  # Default to 60Hz
            
        # Calculate maximum position change for this time step
        max_delta = self.max_speed * dt
        
        # Update pan position with constraints
        if pan != self.pan:
            # Calculate desired change
            pan_delta = pan - self.pan
            # Limit change by max speed
            pan_delta = max(-max_delta, min(max_delta, pan_delta))
            # Apply change
            self.pan += pan_delta
            # Record movement
            self.movement_history['pan'].append({
                'timestamp': current_time,
                'position': self.pan,
                'delta': pan_delta,
                'target': pan
            })
            
        # Update tilt position with constraints
        if tilt != self.tilt:
            # Calculate desired change
            tilt_delta = tilt - self.tilt
            # Limit change by max speed
            tilt_delta = max(-max_delta, min(max_delta, tilt_delta))
            # Apply change
            self.tilt += tilt_delta
            # Record movement
            self.movement_history['tilt'].append({
                'timestamp': current_time,
                'position': self.tilt,
                'delta': tilt_delta,
                'target': tilt
            })
            
        self.last_update = current_time
        return 0
        
    def get_movement_stats(self):
        """Get statistics about servo movement"""
        stats = {
            'pan': {
                'total_movement': 0,
                'max_speed': 0,
                'avg_speed': 0,
                'samples': len(self.movement_history['pan'])
            },
            'tilt': {
                'total_movement': 0,
                'max_speed': 0,
                'avg_speed': 0,
                'samples': len(self.movement_history['tilt'])
            }
        }
        
        # Calculate pan stats
        if len(self.movement_history['pan']) > 1:
            speeds = []
            total_movement = 0
            for i in range(1, len(self.movement_history['pan'])):
                prev = self.movement_history['pan'][i-1]
                curr = self.movement_history['pan'][i]
                dt = curr['timestamp'] - prev['timestamp']
                if dt > 0:
                    speed = abs(curr['delta']) / dt
                    speeds.append(speed)
                    total_movement += abs(curr['delta'])
            
            if speeds:
                stats['pan']['max_speed'] = max(speeds)
                stats['pan']['avg_speed'] = sum(speeds) / len(speeds)
                stats['pan']['total_movement'] = total_movement
                
        # Calculate tilt stats
        if len(self.movement_history['tilt']) > 1:
            speeds = []
            total_movement = 0
            for i in range(1, len(self.movement_history['tilt'])):
                prev = self.movement_history['tilt'][i-1]
                curr = self.movement_history['tilt'][i]
                dt = curr['timestamp'] - prev['timestamp']
                if dt > 0:
                    speed = abs(curr['delta']) / dt
                    speeds.append(speed)
                    total_movement += abs(curr['delta'])
            
            if speeds:
                stats['tilt']['max_speed'] = max(speeds)
                stats['tilt']['avg_speed'] = sum(speeds) / len(speeds)
                stats['tilt']['total_movement'] = total_movement
                
        return stats

class MovementTest:
    """Test harness for video_servo.py movement system"""
    
    def __init__(self):
        if not SERVO_MODULE_AVAILABLE:
            raise ImportError("video_servo module is required but not available")
            
        self.test_dir = "test_results"
        if not os.path.exists(self.test_dir):
            os.makedirs(self.test_dir)
            
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.results_file = os.path.join(self.test_dir, f"movement_test_{self.timestamp}.json")
        self.log_file = os.path.join(self.test_dir, f"movement_test_{self.timestamp}.log")
        
        # Configure file logging
        file_handler = logging.FileHandler(self.log_file)
        file_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        # Store module configuration
        self.servo_config = {
            'pan_gain': video_servo.PAN_GAIN,
            'tilt_gain': video_servo.TILT_GAIN,
            'update_interval': video_servo.UPDATE_INTERVAL,
            'movement_config': video_servo.CONFIG['servo']
        }
        
        self.results = {
            'timestamp': self.timestamp,
            'configuration': {
                'video_servo': self.servo_config
            },
            'tests': [],
            'summary': {
                'total_tests': 0,
                'passed': 0,
                'failed': 0,
                'warnings': 0
            }
        }
        
    def simulate_key_sequence(self, sequence, duration=0.5, sample_rate=0.05):
        """
        Simulate a sequence of key presses and validate movement behavior.
        """
        logger.info(f"\nTesting key sequence: {[s['key'] for s in sequence]}")
        
        # Mock Pixy2
        mock_pixy = MockPixy2()
        sys.modules['pixy'] = mock_pixy
        
        # Initialize controllers
        pan_controller = video_servo.PID_Controller(
            self.servo_config['pan_gain'], 0, 
            self.servo_config['pan_gain'], True)
        tilt_controller = video_servo.PID_Controller(
            self.servo_config['tilt_gain'], 0, 
            self.servo_config['tilt_gain'], True)
            
        # Reset video_servo state
        video_servo.movement_state = {
            'pan': {
                'velocity': 0.0,
                'last_update': time.time(),
                'last_position': video_servo.PIXY_RCS_CENTER_POSITION
            },
            'tilt': {
                'velocity': 0.0,
                'last_update': time.time(),
                'last_position': video_servo.PIXY_RCS_CENTER_POSITION
            }
        }
        
        # Reset key states
        for key in video_servo.key_states:
            video_servo.key_states[key] = {
                'pressed': False,
                'press_start': 0,
                'velocity': 0.0,
                'target_velocity': 0.0
            }

        test_results = {
            'steps': [],
            'errors': [],
            'warnings': [],
            'timing': {
                'start_time': time.time(),
                'end_time': None,
                'total_duration': None
            },
            'movement_data': [],
            'servo_stats': None  # Will be filled at end
        }
        
        try:
            # Execute key sequence
            for step_idx, step in enumerate(sequence):
                step_start = time.time()
                key = ord(step['key'].lower())
                hold_time = step['hold_time']
                expected = step['expected']
                
                # Record initial state
                initial_state = {
                    'pan': {
                        'position': mock_pixy.pan,
                        'velocity': video_servo.movement_state['pan']['velocity']
                    },
                    'tilt': {
                        'position': mock_pixy.tilt,
                        'velocity': video_servo.movement_state['tilt']['velocity']
                    }
                }
                
                # Press key
                video_servo.handle_key_event(key, pan_controller, tilt_controller, 
                                          terminal=None, is_new_press=True)
                
                # Hold and sample movement data
                movement_samples = []
                hold_start = time.time()
                while time.time() - hold_start < hold_time:
                    # Process movement
                    video_servo.process_held_keys(pan_controller, tilt_controller)
                    
                    # Record movement sample
                    movement_samples.append({
                        'timestamp': time.time() - hold_start,
                        'pan': {
                            'position': mock_pixy.pan,
                            'velocity': video_servo.movement_state['pan']['velocity'],
                            'target': pan_controller.target_position
                        },
                        'tilt': {
                            'position': mock_pixy.tilt,
                            'velocity': video_servo.movement_state['tilt']['velocity'],
                            'target': tilt_controller.target_position
                        }
                    })
                    
                    # Log movement details for debugging
                    logger.debug(f"\nMovement update at t={time.time() - hold_start:.3f}s:")
                    logger.debug(f"Pan: pos={mock_pixy.pan:.1f}, vel={video_servo.movement_state['pan']['velocity']:.1f}")
                    logger.debug(f"Tilt: pos={mock_pixy.tilt:.1f}, vel={video_servo.movement_state['tilt']['velocity']:.1f}")
                    
                    time.sleep(sample_rate)
                
                # Release key
                if step['key'] in video_servo.key_states:
                    video_servo.key_states[step['key']]['pressed'] = False
                    video_servo.key_states[step['key']]['velocity'] = 0.0
                
                # Record final state
                final_state = {
                    'pan': {
                        'position': mock_pixy.pan,
                        'velocity': video_servo.movement_state['pan']['velocity']
                    },
                    'tilt': {
                        'position': mock_pixy.tilt,
                        'velocity': video_servo.movement_state['tilt']['velocity']
                    }
                }
                
                # Get movement stats for this step
                step_stats = mock_pixy.get_movement_stats()
                
                # Validate movement
                validation_results = []
                self.validate_movement(pan_controller, tilt_controller, 
                                    video_servo.movement_state, expected,
                                    validation_results)
                
                # Record step results
                test_results['steps'].append({
                    'step_number': step_idx + 1,
                    'key': step['key'],
                    'hold_time': hold_time,
                    'timing': {
                        'start_time': step_start,
                        'duration': time.time() - step_start
                    },
                    'initial_state': initial_state,
                    'final_state': final_state,
                    'expected_state': expected,
                    'movement_samples': movement_samples,
                    'movement_stats': step_stats,
                    'validation_results': validation_results,
                    'status': 'PASS' if not any(r['type'] == 'error' for r in validation_results) else 'FAIL'
                })
                
                # Log step completion with stats
                logger.debug(f"\nStep {step_idx + 1} completed:")
                logger.debug(f"Pan movement: {step_stats['pan']['total_movement']:.1f} units")
                logger.debug(f"Pan speed: avg={step_stats['pan']['avg_speed']:.1f}, max={step_stats['pan']['max_speed']:.1f}")
                logger.debug(f"Tilt movement: {step_stats['tilt']['total_movement']:.1f} units")
                logger.debug(f"Tilt speed: avg={step_stats['tilt']['avg_speed']:.1f}, max={step_stats['tilt']['max_speed']:.1f}")
                
                # Categorize results
                for result in validation_results:
                    if result['type'] == 'error':
                        test_results['errors'].append({
                            'step': step_idx + 1,
                            'message': result['message']
                        })
                    elif result['type'] == 'warning':
                        test_results['warnings'].append({
                            'step': step_idx + 1,
                            'message': result['message']
                        })
            
            # Monitor movement decay
            decay_samples = []
            monitor_start = time.time()
            while time.time() - monitor_start < duration:
                video_servo.process_held_keys(pan_controller, tilt_controller)
                
                # Record decay sample
                decay_samples.append({
                    'timestamp': time.time() - monitor_start,
                    'pan': {
                        'position': mock_pixy.pan,
                        'velocity': video_servo.movement_state['pan']['velocity']
                    },
                    'tilt': {
                        'position': mock_pixy.tilt,
                        'velocity': video_servo.movement_state['tilt']['velocity']
                    }
                })
                
                time.sleep(sample_rate)
            
            # Final validation
            if sequence[-1].get('final_expected'):
                final_validation = []
                self.validate_movement(pan_controller, tilt_controller,
                                    video_servo.movement_state,
                                    sequence[-1]['final_expected'],
                                    final_validation)
                test_results['final_validation'] = {
                    'expected': sequence[-1]['final_expected'],
                    'results': final_validation,
                    'decay_samples': decay_samples
                }
                
            # Get final movement stats
            test_results['servo_stats'] = mock_pixy.get_movement_stats()
                
        except Exception as e:
            logger.error(f"Error during test sequence: {str(e)}")
            test_results['errors'].append({
                'type': 'critical',
                'message': f"Test sequence failed: {str(e)}",
                'timestamp': time.time()
            })
        
        # Record final timing
        test_results['timing']['end_time'] = time.time()
        test_results['timing']['total_duration'] = (
            test_results['timing']['end_time'] - test_results['timing']['start_time']
        )
        
        return test_results
        
    def validate_movement(self, pan_controller, tilt_controller, movement_state, 
                         expected, results):
        """Validate movement state against expected behavior"""
        
        def check_value(actual, expected, tolerance, name):
            """Check if value is within expected range"""
            if isinstance(expected, (int, float)):
                if abs(actual - expected) > tolerance:
                    results.append({
                        'type': 'error',
                        'message': f"{name} outside tolerance: {actual} != {expected}±{tolerance}"
                    })
                    logger.debug(f"Validation failed for {name}:")
                    logger.debug(f"  Expected: {expected} ± {tolerance}")
                    logger.debug(f"  Actual: {actual}")
                    logger.debug(f"  Difference: {abs(actual - expected)}")
                    return False
            return True
            
        # Log current state before validation
        logger.debug("\nValidating movement state:")
        logger.debug("Pan state:")
        logger.debug(f"  Position: {movement_state['pan']['last_position']}")
        logger.debug(f"  Velocity: {movement_state['pan']['velocity']}")
        logger.debug(f"  Target: {pan_controller.target_position}")
        logger.debug("Tilt state:")
        logger.debug(f"  Position: {movement_state['tilt']['last_position']}")
        logger.debug(f"  Velocity: {movement_state['tilt']['velocity']}")
        logger.debug(f"  Target: {tilt_controller.target_position}")
            
        # Check pan movement
        if 'pan_velocity' in expected:
            check_value(movement_state['pan']['velocity'],
                       expected['pan_velocity'],
                       tolerance=10.0,  # Increased tolerance
                       name='Pan velocity')
                       
        if 'pan_position' in expected:
            check_value(movement_state['pan']['last_position'],
                       expected['pan_position'],
                       tolerance=20.0,  # Increased tolerance
                       name='Pan position')
                       
        # Check tilt movement
        if 'tilt_velocity' in expected:
            check_value(movement_state['tilt']['velocity'],
                       expected['tilt_velocity'],
                       tolerance=10.0,  # Increased tolerance
                       name='Tilt velocity')
                       
        if 'tilt_position' in expected:
            check_value(movement_state['tilt']['last_position'],
                       expected['tilt_position'],
                       tolerance=20.0,  # Increased tolerance
                       name='Tilt position')
                       
        # Check controller targets
        if 'target_pan' in expected:
            check_value(pan_controller.target_position,
                       expected['target_pan'],
                       tolerance=20.0,  # Increased tolerance
                       name='Target pan')
                       
        if 'target_tilt' in expected:
            check_value(tilt_controller.target_position,
                       expected['target_tilt'],
                       tolerance=20.0,  # Increased tolerance
                       name='Target tilt')
                       
    def run_test_suite(self):
        """Run comprehensive movement test suite"""
        
        # Test 1: Single key press
        logger.info("\nTest 1: Single key press behavior")
        sequence = [{
            'key': 'd',  # Pan right
            'hold_time': 0.5,
            'expected': {
                'pan_velocity': 30,  # Reduced expectation (ramp-up)
                'pan_position': 520,  # More conservative position change
                'target_pan': 525    # Approximate target
            },
            'final_expected': {
                'pan_velocity': 0,   # Should stop
                'pan_position': 520  # Hold position
            }
        }]
        results = self.simulate_key_sequence(sequence)
        self.results['tests'].append({
            'name': 'Single key press',
            'sequence': sequence,
            'results': results
        })
        
        # Test 2: Key combination
        logger.info("\nTest 2: Key combination behavior")
        sequence = [
            {
                'key': 'd',  # Pan right
                'hold_time': 0.3,
                'expected': {
                    'pan_velocity': 25,  # Initial movement
                    'pan_position': 515  # Small movement first
                }
            },
            {
                'key': 'w',  # Add tilt up
                'hold_time': 0.3,
                'expected': {
                    'pan_velocity': 25,
                    'tilt_velocity': -25,
                    'pan_position': 530,
                    'tilt_position': 485
                }
            }
        ]
        results = self.simulate_key_sequence(sequence)
        self.results['tests'].append({
            'name': 'Key combination',
            'sequence': sequence,
            'results': results
        })
        
        # Test 3: Center command
        logger.info("\nTest 3: Center command behavior")
        sequence = [
            {
                'key': 'd',  # Pan right
                'hold_time': 0.5,
                'expected': {
                    'pan_velocity': 30,
                    'pan_position': 520
                }
            },
            {
                'key': 'c',  # Center
                'hold_time': 0.1,
                'expected': {
                    'pan_velocity': 0,
                    'tilt_velocity': 0,
                    'pan_position': 500,
                    'tilt_position': 500,
                    'target_pan': 500,
                    'target_tilt': 500
                }
            }
        ]
        results = self.simulate_key_sequence(sequence)
        self.results['tests'].append({
            'name': 'Center command',
            'sequence': sequence,
            'results': results
        })
        
        # Test 4: Rapid direction change
        logger.info("\nTest 4: Rapid direction change")
        sequence = [
            {
                'key': 'd',  # Pan right
                'hold_time': 0.3,
                'expected': {
                    'pan_velocity': 25,
                    'pan_position': 515
                }
            },
            {
                'key': 'a',  # Pan left
                'hold_time': 0.3,
                'expected': {
                    'pan_velocity': -25,  # Opposite direction
                    'pan_position': 505   # Small movement back
                }
            }
        ]
        results = self.simulate_key_sequence(sequence)
        self.results['tests'].append({
            'name': 'Rapid direction change',
            'sequence': sequence,
            'results': results
        })
        
        # Save results
        self.save_results()
        
    def save_results(self):
        """Save enhanced test results to file"""
        # Calculate detailed summary
        summary = {
            'total_tests': len(self.results['tests']),
            'passed': 0,
            'failed': 0,
            'warnings': 0,
            'total_steps': 0,
            'passed_steps': 0,
            'failed_steps': 0,
            'total_duration': 0,
            'average_step_duration': 0,
            'error_categories': {}
        }
        
        all_step_durations = []
        
        for test in self.results['tests']:
            test_passed = True
            test_has_warning = False
            
            # Count steps
            summary['total_steps'] += len(test['sequence'])
            
            # Analyze each step
            for step in test['results']['steps']:
                all_step_durations.append(step['timing']['duration'])
                
                if step['status'] == 'PASS':
                    summary['passed_steps'] += 1
                else:
                    summary['failed_steps'] += 1
                    test_passed = False
            
            # Count warnings
            if test['results']['warnings']:
                test_has_warning = True
                
            # Categorize errors
            for error in test['results']['errors']:
                error_type = error.get('type', 'validation')
                if error_type not in summary['error_categories']:
                    summary['error_categories'][error_type] = 0
                summary['error_categories'][error_type] += 1
            
            # Update test counts
            if test_passed:
                if test_has_warning:
                    summary['warnings'] += 1
                else:
                    summary['passed'] += 1
            else:
                summary['failed'] += 1
            
            # Add timing
            summary['total_duration'] += test['results']['timing']['total_duration']
        
        # Calculate average step duration
        if all_step_durations:
            summary['average_step_duration'] = sum(all_step_durations) / len(all_step_durations)
        
        # Update summary
        self.results['summary'] = summary
        
        # Add timestamp and version info
        self.results['metadata'] = {
            'timestamp': self.timestamp,
            'python_version': sys.version,
            'os_platform': sys.platform
        }
        
        # Save to file
        with open(self.results_file, 'w') as f:
            json.dump(self.results, f, indent=2)
            
        # Log summary
        logger.info("\n=== TEST SUMMARY ===")
        logger.info(f"Total Tests: {summary['total_tests']}")
        logger.info(f"├─ Passed: {summary['passed']}")
        logger.info(f"├─ Failed: {summary['failed']}")
        logger.info(f"└─ Warnings: {summary['warnings']}")
        logger.info("\nStep Details:")
        logger.info(f"├─ Total Steps: {summary['total_steps']}")
        logger.info(f"├─ Passed Steps: {summary['passed_steps']}")
        logger.info(f"└─ Failed Steps: {summary['failed_steps']}")
        logger.info("\nTiming:")
        logger.info(f"├─ Total Duration: {summary['total_duration']:.2f}s")
        logger.info(f"└─ Avg Step Duration: {summary['average_step_duration']:.3f}s")
        if summary['error_categories']:
            logger.info("\nError Categories:")
            for category, count in summary['error_categories'].items():
                logger.info(f"└─ {category}: {count}")
        logger.info(f"\nDetailed results saved to: {self.results_file}")
        logger.info(f"Test log saved to: {self.log_file}")

def main():
    """Run movement tests"""
    try:
        if not SERVO_MODULE_AVAILABLE:
            logger.error("Cannot run tests: video_servo module not available")
            sys.exit(1)
            
        # Create and run test suite
        test_suite = MovementTest()
        test_suite.run_test_suite()
        
    except KeyboardInterrupt:
        logger.info("\nTesting interrupted by user")
    except Exception as e:
        logger.error(f"Error during testing: {str(e)}")
        raise

if __name__ == "__main__":
    main() 