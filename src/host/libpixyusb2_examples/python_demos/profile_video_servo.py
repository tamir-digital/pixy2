#!/usr/bin/env python3

import cProfile
import pstats
import io
import sys
import os
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def profile_video_servo():
    """Profile video_servo.py execution"""
    # Create profile output directory
    profile_dir = "profile_results"
    if not os.path.exists(profile_dir):
        os.makedirs(profile_dir)
    
    # Generate timestamped output file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stats_file = os.path.join(profile_dir, f"video_servo_profile_{timestamp}.txt")
    
    logger.info("Starting video_servo.py profiling...")
    logger.info(f"Results will be saved to: {stats_file}")
    
    # Create profiler
    pr = cProfile.Profile()
    
    # Start profiling
    try:
        import video_servo
        pr.enable()
        video_servo.main()  # This will run until user exits
    except KeyboardInterrupt:
        logger.info("\nProfiling stopped by user")
    except Exception as e:
        logger.error(f"Error during profiling: {str(e)}")
    finally:
        # Stop profiling
        pr.disable()
        
        # Sort stats by cumulative time
        s = io.StringIO()
        ps = pstats.Stats(pr, stream=s).sort_stats('cumulative')
        
        # Print top 50 time-consuming functions
        ps.print_stats(50)
        
        # Save detailed stats to file
        with open(stats_file, 'w') as f:
            # Write summary
            f.write("=== VIDEO_SERVO.PY PROFILE RESULTS ===\n\n")
            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("Top 50 time-consuming functions:\n\n")
            f.write(s.getvalue())
            
            # Write function callers/callees
            f.write("\nDetailed call information:\n")
            f.write("=" * 50 + "\n")
            ps.print_callers(50, file=f)
            f.write("\nFunction call chains:\n")
            f.write("=" * 50 + "\n")
            ps.print_callees(50, file=f)
        
        logger.info(f"\nProfile results saved to: {stats_file}")
        logger.info("\nTop 10 time-consuming functions:")
        print("\n" + "\n".join(s.getvalue().split("\n")[:15]))

if __name__ == "__main__":
    profile_video_servo() 