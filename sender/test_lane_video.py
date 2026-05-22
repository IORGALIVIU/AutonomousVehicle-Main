#!/usr/bin/env python3
"""
Test Autonomous Lane Following on Pre-recorded Video
Run this to test lane detection and control before deploying to robot.
"""

import cv2
import argparse
import time
import logging
from pathlib import Path
from autonomous_driver import AutonomousDriver

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_video(video_path, 
               output_path=None,
               enable_hardware=False,
               max_frames=None,
               playback_speed=1.0):
    """
    Test autonomous driving on video file.
    
    Args:
        video_path: Path to input video
        output_path: Path to save output video (None = don't save)
        enable_hardware: If True, control actual robot
        max_frames: Maximum frames to process (None = all)
        playback_speed: Playback speed multiplier (1.0 = real-time)
    """
    # Check video exists
    if not Path(video_path).exists():
        logger.error(f"Video file not found: {video_path}")
        return
    
    # Open video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        logger.error(f"Cannot open video: {video_path}")
        return
    
    # Get video properties
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    logger.info("=" * 60)
    logger.info("Autonomous Lane Following Test")
    logger.info("=" * 60)
    logger.info(f"Video: {video_path}")
    logger.info(f"Resolution: {width}x{height}")
    logger.info(f"FPS: {fps:.2f}")
    logger.info(f"Total frames: {total_frames}")
    logger.info(f"Hardware control: {'ENABLED' if enable_hardware else 'DISABLED'}")
    logger.info("=" * 60)
    
    # Initialize autonomous driver
    driver = AutonomousDriver(
        img_width=width,
        img_height=height,
        enable_hardware=enable_hardware,
        show_visualization=True
    )
    
    # Setup video writer if saving output
    writer = None
    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        logger.info(f"Saving output to: {output_path}")
    
    # Processing loop
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Read frame
            ret, frame = cap.read()
            if not ret:
                logger.info("End of video reached")
                break
            
            # Check max frames limit
            if max_frames and frame_count >= max_frames:
                logger.info(f"Reached max frames limit: {max_frames}")
                break
            
            # Process frame
            result = driver.process_frame(frame)
            
            # Display
            if result['annotated_frame'] is not None:
                # Resize for display if too large
                display_frame = result['annotated_frame']
                display_height = 720
                if height > display_height:
                    scale = display_height / height
                    display_width = int(width * scale)
                    display_frame = cv2.resize(display_frame, 
                                              (display_width, display_height))
                
                cv2.imshow('Autonomous Lane Following', display_frame)
                
                # Save to output video
                if writer:
                    writer.write(result['annotated_frame'])
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("User quit")
                break
            elif key == ord('p'):
                logger.info("Paused - press any key to continue")
                cv2.waitKey(0)
            elif key == ord('s'):
                # Save current frame
                timestamp = int(time.time())
                save_path = f"frame_{timestamp}.jpg"
                cv2.imwrite(save_path, result['annotated_frame'])
                logger.info(f"Frame saved: {save_path}")
            
            # Throttle playback speed
            if playback_speed < 10:  # Don't throttle if very fast
                time.sleep((1.0 / fps) / playback_speed)
            
            frame_count += 1
            
            # Progress update every 30 frames
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                current_fps = frame_count / elapsed
                progress = (frame_count / total_frames) * 100
                logger.info(f"Progress: {progress:.1f}% | "
                          f"Frame: {frame_count}/{total_frames} | "
                          f"FPS: {current_fps:.1f}")
    
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    
    finally:
        # Cleanup
        elapsed = time.time() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0
        
        logger.info("=" * 60)
        logger.info("Test Summary")
        logger.info("=" * 60)
        logger.info(f"Frames processed: {frame_count}")
        logger.info(f"Total time: {elapsed:.2f}s")
        logger.info(f"Average FPS: {avg_fps:.2f}")
        logger.info(f"Average processing time: {driver.processing_time_avg*1000:.1f}ms/frame")
        logger.info("=" * 60)
        
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        driver.cleanup()
        
        logger.info("Test complete!")


def main():
    parser = argparse.ArgumentParser(
        description="Test Autonomous Lane Following on Video"
    )
    parser.add_argument(
        "--video",
        required=True,
        help="Path to input video file"
    )
    parser.add_argument(
        "--output",
        help="Path to save output video (optional)"
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Enable robot hardware control (CAUTION!)"
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        help="Maximum frames to process (default: all)"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (default: 1.0)"
    )
    
    args = parser.parse_args()
    
    # Warning if hardware enabled
    if args.hardware:
        print("\n" + "!" * 60)
        print("WARNING: Hardware control is ENABLED!")
        print("Make sure robot is safely positioned before continuing.")
        print("!" * 60)
        response = input("\nContinue? (yes/no): ")
        if response.lower() != 'yes':
            print("Aborted.")
            return
    
    test_video(
        video_path=args.video,
        output_path=args.output,
        enable_hardware=args.hardware,
        max_frames=args.max_frames,
        playback_speed=args.speed
    )


if __name__ == "__main__":
    main()
