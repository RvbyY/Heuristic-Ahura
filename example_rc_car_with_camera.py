#!/usr/bin/env python3
"""
Example script showing how to use the adapted agent.py with RC car hardware and camera vision.

This demonstrates:
1. Loading configuration from YAML
2. Using camera for lane detection and speed estimation
3. Running the control loop with VESC motor and servo control
4. Proper cleanup on exit

Requirements:
- Camera (USB or CSI camera on Jetson)
- OpenCV (cv2)
- VESC serial connection for servo control
"""

import sys
import os
import time
import argparse
import yaml
import cv2

import agent
from train import VESCRobocar, main_loop, create_simple_sensors, create_simple_state
from camera_vision import CameraVision


def display_available():
    """Return True when OpenCV can reasonably open a local preview window."""
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def load_config(config_path='configs/base_heuristic.yaml'):
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return agent.heuristic_value(cfg)


def parse_args():
    parser = argparse.ArgumentParser(description="Run RC car control with camera vision.")
    parser.add_argument(
        "--allow-no-lane",
        action="store_true",
        help="Allow driving with default distance readings when no lane is detected.",
    )
    parser.add_argument(
        "--max-duty-percent",
        type=float,
        default=5.0,
        help="Maximum VESC duty command in percent. Default matches the lowest manual motor test.",
    )
    parser.add_argument(
        "--min-forward-duty-percent",
        type=float,
        default=5.0,
        help="Minimum positive VESC duty used to overcome motor deadband.",
    )
    parser.add_argument(
        "--camera-fps",
        type=int,
        default=15,
        help="Camera FPS. Lower values reduce OAK USB load.",
    )
    parser.add_argument(
        "--max-missed-frames",
        type=int,
        default=10,
        help="Stop after this many consecutive empty camera reads.",
    )
    return parser.parse_args()


def main():
    """Main control loop for RC car with camera vision."""
    args = parse_args()
    print("Initializing RC car control system with camera vision...")
    
    # Load configuration
    values = load_config()
    print(f"Configuration loaded: max_speed={values.max_speed}, min_speed={values.min_speed}")
    
    # Initialize state
    state = create_simple_state()
    print("State initialized")
    
    # Initialize camera vision
    try:
        vision = CameraVision(camera_id=0, width=640, height=480, fps=args.camera_fps)
        print("Camera vision initialized")
    except Exception as e:
        print(f"Failed to initialize camera: {e}")
        print("Make sure a camera is connected")
        return
    
    # Initialize RC car hardware
    try:
        car = VESCRobocar(
            port='/dev/ttyACM0',
            baudrate=115200,
            max_duty_percent=args.max_duty_percent,
            min_forward_duty_percent=args.min_forward_duty_percent,
        )
        car_servo = car.servo
        car_esc = car.throttle
        print(
            "VESC connected on /dev/ttyACM0 "
            f"(max duty={args.max_duty_percent:.1f}%, "
            f"min forward={args.min_forward_duty_percent:.1f}%)"
        )
        
    except Exception as e:
        print(f"Failed to initialize hardware: {e}")
        print("Make sure:")
        print("  - VESC is connected to /dev/ttyACM0")
        print("  - Running with appropriate permissions")
        vision.cleanup()
        return
    
    # Center steering and stop motor
    car_servo.write(0.5)
    car_esc.write(0.0, 0.0)
    time.sleep(0.5)
    
    # Start race timer for pace calculation
    vision.start_race_timer()
    
    print("\nStarting control loop...")
    print("Press Ctrl+C to stop")
    gui_enabled = display_available()
    if gui_enabled:
        print("Camera window: Press 'q' to quit, 'd' to toggle debug view\n")
    else:
        print("No DISPLAY/WAYLAND_DISPLAY found; running without camera window")
        print("Use Ctrl+C to stop\n")
    
    show_debug = gui_enabled
    loop_count = 0
    missed_frames = 0
    start_time = time.time()
    
    try:
        while True:
            # Read camera frame
            frame = vision.read_frame()
            if frame is None:
                car_servo.write(0.5)
                car_esc.write(0.0, 0.0)
                missed_frames += 1
                if vision.last_read_error is not None:
                    print(f"Camera read failed: {vision.last_read_error}")
                    break
                if missed_frames >= args.max_missed_frames:
                    print(f"Failed to read camera frame {missed_frames} times in a row")
                    break
                time.sleep(values.dt)
                continue
            missed_frames = 0
            
            # Process frame: detect lanes and estimate speed
            result = vision.process_frame(frame, draw_debug=show_debug)
            
            # Get distance sensors from lane detection
            dist_readings = result['distance_sensors']
            
            # Get current speed from camera
            current_speed = result['speed']
            lane_detected = result['lane_width'] is not None
            
            # Alternative: use average pace for more stable speed estimate
            # current_speed = result['avg_pace']

            if not lane_detected and not args.allow_no_lane:
                car_servo.write(0.5)
                car_esc.write(0.0, 0.0)
                output = agent.output(0.0, 0.0, 0.0)
                lane_offset = result['center_offset']
            else:
                # Create sensors object
                sensors = create_simple_sensors(dist_readings, current_speed)
            
                # Run control algorithm (no MLP model for friction estimation)
                output = main_loop(
                    sensors,
                    state,
                    values,
                    car_esc,
                    car_servo,
                    mlp_model=None,
                    enable_stuck_recovery=False,
                )
                
                # Optional: Use lane center offset to adjust steering
                # This provides additional feedback from camera
                lane_offset = result['center_offset']
                if abs(lane_offset) > 0.1:  # Significant offset from center
                    # Blend camera-based correction with algorithm output
                    # Negative offset = car is left of center, steer right (positive)
                    correction = -lane_offset * 0.3  # 30% correction factor
                    corrected_steer = output.steer + correction
                    corrected_steer = max(-1.0, min(1.0, corrected_steer))
                    
                    # Apply corrected steering
                    car_servo.write((corrected_steer + 1.0) / 2.0)
            
            # Display camera view
            if gui_enabled and result['frame'] is not None:
                cv2.imshow('RC Car Vision', result['frame'])
            
            # Debug output every 50 iterations (~1 second at 50Hz)
            loop_count += 1
            if loop_count % 50 == 0:
                elapsed = time.time() - start_time
                hz = loop_count / elapsed
                print(f"[{loop_count:5d}] Speed: {current_speed:5.2f} m/s | "
                      f"Pace: {result['avg_pace']:5.2f} m/s | "
                      f"Lane: {'OK' if lane_detected else 'LOST'} | "
                      f"Steer: {output.steer:+5.2f} | "
                      f"Lane Offset: {lane_offset:+5.2f} | "
                      f"Accel: {output.accel:4.2f} | "
                      f"Brake: {output.brake:4.2f} | "
                      f"VESC: {car_esc.last_command_percent:+5.2f}% | "
                      f"Hz: {hz:5.1f}")
            
            # Handle keyboard input
            if gui_enabled:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\nQuitting...")
                    break
                elif key == ord('d'):
                    show_debug = not show_debug
                    print(f"Debug view: {'ON' if show_debug else 'OFF'}")
            
            # Control loop timing (50Hz = 0.02s)
            time.sleep(values.dt)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    finally:
        # Clean shutdown
        print("Cleaning up...")
        car.cleanup()
        vision.cleanup()
        print("RC car stopped and cleaned up")


if __name__ == '__main__':
    main()
