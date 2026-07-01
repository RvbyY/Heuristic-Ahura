#!/usr/bin/env python3
"""
Example script showing how to use the adapted agent.py with RC car hardware and camera vision.

This demonstrates:
1. Loading configuration from YAML
2. Using camera for lane detection and speed estimation
3. Running the control loop with GPIO/ESC and Servo control
4. Proper cleanup on exit

Requirements:
- Camera (USB or CSI camera on Jetson)
- OpenCV (cv2)
- Jetson GPIO for ESC control
- VESC serial connection for servo control
"""

import sys
import os
import time
import yaml
import cv2

import agent
from train import ESC, Servo, main_loop, create_simple_sensors, create_simple_state
from camera_vision import CameraVision


def load_config(config_path='configs/base_heuristic.yaml'):
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return agent.heuristic_value(cfg)


def main():
    """Main control loop for RC car with camera vision."""
    print("Initializing RC car control system with camera vision...")
    
    # Load configuration
    values = load_config()
    print(f"Configuration loaded: max_speed={values.max_speed}, min_speed={values.min_speed}")
    
    # Initialize state
    state = create_simple_state()
    print("State initialized")
    
    # Initialize camera vision (OAK-D Lite via USB-C, DepthAI)
    try:
        vision = CameraVision(width=640, height=480, fps=30, use_oak=True)
        print("Camera vision initialized (OAK-D Lite)")
    except Exception as e:
        print(f"Failed to initialize camera: {e}")
        print("Make sure the OAK-D Lite is plugged in USB-C (USB3 port)")
        print("and depthai is installed: pip3 install depthai")
        return

    # Initialize RC car hardware
    try:
        # Servo control via VESC serial
        car_servo = Servo(port='/dev/ttyACM0', baudrate=115200)
        print("Servo connected via VESC serial")

        # ESC control via VESC serial (same port, same VESC)
        car_esc = ESC(port='/dev/ttyACM0', baudrate=115200)
        print("ESC connected via VESC serial")

    except Exception as e:
        print(f"Failed to initialize hardware: {e}")
        print("Make sure:")
        print("  - VESC is connected to /dev/ttyACM0")
        print("  - pyvesc is installed: pip3 install pyvesc")
        print("  - Running with appropriate permissions (sudo or dialout group)")
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
    print("Camera window: Press 'q' to quit, 'd' to toggle debug view\n")
    
    show_debug = True
    loop_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Read camera frame
            frame = vision.read_frame()
            if frame is None:
                print("Failed to read camera frame")
                break
            
            # Process frame: detect lanes and estimate speed
            result = vision.process_frame(frame, draw_debug=show_debug)
            
            # Get distance sensors from lane detection
            dist_readings = result['distance_sensors']
            
            # Get current speed from camera
            current_speed = result['speed']
            
            # Alternative: use average pace for more stable speed estimate
            # current_speed = result['avg_pace']
            
            # Create sensors object
            sensors = create_simple_sensors(dist_readings, current_speed)
            
            # Run control algorithm (no MLP model for friction estimation)
            output = main_loop(sensors, state, values, car_esc, car_servo, mlp_model=None)
            
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
            if result['frame'] is not None:
                cv2.imshow('RC Car Vision', result['frame'])
            
            # Debug output every 50 iterations (~1 second at 50Hz)
            loop_count += 1
            if loop_count % 50 == 0:
                elapsed = time.time() - start_time
                hz = loop_count / elapsed
                print(f"[{loop_count:5d}] Speed: {current_speed:5.2f} m/s | "
                      f"Pace: {result['avg_pace']:5.2f} m/s | "
                      f"Steer: {output.steer:+5.2f} | "
                      f"Lane Offset: {lane_offset:+5.2f} | "
                      f"Accel: {output.accel:4.2f} | "
                      f"Brake: {output.brake:4.2f} | "
                      f"Hz: {hz:5.1f}")
            
            # Handle keyboard input
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
        car_esc.cleanup()
        car_servo.cleanup()
        vision.cleanup()
        print("RC car stopped and cleaned up")


if __name__ == '__main__':
    main()