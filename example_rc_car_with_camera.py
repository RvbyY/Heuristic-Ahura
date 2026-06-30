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


def compute_camera_steer(result, offset_gain, lookahead_gain, heading_gain, max_abs_steer):
    """Compute steering from lane center offset and lookahead lane heading."""
    steer = (
        offset_gain * result['center_offset'] +
        lookahead_gain * result['lookahead_offset'] +
        heading_gain * result['heading_error']
    )
    return max(-max_abs_steer, min(max_abs_steer, steer))


def smooth_steer(target_steer, previous_steer, alpha, max_change):
    """Low-pass and rate-limit steering to avoid reacting to one noisy frame."""
    filtered = previous_steer + alpha * (target_steer - previous_steer)
    delta = max(-max_change, min(max_change, filtered - previous_steer))
    return previous_steer + delta


def lane_status(result, args):
    """Classify lane fit quality for safe driving decisions."""
    lane_width = result['lane_width']
    if lane_width is None:
        return "LOST"
    lane_width_abs = abs(lane_width)
    if lane_width_abs < args.min_lane_width:
        return "BADWID"
    max_lane_width = args.max_lane_width
    if max_lane_width <= 0:
        max_lane_width = args.camera_width * args.max_lane_width_factor
    if lane_width_abs > max_lane_width:
        return "BADWID"
    if abs(result['center_offset']) > args.max_lane_offset:
        return "BADOFF"
    if abs(result['lookahead_offset']) > args.max_lane_offset:
        return "BADOFF"
    if abs(result['heading_error']) > args.max_lane_heading:
        return "BADHDG"
    return "OK"


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
        default=10.0,
        help="Maximum VESC duty command in percent. Manual driving used 15.",
    )
    parser.add_argument(
        "--min-forward-duty-percent",
        type=float,
        default=8.0,
        help="Minimum positive VESC duty used to overcome motor deadband.",
    )
    parser.add_argument(
        "--drive-duty-percent",
        type=float,
        default=8.0,
        help="Fixed forward VESC duty used while lane geometry is valid.",
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=416,
        help="Camera preview width. Lower values reduce OAK USB load.",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=320,
        help="Camera preview height. Lower values reduce OAK USB load.",
    )
    parser.add_argument(
        "--camera-fps",
        type=int,
        default=10,
        help="Camera FPS. Lower values reduce OAK USB load.",
    )
    parser.add_argument(
        "--max-missed-frames",
        type=int,
        default=10,
        help="Stop after this many consecutive empty camera reads.",
    )
    parser.add_argument(
        "--estimate-speed",
        action="store_true",
        help="Enable optical-flow speed estimation. Disabled by default for faster steering updates.",
    )
    parser.add_argument(
        "--lookahead-y-fraction",
        type=float,
        default=0.75,
        help="Lookahead row in ROI: 0=far/top, 1=near/bottom. Higher is steadier.",
    )
    parser.add_argument(
        "--lane-offset-gain",
        type=float,
        default=0.7,
        help="Steering gain for current lane center offset.",
    )
    parser.add_argument(
        "--lane-lookahead-gain",
        type=float,
        default=1.6,
        help="Steering gain for lookahead lane center offset.",
    )
    parser.add_argument(
        "--lane-heading-gain",
        type=float,
        default=0.6,
        help="Steering gain for lookahead lane heading.",
    )
    parser.add_argument(
        "--max-steer",
        type=float,
        default=1.0,
        help="Maximum absolute autonomous steering command.",
    )
    parser.add_argument(
        "--steer-filter-alpha",
        type=float,
        default=0.45,
        help="Steering smoothing factor. Lower is smoother.",
    )
    parser.add_argument(
        "--max-steer-change",
        type=float,
        default=0.25,
        help="Maximum steering change per camera frame.",
    )
    parser.add_argument(
        "--max-lane-offset",
        type=float,
        default=0.85,
        help="Reject lane fits with saturated center/lookahead offsets.",
    )
    parser.add_argument(
        "--max-lane-heading",
        type=float,
        default=0.75,
        help="Reject lane fits with impossible heading jumps.",
    )
    parser.add_argument(
        "--min-lane-width",
        type=float,
        default=10.0,
        help="Reject lane fits narrower than this many pixels.",
    )
    parser.add_argument(
        "--max-lane-width",
        type=float,
        default=0.0,
        help="Reject lane fits wider than this many pixels. 0 uses camera_width * max_lane_width_factor.",
    )
    parser.add_argument(
        "--max-lane-width-factor",
        type=float,
        default=1.25,
        help="When max lane width is 0, reject widths above camera_width times this factor.",
    )
    parser.add_argument(
        "--lane-loss-grace-frames",
        type=int,
        default=5,
        help="Continue briefly with last good steering after transient lane loss.",
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
        vision = CameraVision(
            camera_id=0,
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
            estimate_speed_enabled=args.estimate_speed,
            lookahead_y_fraction=args.lookahead_y_fraction,
        )
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
    prev_steer = 0.0
    last_good_steer = 0.0
    lane_loss_frames = 0
    has_good_lane = False
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
            lane_state = lane_status(result, args)
            lane_valid = lane_state == "OK"
            lane_offset = result['center_offset']
            lookahead_offset = result['lookahead_offset']
            heading_error = result['heading_error']
            lane_width = result['lane_width']
            
            # Alternative: use average pace for more stable speed estimate
            # current_speed = result['avg_pace']

            use_grace_drive = (
                not lane_valid and
                has_good_lane and
                lane_loss_frames < args.lane_loss_grace_frames
            )

            if lane_valid:
                lane_loss_frames = 0
            else:
                lane_loss_frames += 1

            if not lane_valid and not args.allow_no_lane and not use_grace_drive:
                car_servo.write(0.5)
                car_esc.write(0.0, 0.0)
                output = agent.output(0.0, 0.0, 0.0)
                prev_steer = 0.0
                drive_mode = "STOP"
            elif use_grace_drive:
                car_servo.write((last_good_steer + 1.0) / 2.0)
                car_esc.write_percent(args.drive_duty_percent)
                output = agent.output(last_good_steer, 0.0, 0.0)
                drive_mode = "GRACE"
            else:
                # Create sensors object
                sensors = create_simple_sensors(dist_readings, current_speed)
                camera_steer = None
                if lane_valid:
                    target_steer = compute_camera_steer(
                        result,
                        args.lane_offset_gain,
                        args.lane_lookahead_gain,
                        args.lane_heading_gain,
                        args.max_steer,
                    )
                    camera_steer = smooth_steer(
                        target_steer,
                        prev_steer,
                        args.steer_filter_alpha,
                        args.max_steer_change,
                    )
            
                # Run control algorithm (no MLP model for friction estimation)
                output = main_loop(
                    sensors,
                    state,
                    values,
                    car_esc,
                    car_servo,
                    mlp_model=None,
                    enable_stuck_recovery=False,
                    steer_override=camera_steer,
                )
                prev_steer = output.steer
                if lane_valid:
                    last_good_steer = output.steer
                    has_good_lane = True
                    car_esc.write_percent(args.drive_duty_percent)
                drive_mode = "DRIVE"
            
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
                      f"Lane: {lane_state:7s} | "
                      f"Mode: {drive_mode:5s} | "
                      f"Steer: {output.steer:+5.2f} | "
                      f"Lane Offset: {lane_offset:+5.2f} | "
                      f"Lookahead: {lookahead_offset:+5.2f} | "
                      f"Heading: {heading_error:+5.2f} | "
                      f"Width: {0.0 if lane_width is None else lane_width:6.1f} | "
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
