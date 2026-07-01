#!/usr/bin/env python3
"""
Example script showing how to use the adapted agent.py with RC car hardware and camera vision.

Requirements:
- OAK-D Lite camera via USB-C (USB3 port)
- depthai : pip3 install depthai
- pyvesc  : pip3 install pyvesc
- pyyaml  : pip3 install pyyaml
- VESC connected on /dev/ttyACM0
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
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return agent.heuristic_value(cfg)


def main():
    print("Initializing RC car control system with camera vision...")

    values = load_config()
    print(f"Configuration loaded: max_speed={values.max_speed}, min_speed={values.min_speed}")

    state = create_simple_state()
    print("State initialized")

    # --- Caméra OAK-D Lite (USB-C) ---
    try:
        vision = CameraVision(width=640, height=480, fps=30, use_oak=True)
        print("Camera vision initialized (OAK-D Lite)")
    except Exception as e:
        print(f"Failed to initialize camera: {e}")
        print("Make sure the OAK-D Lite is plugged in USB-C (USB3 port)")
        print("and depthai is installed: pip3 install depthai")
        return

    # --- Hardware VESC ---
    try:
        car_servo = Servo(port='/dev/ttyACM0', baudrate=115200)
        print("Servo connected via VESC serial")
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

    # Centrage servo + moteur à l'arrêt avant de démarrer
    car_servo.write(0.5)
    car_esc.write(0.0, 0.0)
    time.sleep(0.5)

    vision.start_race_timer()

    print("\nStarting control loop...")
    print("Press Ctrl+C to stop\n")

    # show_debug = False : pas d'affichage cv2, on tourne en SSH sans écran.
    # Mettre à True uniquement si un écran est branché directement sur la Jetson.
    show_debug = False
    loop_count = 0
    start_time = time.time()

    try:
        while True:
            frame = vision.read_frame()
            if frame is None:
                print("Failed to read camera frame")
                break

            result = vision.process_frame(frame, draw_debug=show_debug)

            dist_readings  = result['distance_sensors']
            current_speed  = result['speed']
            lane_offset    = result['center_offset']

            sensors = create_simple_sensors(dist_readings, current_speed)
            output  = main_loop(sensors, state, values, car_esc, car_servo, mlp_model=None)

            # Correction supplémentaire via center_offset (plus précis que les pseudo-distances)
            if abs(lane_offset) > 0.1:
                correction      = -lane_offset * 0.3
                corrected_steer = max(-1.0, min(1.0, output.steer + correction))
                car_servo.write((corrected_steer + 1.0) / 2.0)

            # Debug console toutes les 50 itérations (~1 seconde à 50Hz)
            loop_count += 1
            if loop_count % 50 == 0:
                elapsed = time.time() - start_time
                hz = loop_count / elapsed
                print(f"[{loop_count:5d}] Speed: {current_speed:5.2f} m/s | "
                      f"Pace: {result['avg_pace']:5.2f} m/s | "
                      f"Steer: {output.steer:+5.2f} | "
                      f"Offset: {lane_offset:+5.2f} | "
                      f"Accel: {output.accel:4.2f} | "
                      f"Brake: {output.brake:4.2f} | "
                      f"Hz: {hz:5.1f}")

            time.sleep(values.dt)

    except KeyboardInterrupt:
        print("\n\nStopping...")

    finally:
        print("Cleaning up...")
        car_esc.cleanup()
        car_servo.cleanup()
        vision.cleanup()
        print("RC car stopped and cleaned up")


if __name__ == '__main__':
    main()