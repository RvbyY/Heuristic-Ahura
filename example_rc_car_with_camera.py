#!/usr/bin/env python3
"""
RC car control loop — OAK-D Lite + VESC

Requirements:
- depthai  : pip3 install depthai
- pyvesc   : pip3 install pyvesc
- pyyaml   : pip3 install pyyaml
- VESC sur /dev/ttyACM0
- OAK-D Lite branchée en USB-C (port USB3)
"""

import time
import yaml

import agent
from train import ESC, Servo, main_loop, create_simple_sensors, create_simple_state
from camera_vision import CameraVision


def load_config(config_path='configs/base_heuristic.yaml'):
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return agent.heuristic_value(cfg)


def main():
    print("Initializing RC car control system...")

    values = load_config()
    print(f"Config loaded: max_speed={values.max_speed}, min_speed={values.min_speed}")

    state = create_simple_state()

    # --- Caméra OAK-D Lite ---
    try:
        vision = CameraVision(width=640, height=480, fps=30, use_oak=True)
        print("Camera OK (OAK-D Lite)")
    except Exception as e:
        print(f"Camera init failed: {e}")
        return

    # --- VESC (servo + moteur sur le même port série) ---
    try:
        car_servo = Servo(port='/dev/ttyACM0', baudrate=115200)
        car_esc   = ESC(port='/dev/ttyACM0', baudrate=115200)
        print("VESC OK (/dev/ttyACM0)")
    except Exception as e:
        print(f"VESC init failed: {e}")
        vision.cleanup()
        return

    # Sécurité : servo centré, moteur à zéro avant de démarrer
    car_servo.write(0.5)
    car_esc.write(0.0, 0.0)
    time.sleep(0.5)

    vision.start_race_timer()

    print("\nControl loop started. Ctrl+C to stop.\n")

    loop_count  = 0
    try:
        while True:
            frame = vision.read_frame()
            if frame is None:
                print("Camera frame lost")
                break

            result = vision.process_frame(frame, draw_debug=False)

            dist_readings = result['distance_sensors']
            current_speed = result['speed']
            lane_offset   = result['center_offset']

            sensors = create_simple_sensors(dist_readings, current_speed)
            output  = main_loop(sensors, state, values, car_esc, car_servo, mlp_model=None)

            # Correction direction via center_offset caméra
            if abs(lane_offset) > 0.1:
                correction      = -lane_offset * 0.3
                corrected_steer = max(-1.0, min(1.0, output.steer + correction))
                car_servo.write((corrected_steer + 1.0) / 2.0)

            # Log console toutes les ~1 seconde
            loop_count += 1
            if loop_count % 50 == 0:
                hz = loop_count / (time.time() - start_time)
                print(f"[{loop_count:5d}] "
                      f"Speed: {current_speed:5.2f} m/s | "
                      f"Steer: {output.steer:+5.2f} | "
                      f"Offset: {lane_offset:+5.2f} | "
                      f"Accel: {output.accel:4.2f} | "
                      f"Hz: {hz:4.1f}")

            time.sleep(values.dt)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        print("Cleaning up...")
        car_esc.cleanup()
        car_servo.cleanup()
        vision.cleanup()
        print("Done.")


if __name__ == '__main__':
    main()