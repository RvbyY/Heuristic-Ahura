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
from train import ESC, Servo, create_simple_state
from camera_vision import CameraVision


def load_config(config_path='configs/base_heuristic.yaml'):
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return agent.heuristic_value(cfg)


def main():
    print("Initializing RC car control system...")

    values = load_config()
    state  = create_simple_state()

    # --- Caméra OAK-D Lite ---
    try:
        vision = CameraVision(width=640, height=480, fps=30, use_oak=True)
        print("Camera OK (OAK-D Lite)")
    except Exception as e:
        print(f"Camera init failed: {e}")
        return

    # --- VESC ---
    try:
        car_servo = Servo(port='/dev/ttyACM0', baudrate=115200)
        car_esc   = ESC(port='/dev/ttyACM0', baudrate=115200)
        print("VESC OK (/dev/ttyACM0)")
    except Exception as e:
        print(f"VESC init failed: {e}")
        vision.cleanup()
        return

    car_servo.write(0.5)
    car_esc.write(0.0, 0.0)
    time.sleep(0.5)
    vision.start_race_timer()

    # --- Paramètres à ajuster ---
    # Vitesse : commence à 0.15, monte par 0.05 selon le comportement réel
    DUTY_CYCLE = 0.15

    # Sensibilité de la direction : multiplie center_offset [-1..1] → angle servo
    # Augmente si la voiture ne tourne pas assez dans les virages
    # Baisse si elle oscille trop en ligne droite
    STEER_GAIN = 0.6

    print(f"\nControl loop started. Duty: {DUTY_CYCLE:.0%}, Gain: {STEER_GAIN}. Ctrl+C to stop.\n")

    loop_count = 0
    start_time = time.time()

    try:
        while True:
            frame = vision.read_frame()
            if frame is None:
                print("Camera frame lost")
                break

            result = vision.process_frame(frame, draw_debug=False)
            lane_offset   = result['center_offset']
            current_speed = result['speed']

            # LOG TEMPORAIRE — à supprimer une fois le diagnostic fait
            print(f"offset={result['center_offset']:+.3f} | left={result['left_line']} | right={result['right_line']} | width={result['lane_width']}")

            lane_offset   = result['center_offset']   # -1.0 (trop à droite) .. +1.0 (trop à gauche)
            current_speed = result['speed']

            # --- Direction ---
            # center_offset positif = voiture trop à gauche → braquer à droite (steer positif)
            # center_offset négatif = voiture trop à droite → braquer à gauche (steer négatif)
            # On multiplie par STEER_GAIN pour doser la correction
            steer = lane_offset * STEER_GAIN
            steer = max(-1.0, min(1.0, steer))

            # Conversion [-1, 1] → [0, 1] pour le servo (0=gauche, 0.5=centre, 1=droite)
            servo_pos = (steer + 1.0) / 2.0
            car_servo.write(servo_pos)

            # --- Vitesse fixe ---
            car_esc.write(DUTY_CYCLE, 0.0)

            # Mise à jour état
            state.lap_position += current_speed * values.dt
            state.prev_steer    = steer

            # Log console toutes les ~1 seconde
            loop_count += 1
            if loop_count % 50 == 0:
                hz = loop_count / (time.time() - start_time)
                print(f"[{loop_count:5d}] "
                      f"Speed: {current_speed:5.2f} m/s | "
                      f"Offset: {lane_offset:+5.2f} | "
                      f"Steer: {steer:+5.2f} | "
                      f"Servo: {servo_pos:4.2f} | "
                      f"Hz: {hz:4.1f}")

            # time.sleep(values.dt)

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