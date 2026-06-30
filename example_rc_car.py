#!/usr/bin/env python3
"""
Example script showing how to use the adapted agent.py with RC car hardware.

This demonstrates:
1. Loading configuration from YAML
2. Creating simplified sensors for RC car
3. Running the control loop with GPIO/ESC and Servo control
4. Proper cleanup on exit

Requirements:
- Distance sensors (e.g., ultrasonic or lidar)
- Speed estimation (e.g., from encoder or IMU)
- Jetson GPIO for ESC control
- VESC serial connection for servo control
"""

import sys
import os
import time
import yaml

import agent
from train import ESC, Servo, main_loop, create_simple_sensors, create_simple_state


def load_config(config_path='configs/base_heuristic.yaml'):
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return agent.heuristic_value(cfg)


def get_distance_sensors():
    """
    Placeholder function to read distance sensors.

    Replace this with actual sensor reading code.
    Should return a list of distance readings.

    For example, with 3 sensors (left, center, right):
    - dist[0] = right sensor
    - dist[1] = center sensor
    - dist[2] = left sensor

    Returns:
        list: Distance readings in meters (or consistent units)
    """
    # TODO: Replace with actual sensor reading
    # Example: return [read_left_sensor(), read_center_sensor(), read_right_sensor()]
    return [2.0, 3.0, 2.0]  # Dummy values


def get_current_speed(output, ):
    """
    Placeholder function to estimate current speed.

    Replace this with actual speed estimation code.
    Can use:
    - Wheel encoder
    - IMU integration
    - GPS speed

    Returns:
        float: Current speed in m/s (or consistent units)
    """
    # TODO: Replace with actual speed estimation
    # Example: return calculate_speed_from_encoder()
    return 0.0  # Dummy value


def main():
    """Main control loop for RC car."""
    print("Initializing RC car control system...")
    values = load_config()
    print(f"Configuration loaded: max_speed={values.max_speed}, min_speed={values.min_speed}")
    state = create_simple_state()
    print("State initialized")

    try:
        car_servo = Servo(port='/dev/ttyACM0', baudrate=115200)
        print("Servo connected via VESC")
        car_esc = ESC(pin=33, frequency=50)
        print("ESC initialized on GPIO pin 33")
    except Exception as e:
        print(f"Failed to initialize hardware: {e}")
        print("Make sure:")
        print("  - VESC is connected to /dev/ttyACM0")
        print("  - Jetson.GPIO is installed")
        print("  - Running with appropriate permissions")
        return

    car_servo.write(0.5)
    car_esc.write(0.0, 0.0)
    time.sleep(0.5)
    print("\nStarting control loop...")
    print("Press Ctrl+C to stop\n")

    try:
        loop_count = 0
        start_time = time.time()
        while True:
            dist_readings = get_distance_sensors()
            current_speed = get_current_speed()
            sensors = create_simple_sensors(dist_readings, current_speed)
            output = main_loop(sensors, state, values, car_esc, car_servo, mlp_model=None)
            loop_count += 1
            if loop_count % 50 == 0:
                elapsed = time.time() - start_time
                hz = loop_count / elapsed
                print(f"[{loop_count:5d}] Speed: {current_speed:5.2f} | "
                      f"Steer: {output.steer:+5.2f} | "
                      f"Accel: {output.accel:4.2f} | "
                      f"Brake: {output.brake:4.2f} | "
                      f"Hz: {hz:5.1f}")
            # Control loop timing (50Hz = 0.02s)
            time.sleep(values.dt)

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        print("Cleaning up...")
        car_esc.cleanup()
        car_servo.cleanup()
        print("RC car stopped and cleaned up")


if __name__ == '__main__':
    main()