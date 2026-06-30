# Autonomous RC Car Overview

## Purpose

This project runs a heuristic autonomous-driving algorithm on a physical RC car using an NVIDIA Jetson Nano. The current focus is testing camera-based lane following with real steering and throttle hardware.

## Main Hardware

- Compute: NVIDIA Jetson Nano.
- Camera: Luxonis OAK-D Lite over USB-C, accessed through DepthAI/XLink.
- Steering: servo command sent through the VESC serial port, usually `/dev/ttyACM0`.
- Throttle/brake: motor duty command sent through the same VESC serial port.

The OAK-D Lite is not a normal webcam and usually does not appear as `/dev/video*`. It should appear in `lsusb` as `03e7:2485`.

## Main Software Files

- `example_rc_car_with_camera.py`: main hardware run script with camera vision.
- `camera_vision.py`: reads camera frames, detects lane lines, estimates speed, and creates distance readings.
- `train.py`: connects the algorithm output to servo and ESC hardware.
- `agent.py`: heuristic steering, speed, safety, and state logic.
- `configs/base_heuristic.yaml`: driving parameters such as speed limits, timing, and control constants.

## Camera Pipeline

The camera system detects two white lane boundary lines in the lower half of the image. From those lines it computes:

- lane center offset,
- lane width,
- estimated speed from optical flow,
- simplified distance readings: `[left, center, right]`.

These three distance readings are passed to the driving algorithm. For this RC car path, steering uses the simple three-sensor method instead of the older wider simulator sensor fan.

## Control Pipeline

Each loop:

1. Read a frame from the OAK-D Lite.
2. Detect lane boundaries and estimate speed.
3. Convert camera data into sensor values.
4. Compute steering, acceleration, and brake.
5. Send steering to the VESC servo command.
6. Send throttle/brake to the VESC `SetDutyCycle` motor command.

The script also supports headless runs. If no `DISPLAY` or `WAYLAND_DISPLAY` is available, it skips OpenCV preview windows and runs from the terminal.

## Safety Behavior

- If no lane is detected, `example_rc_car_with_camera.py` stops by default: centered steering, neutral throttle, no brake.
- Camera-only runs disable stuck recovery because optical-flow speed is unreliable at standstill.
- Use `--allow-no-lane` only for controlled bench tests, preferably with the wheels lifted.
- The camera run uses the same VESC motor command as manual `pilotage.py`: `SetDutyCycle(int(percent * 1000))`.
- The default maximum VESC duty is `5%`, matching the lowest manual motor test that should overcome motor deadband. The algorithm speed cap remains `max_speed: 3.0`.
- The default camera FPS is `15` to reduce OAK USB load, especially if the camera is running through USB2.
- If camera reads fail, the script centers steering, sends zero VESC motor duty, and exits cleanly after repeated misses or a DepthAI error.
- Cleanup centers steering, sends zero VESC motor duty, closes the camera, and releases OpenCV resources.

## Useful Commands

Run the car with camera lane detection:

```bash
sudo python3 example_rc_car_with_camera.py
```

Bench-test without requiring visible lane lines:

```bash
sudo python3 example_rc_car_with_camera.py --allow-no-lane
```

Use a different VESC duty limit if needed:

```bash
sudo python3 example_rc_car_with_camera.py --max-duty-percent 10
```

Lower camera load if the OAK is unstable:

```bash
sudo python3 example_rc_car_with_camera.py --camera-fps 10
```

Check Python syntax after edits:

```bash
python3 -m py_compile agent.py train.py camera_vision.py example_rc_car_with_camera.py
```

Inspect USB devices:

```bash
lsusb
lsusb -t
```

For stable camera throughput, prefer a USB3 cable and port. If the OAK appears on a `480M` bus in `lsusb -t`, it is running through USB2.
