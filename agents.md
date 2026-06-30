# Project Context for Agents

## Project

This repository adapts the ML-Ahura heuristic racing algorithm for a physical autonomous RC car running on an NVIDIA Jetson Nano developer kit.

## Hardware Context

- Compute: NVIDIA Jetson Nano developer kit.
- Camera: Luxonis OAK-D Lite by default through DepthAI; USB/CSI OpenCV capture is available with `use_oak=False`.
- Steering: Servo command sent through VESC serial, usually `/dev/ttyACM0`.
- Throttle/brake: ESC controlled with Jetson GPIO PWM, currently board pin 33.
- Typical run command on the car: `sudo python3 example_rc_car_with_camera.py`.

## Current Work

We are currently trying the camera-based driving algorithm on the physical car. The camera pipeline detects white lane boundaries, estimates speed with optical flow, and converts lane geometry into simplified distance sensor readings.

The active example is `example_rc_car_with_camera.py`, which wires together:

- `camera_vision.py` for camera frames, lane detection, speed, and simulated distances.
- `train.py` for hardware control and the main control loop.
- `agent.py` for heuristic steering, speed, safety, and state logic.
- `configs/base_heuristic.yaml` for control parameters.

## Important Data Shape

Camera vision currently returns exactly three distance readings:

```python
[left_dist, center_dist, right_dist]
```

Older heuristic code may assume a wider TORCS-style distance sensor fan. When changing control logic, keep the three-sensor RC car path explicit and avoid indexing beyond the available readings.

## Safety Notes

- Hardware control writes directly to steering and ESC. Keep fixes conservative.
- Preserve cleanup paths that center steering, neutral the ESC, close serial, release camera, and destroy OpenCV windows.
- Prefer testing algorithm changes with dummy hardware objects before running on the real car.
- Do not increase speed limits or throttle behavior without explicit operator approval.

## Useful Commands

```bash
python3 -m py_compile agent.py train.py camera_vision.py example_rc_car_with_camera.py
sudo python3 example_rc_car_with_camera.py
```

## Known Runtime Warnings

Jetson.GPIO may warn when the carrier board is not recognized as an official Jetson Developer Kit. That warning is separate from Python logic bugs, but GPIO behavior should still be validated carefully on the actual hardware.

## Camera Diagnostics

- OAK-D Lite appears in `lsusb` as VID:PID `03e7:2485` and uses DepthAI/XLink, not `/dev/video*`.
- If running over SSH or without a desktop, `DISPLAY` and `WAYLAND_DISPLAY` may be unset. Do not call `cv2.imshow()` or `cv2.waitKey()` in that mode.
- On July 1, 2026, the OAK was detected on the 480M USB2 bus. Prefer a USB3 cable/port/path for stable camera throughput.
- Camera-only speed uses optical flow and is unreliable at standstill. Do not enable stuck recovery for camera-only runs unless a better speed source is available.
- `example_rc_car_with_camera.py` defaults to a safety stop when no lane is detected. Use `--allow-no-lane` only for controlled bench testing.
- Manual driving in `../car-code/LibGamepad/pilotage.py` uses `Robocar.avancer()` from `../car-code/LibGamepad/robocar.py`, which sends `pyvesc.SetDutyCycle(int(percent * 1000))` over `/dev/ttyACM0`. The AI camera script should use the same VESC motor path, not Jetson GPIO pin 33.
- Keep autonomous VESC duty conservative during tests. The camera script uses fixed `--drive-duty-percent 8` while lane geometry is valid; manual driving used `15%`. The heuristic speed cap remains `max_speed: 3.0`.
- OAK read failures must stop the VESC before retrying or exiting. The camera script defaults to `416x320 @ 10 FPS` and `maxSize=1` output queue to reduce USB/host load.
- Camera steering should use lane geometry directly: bottom lane center offset plus lookahead/heading error. Symmetric `[left, center, right]` camera distances alone produce near-zero steering on curves.
- For autonomous steering responsiveness, optical-flow speed estimation is disabled by default; enable with `--estimate-speed` only when measuring camera speed.
- Reject saturated lane geometry (`Lane: INVALID`) instead of steering on it. The lookahead row defaults near the bottom of the ROI (`--lookahead-y-fraction 0.75`) because far/top ROI lookahead caused false hard-right steering on straight lines.
- The log `Mode` field is the best motor-state indicator: `DRIVE`, `GRACE`, or `STOP`. `Accel` is still the heuristic output, but VESC duty is overridden by fixed drive duty in camera mode.
