# Camera Vision System Documentation

## Overview

The camera vision system provides lane detection and speed estimation capabilities for the RC car. It uses computer vision techniques to:

1. **Detect lane boundaries** (two white lines)
2. **Estimate speed** using optical flow (visual odometry)
3. **Calculate average pace** over the entire race
4. **Simulate distance sensors** from lane detection

## Features

### 1. Lane Detection

Detects two white lines marking the road boundaries using an advanced algorithm:

**Algorithm Steps:**
1. **Preprocessing**: 
   - ROI (Region of Interest): Crops lower half of frame (50% from top)
   - HSV thresholding: Isolates white lines (HSV range: [0,0,200] to [180,30,255])
   - Morphological operations: Cleans up noise

2. **Detection**: 
   - Sliding window search with 8 vertical slices
   - Computes centroids in each window
   - Recenters windows based on pixel density
   - Minimum 50 pixels required to recenter

3. **Smoothing**: 
   - Polynomial fitting (degree 2) for smooth curves
   - Handles curved lanes better than straight lines

**Output:**
- Left and right lane lines (polynomial curves)
- Center offset from lane center [-1.0, 1.0]
- Lane width in pixels

**Advantages over Hough Transform:**
- Better handling of curved lanes
- More robust to noise and gaps
- Smoother lane tracking
- Adaptive to lane position changes

### 2. Speed Estimation

Uses optical flow (Farneback method) to estimate speed:
- Analyzes pixel movement between frames
- Focuses on road surface (lower part of frame)
- Converts pixel flow to meters per second
- Applies moving average for smoothing

**Calibration:**
- `pixels_per_meter`: Default 100 (adjust based on camera height)
- Use `calibrate_pixels_per_meter()` for accurate calibration

### 3. Race Timer and Pace

Tracks total distance and time:
- Start with `start_race_timer()`
- Accumulates distance traveled
- Calculates average pace (total_distance / elapsed_time)

### 4. Distance Sensor Simulation

Converts lane width to distance estimates:
- Wider lane in image = closer to camera
- Narrower lane = farther from camera
- Returns [left_dist, center_dist, right_dist] in meters

## Usage

### Basic Usage

```python
from camera_vision import CameraVision

# Initialize camera
vision = CameraVision(camera_id=0, width=640, height=480, fps=30)

# Start race timer
vision.start_race_timer()

# Main loop
while True:
    # Read frame
    frame = vision.read_frame()
    if frame is None:
        break
    
    # Process frame
    result = vision.process_frame(frame, draw_debug=True)
    
    # Get data
    speed = result['speed']
    avg_pace = result['avg_pace']
    distance_sensors = result['distance_sensors']
    center_offset = result['center_offset']
    
    # Display
    if result['frame'] is not None:
        cv2.imshow('Vision', result['frame'])
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
vision.cleanup()
```

### Integration with RC Car Control

See `example_rc_car_with_camera.py` for full integration example.

```python
from camera_vision import CameraVision
from train import ESC, Servo, main_loop, create_simple_sensors, create_simple_state

# Initialize
vision = CameraVision(camera_id=0)
car_servo = Servo()
car_esc = ESC()
state = create_simple_state()

vision.start_race_timer()

while True:
    # Get camera data
    frame = vision.read_frame()
    result = vision.process_frame(frame)
    
    # Create sensors from camera
    dist_readings = result['distance_sensors']
    current_speed = result['speed']  # or result['avg_pace']
    sensors = create_simple_sensors(dist_readings, current_speed)
    
    # Run control algorithm
    output = main_loop(sensors, state, values, car_esc, car_servo)
    
    # Optional: Apply lane offset correction
    lane_offset = result['center_offset']
    if abs(lane_offset) > 0.1:
        correction = -lane_offset * 0.3
        corrected_steer = output.steer + correction
        car_servo.write((corrected_steer + 1.0) / 2.0)
```

## Calibration

### Camera Position

For best results:
- Mount camera facing forward
- Angle slightly downward (10-20 degrees)
- Height: 10-30 cm above ground
- Ensure clear view of road ahead

### Pixels Per Meter Calibration

Method 1: Manual measurement
```python
# Drive car at known speed for known time
known_distance = 2.0  # meters
measured_flow = 200   # pixels (from optical flow)
time_delta = 1.0      # seconds

vision.calibrate_pixels_per_meter(known_distance, measured_flow, time_delta)
```

Method 2: Trial and error
- Start with default (100 pixels/meter)
- Compare camera speed with actual speed
- Adjust `vision.pixels_per_meter` accordingly

### Lane Detection Parameters

Adjust in `CameraVision.__init__()`:
- `roi_top`: Region of interest top (default: 50% from top - lower half)
- `roi_bottom`: Region of interest bottom (default: frame height)
- `n_windows`: Number of sliding windows (default: 8 slices)
- `window_margin`: Width of windows +/- margin (default: 50 pixels)
- `min_pixels`: Minimum pixels to recenter window (default: 50)
- `hsv_lower`: HSV lower bound for white lines (default: [0, 0, 200])
- `hsv_upper`: HSV upper bound for white lines (default: [180, 30, 255])
- `polyfit_degree`: Polynomial degree for smoothing (default: 2)

**HSV Thresholding for White Lines:**
- Hue: 0-180 (all colors, white has no specific hue)
- Saturation: 0-30 (low saturation for white)
- Value: 200-255 (high brightness for white)

**Sliding Window Parameters:**
- More windows (e.g., 12) = better curve tracking but slower
- Larger margin (e.g., 100) = more tolerant but less precise
- Higher min_pixels (e.g., 100) = more stable but may miss faint lines

**Polyfit Degree:**
- Degree 1: Straight lines only
- Degree 2: Smooth curves (recommended)
- Degree 3+: Very tight curves but may overfit

## Parameters

### CameraVision Constructor

```python
CameraVision(camera_id=0, width=640, height=480, fps=30)
```

- `camera_id`: Camera device ID (0 for default)
- `width`: Frame width in pixels
- `height`: Frame height in pixels
- `fps`: Target frames per second

### process_frame() Output

```python
{
    'left_line': (x1, y1, x2, y2) or None,
    'right_line': (x1, y1, x2, y2) or None,
    'center_offset': float [-1.0, 1.0],
    'lane_width': float or None,
    'speed': float (m/s),
    'avg_pace': float (m/s),
    'distance_sensors': [left, center, right] (meters),
    'frame': numpy.ndarray or None (if draw_debug=True)
}
```

## Performance

### Frame Rate
- Target: 30 fps
- Actual: Depends on hardware and processing
- Jetson Nano: ~20-30 fps at 640x480
- Jetson Xavier: ~30+ fps at 640x480

### Latency
- Processing time: ~20-40 ms per frame
- Total latency: ~50-70 ms (including camera capture)

### Accuracy

**Speed Estimation:**
- Accuracy: ±10-20% (depends on calibration)
- Better on textured surfaces
- Less accurate on smooth surfaces

**Lane Detection:**
- Works well with clear white lines
- Struggles with:
  - Faded or dirty lines
  - Strong shadows
  - Reflections
  - Curves (uses straight line approximation)

## Troubleshooting

### No Camera Found
```
RuntimeError: Failed to open camera 0
```
**Solutions:**
- Check camera connection
- Try different camera_id (0, 1, 2, ...)
- Check permissions: `ls -l /dev/video*`
- For CSI camera on Jetson: use `camera_id=0` with GStreamer pipeline

### No Lines Detected
```
left_line: None, right_line: None
```
**Solutions:**
- Adjust HSV thresholds for white detection:
  - Lower `hsv_lower[2]` (e.g., 180) for darker lines
  - Increase `hsv_upper[1]` (e.g., 50) for less pure white
- Adjust sliding window parameters:
  - Decrease `min_pixels` (e.g., 30) for faint lines
  - Increase `window_margin` (e.g., 100) for wider search
- Check lighting conditions (avoid shadows and glare)
- Ensure white lines are visible and clean
- Adjust ROI (roi_top, roi_bottom)
- Verify camera focus and exposure

### Inaccurate Speed
```
Speed fluctuates wildly or is always 0
```
**Solutions:**
- Calibrate `pixels_per_meter`
- Increase `speed_history` size for more smoothing
- Check frame rate (should be stable)
- Ensure sufficient texture on road surface
- Use `avg_pace` instead of instantaneous `speed`

### High CPU Usage
```
Frame rate drops below 20 fps
```
**Solutions:**
- Reduce frame resolution (e.g., 320x240)
- Reduce fps (e.g., 20 fps)
- Optimize ROI (smaller region)
- Use hardware acceleration if available

## Testing

### Standalone Test

Run the camera vision module directly:
```bash
cd ML-Ahura
python3 camera_vision.py
```

Controls:
- Press 'q' to quit
- Press 's' to start race timer

### With RC Car

Run the full integration example:
```bash
cd ML-Ahura
python3 example_rc_car_with_camera.py
```

Controls:
- Press 'q' to quit
- Press 'd' to toggle debug view
- Ctrl+C to stop

## Requirements

### Software
```bash
pip3 install opencv-python numpy
```

For Jetson (optimized):
```bash
# Use pre-built OpenCV with CUDA support
sudo apt-get install python3-opencv
```

### Hardware
- Camera (USB or CSI)
- Sufficient lighting
- Clear white lane markings
- Textured road surface (for speed estimation)

## Limitations

1. **Straight lines only**: Uses straight line approximation for curves
2. **Lighting dependent**: Requires good lighting conditions
3. **Surface dependent**: Speed estimation works best on textured surfaces
4. **Calibration required**: Needs calibration for accurate speed
5. **Processing overhead**: Adds ~20-40 ms latency

## Future Improvements

1. **Curve detection**: Use polynomial fitting for curved lanes
2. **Deep learning**: Use neural networks for robust lane detection
3. **Sensor fusion**: Combine with IMU/GPS for better speed estimation
4. **Adaptive thresholds**: Auto-adjust parameters based on conditions
5. **Multi-camera**: Use multiple cameras for 360° awareness
6. **Object detection**: Detect obstacles and other vehicles

## References

- OpenCV Documentation: https://docs.opencv.org/
- Optical Flow Tutorial: https://docs.opencv.org/master/d4/dee/tutorial_optical_flow.html
- Lane Detection: https://towardsdatascience.com/finding-lane-lines-simple-pipeline-for-lane-detection-d02b62e7572b
