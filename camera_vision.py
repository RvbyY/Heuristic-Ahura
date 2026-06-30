#!/usr/bin/env python3
"""
Camera-based vision system for RC car.

Features:
1. Lane detection (two white lines as road delimiters)
2. Speed estimation using visual odometry
3. Distance sensor simulation from camera

Requirements:
- OpenCV (cv2)
- NumPy
- Camera (USB or CSI camera on Jetson)
"""

import cv2  # type: ignore[import-not-found]
import numpy as np
import time
from collections import deque


class CameraVision:
    """
    Camera-based vision system for lane detection and speed estimation.
    """
    def __init__(self, camera_id=0, width=640, height=480, fps=30):
        """
        Initialize camera vision system.

        Args:
            camera_id: Camera device ID (0 for default camera)
            width: Frame width in pixels
            height: Frame height in pixels
            fps: Target frames per second
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_id}")
        # Lane detection parameters
        self.roi_top = int(height * 0.5)  # Region of interest top (50% from top - lower half)
        self.roi_bottom = height
        # Sliding window parameters
        self.n_windows = 8  # Number of sliding windows (8 slices)
        self.window_margin = 50  # Width of the windows +/- margin
        self.min_pixels = 50  # Minimum number of pixels to recenter window
        # HSV thresholding parameters for white lines
        self.hsv_lower = np.array([0, 0, 200])  # Lower bound for white
        self.hsv_upper = np.array([180, 30, 255])  # Upper bound for white
        # Polyfit parameters
        self.polyfit_degree = 2  # Degree 2 polynomial for smoothing
        # Speed estimation
        self.prev_frame = None
        self.prev_time = None
        self.speed_history = deque(maxlen=10)  # Moving average
        self.pixels_per_meter = 100  # Calibration: pixels per meter (adjust based on camera height)
        # Race timer for pace calculation
        self.race_start_time = None
        self.total_distance = 0.0  # meters
        print(f"Camera initialized: {width}x{height} @ {fps}fps")

    def start_race_timer(self):
        """Start the race timer for pace calculation."""
        self.race_start_time = time.time()
        self.total_distance = 0.0
        print("Race timer started")

    def detect_lanes(self, frame):
        """
        Detect two white lines (lane boundaries) using sliding window method.

        Algorithm:
        1. Preprocessing: ROI (crop lower half) + HSV thresholding
        2. Detection: Sliding window with 8 slices and centroids
        3. Smoothing: Polyfit degree 2

        Args:
            frame: BGR image from camera

        Returns:
            tuple: (left_line, right_line, center_offset, lane_width)
                - left_line: (x1, y1, x2, y2) or None
                - right_line: (x1, y1, x2, y2) or None
                - center_offset: offset from center [-1.0, 1.0]
                - lane_width: width in pixels or None
        """
        roi_frame = frame[self.roi_top:self.roi_bottom, :]
        roi_height = roi_frame.shape[0]
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        left_lane_inds, right_lane_inds = self._sliding_window_search(mask)

        if len(left_lane_inds) == 0 and len(right_lane_inds) == 0:
            return None, None, 0.0, None
        nonzero = mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_fit = None
        right_fit = None

        if len(left_lane_inds) > 0:
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            if len(leftx) >= 3:  # Need at least 3 points for degree 2
                left_fit = np.polyfit(lefty, leftx, self.polyfit_degree)
        if len(right_lane_inds) > 0:
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
            if len(rightx) >= 3:  # Need at least 3 points for degree 2
                right_fit = np.polyfit(righty, rightx, self.polyfit_degree)
        left_line = None
        right_line = None

        if left_fit is not None:
            y1 = 0
            y2 = roi_height - 1
            x1 = int(np.polyval(left_fit, y1))
            x2 = int(np.polyval(left_fit, y2))
            left_line = (x1, y1 + self.roi_top, x2, y2 + self.roi_top)
        if right_fit is not None:
            y1 = 0
            y2 = roi_height - 1
            x1 = int(np.polyval(right_fit, y1))
            x2 = int(np.polyval(right_fit, y2))
            right_line = (x1, y1 + self.roi_top, x2, y2 + self.roi_top)
        center_offset = 0.0
        lane_width = None

        if left_line is not None and right_line is not None:
            left_x = left_line[2]  # x2 (bottom)
            right_x = right_line[2]  # x2 (bottom)
            lane_center = (left_x + right_x) / 2
            frame_center = self.width / 2
            center_offset = (lane_center - frame_center) / (self.width / 2)
            center_offset = np.clip(center_offset, -1.0, 1.0)
            lane_width = right_x - left_x
        return left_line, right_line, center_offset, lane_width

    def _sliding_window_search(self, binary_warped):
        """
        Sliding window search for lane pixels.

        Args:
            binary_warped: Binary image (thresholded)

        Returns:
            tuple: (left_lane_inds, right_lane_inds) - indices of lane pixels
        """
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        window_height = int(binary_warped.shape[0] / self.n_windows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.n_windows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - self.window_margin
            win_xleft_high = leftx_current + self.window_margin
            win_xright_low = rightx_current - self.window_margin
            win_xright_high = rightx_current + self.window_margin
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                             (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.min_pixels:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.min_pixels:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        return left_lane_inds, right_lane_inds

    def estimate_speed(self, frame):
        """
        Estimate speed using optical flow (visual odometry).

        Args:
            frame: Current BGR frame

        Returns:
            float: Estimated speed in m/s
        """
        current_time = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_frame is None or self.prev_time is None:
            self.prev_frame = gray
            self.prev_time = current_time
            return 0.0

        flow = cv2.calcOpticalFlowFarneback(
            self.prev_frame,
            gray,
            None,
            pyr_scale=0.5,
            levels=3,
            winsize=15,
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=0
        )
        roi_flow = flow[self.roi_top:self.roi_bottom, :]
        avg_flow_y = np.mean(roi_flow[:, :, 1])
        dt = current_time - self.prev_time

        if dt > 0:
            pixels_per_sec = abs(avg_flow_y) / dt
            speed = pixels_per_sec / self.pixels_per_meter
            self.speed_history.append(speed)
            if self.race_start_time is not None:
                self.total_distance += speed * dt
        else:
            speed = 0.0
        self.prev_frame = gray
        self.prev_time = current_time
        if len(self.speed_history) > 0:
            return np.mean(self.speed_history)
        return 0.0

    def get_average_pace(self):
        """
        Get average pace since race start.

        Returns:
            float: Average speed in m/s, or 0.0 if race not started
        """
        if self.race_start_time is None:
            return 0.0
        elapsed_time = time.time() - self.race_start_time

        if elapsed_time > 0 and self.total_distance > 0:
            return self.total_distance / elapsed_time
        return 0.0

    def get_distance_sensors_from_lanes(self, left_line, right_line, lane_width):
        """
        Simulate distance sensors from lane detection.

        Args:
            left_line: Left lane line (x1, y1, x2, y2) or None
            right_line: Right lane line (x1, y1, x2, y2) or None
            lane_width: Lane width in pixels or None

        Returns:
            list: [left_dist, center_dist, right_dist] in meters
        """
        default_dist = 4.0

        if lane_width is None:
            return [default_dist, default_dist, default_dist]
        # Convert lane width to distance estimate
        # Wider lane in image = closer to camera
        # Narrower lane = farther from camera
        # This is a simplified model
        # Calibration: assume lane width is ~1 meter in real world
        # and appears as ~200 pixels when 2 meters away
        reference_width = 200  # pixels at 2 meters
        reference_dist = 2.0   # meters
        # Inverse relationship: dist = reference_dist * (reference_width / lane_width)
        center_dist = reference_dist * (reference_width / max(lane_width, 1))
        center_dist = np.clip(center_dist, 0.5, 4.0)
        # Left and right distances based on line presence
        left_dist = center_dist if left_line is not None else default_dist
        right_dist = center_dist if right_line is not None else default_dist
        return [left_dist, center_dist, right_dist]

    def read_frame(self):
        """
        Read a frame from the camera.

        Returns:
            numpy.ndarray: BGR frame, or None if failed
        """
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def process_frame(self, frame, draw_debug=False):
        """
        Process a frame: detect lanes and estimate speed.

        Args:
            frame: BGR frame from camera
            draw_debug: If True, draw debug visualization on frame

        Returns:
            dict: {
                'left_line': left lane line or None,
                'right_line': right lane line or None,
                'center_offset': offset from center [-1.0, 1.0],
                'lane_width': lane width in pixels or None,
                'speed': estimated speed in m/s,
                'avg_pace': average pace since race start,
                'distance_sensors': [left, center, right] distances in meters,
                'frame': processed frame (if draw_debug=True)
            }
        """
        left_line, right_line, center_offset, lane_width = self.detect_lanes(frame)
        speed = self.estimate_speed(frame)
        avg_pace = self.get_average_pace()
        distance_sensors = self.get_distance_sensors_from_lanes(left_line, right_line, lane_width)

        if draw_debug:
            debug_frame = frame.copy()
            cv2.rectangle(debug_frame, (0, self.roi_top), (self.width, self.roi_bottom), (0, 255, 0), 2)
            if left_line is not None:
                x1, y1, x2, y2 = left_line
                cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            if right_line is not None:
                x1, y1, x2, y2 = right_line
                cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            frame_center = self.width // 2
            cv2.line(debug_frame, (frame_center, 0), (frame_center, self.height), (255, 0, 0), 2)
            if lane_width is not None:
                lane_center = int(frame_center + center_offset * (self.width / 2))
                cv2.line(debug_frame, (lane_center, 0), (lane_center, self.height), (0, 255, 255), 2)
            cv2.putText(debug_frame, f"Speed: {speed:.2f} m/s", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_frame, f"Avg Pace: {avg_pace:.2f} m/s", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_frame, f"Offset: {center_offset:+.2f}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_frame, f"Dist: L={distance_sensors[0]:.1f} C={distance_sensors[1]:.1f} R={distance_sensors[2]:.1f}",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            frame = debug_frame

        return {
            'left_line': left_line,
            'right_line': right_line,
            'center_offset': center_offset,
            'lane_width': lane_width,
            'speed': speed,
            'avg_pace': avg_pace,
            'distance_sensors': distance_sensors,
            'frame': frame if draw_debug else None
        }

    def calibrate_pixels_per_meter(self, known_distance_meters, measured_flow_pixels, time_delta):
        """
        Calibrate the pixels_per_meter parameter.

        Args:
            known_distance_meters: Known distance traveled in meters
            measured_flow_pixels: Measured optical flow in pixels
            time_delta: Time taken in seconds
        """
        if time_delta > 0:
            speed_mps = known_distance_meters / time_delta
            pixels_per_sec = measured_flow_pixels / time_delta
            self.pixels_per_meter = pixels_per_sec / speed_mps
            print(f"Calibrated: {self.pixels_per_meter:.2f} pixels/meter")

    def cleanup(self):
        """Release camera resources."""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        print("Camera released")


if __name__ == '__main__':
    print("Camera Vision Test")
    print("Press 'q' to quit, 's' to start race timer")
    try:
        vision = CameraVision(camera_id=0, width=640, height=480, fps=30)
        while True:
            frame = vision.read_frame()
            if frame is None:
                print("Failed to read frame")
                break
            result = vision.process_frame(frame, draw_debug=True)
            if result['frame'] is not None:
                cv2.imshow('Camera Vision', result['frame'])
            print(f"Speed: {result['speed']:.2f} m/s | "
                  f"Pace: {result['avg_pace']:.2f} m/s | "
                  f"Offset: {result['center_offset']:+.2f} | "
                  f"Sensors: {result['distance_sensors']}")
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                vision.start_race_timer()
    finally:
        vision.cleanup()
