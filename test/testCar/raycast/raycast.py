#!/usr/bin/env python3
import cv2
import numpy as np
import math
import time
import os
import argparse

try:
    from numba import njit
    NUMBA = True
except ImportError:
    NUMBA = False

# --- CONFIGURATION ---
IMG_PATH       = os.path.expanduser("~/robocar/vpu/data/frame.png")
OUTPUT_DIR     = os.path.expanduser("~/robocar/camera/data")
OUTPUT_FILE    = os.path.join(OUTPUT_DIR, "distance.txt")
ANNOT_IMG_FILE = os.path.join(OUTPUT_DIR, "annotated.png")

NUMBER_RAY     = 60
FOV_DEG        = 180
STEP_SIZE      = 1.0
THRESHOLD      = 200
FPS_TARGET     = 30.0
RETRIES        = 3
RETRY_DELAY    = 0.01
CROP_RATIO     = 0.0
_start_angle = math.radians(90 - FOV_DEG / 2)
_angle_step  = math.radians(FOV_DEG / (NUMBER_RAY - 1))
DELTAS = [
    (math.cos(_start_angle + k * _angle_step) * STEP_SIZE,
     math.sin(_start_angle + k * _angle_step) * STEP_SIZE)
    for k in range(NUMBER_RAY)
]


def atomic_write(path: str, data: bytes):
    """Atomic write of a binary/text file."""
    base, ext = os.path.splitext(path)
    tmp = f"{base}.tmp{ext}"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def load_frame(path, retries=RETRIES, delay=RETRY_DELAY):
    """Try multiple reads to avoid partial files."""
    for _ in range(retries):
        img = cv2.imread(path)
        if img is not None and img.size > 0:
            return img
        time.sleep(delay)
    return None


if NUMBA:
    @njit
    def _raycast_loop(mask, deltas, h, w):
        out = np.zeros(len(deltas), np.int32)
        cx, cy = w * 0.5, h - 1.0
        for k in range(len(deltas)):
            dx, dy = deltas[k]
            x, y = cx, cy
            dist = 0
            while True:
                xi = int(x + 0.5)
                yi = int(y + 0.5)
                if xi < 0 or xi >= w or yi < 0 or yi >= h:
                    out[k] = dist
                    break
                if mask[yi, xi]:
                    out[k] = dist
                    break
                x += dx; y -= dy; dist += 1
        return out


def get_raycasts(image: np.ndarray) -> np.ndarray:
    """Return a 1D array of distances."""
    h, w = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = gray > THRESHOLD

    crop_h = int(h * CROP_RATIO)
    if crop_h > 0:
        mask[:crop_h, :] = False

    if NUMBA:
        return _raycast_loop(mask.astype(np.uint8),
                             np.array(DELTAS), h, w)

    distances = []
    for dx, dy in DELTAS:
        x, y = w * 0.5, h - 1.0
        dist = 0
        while True:
            xi, yi = int(x + 0.5), int(y + 0.5)
            if xi < 0 or xi >= w or yi < 0 or yi >= h:
                distances.append(dist)
                break
            if mask[yi, xi]:
                distances.append(dist)
                break
            x += dx; y -= dy; dist += 1
    return np.array(distances, dtype=np.int32)


def annotate_image(image: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Draw rays from bottom-center to contact points."""
    h, w = image.shape[:2]
    cx, cy = int(w * 0.5), int(h - 1.0)
    annotated = image.copy()
    for k, (dx, dy) in enumerate(DELTAS):
        dist = int(distances[k])
        end_x = cx + int(dx * dist + 0.5)
        end_y = cy - int(dy * dist + 0.5)
        cv2.line(annotated, (cx, cy), (end_x, end_y), (0, 0, 255), 1)
    return annotated


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    parser = argparse.ArgumentParser(
        description="Raycast at 30 FPS: frame.png -> distance.txt"
    )
    parser.add_argument(
        "-a", "--annotate",
        action="store_true",
        help="Generate annotated.png with rays drawn"
    )
    args = parser.parse_args()

    interval = 1.0 / FPS_TARGET
    print(f"[INFO] Reading {IMG_PATH} at {FPS_TARGET:.1f} FPS")

    while True:
        t0 = time.time()
        img = load_frame(IMG_PATH)
        if img is None:
            print(f"[WARN] Cannot load {IMG_PATH}, skipping")
            time.sleep(interval)
            continue

        dists = get_raycasts(img)

        text = ",".join(map(str, dists)).encode("utf-8")
        atomic_write(OUTPUT_FILE, text)

        if args.annotate:
            ann = annotate_image(img, dists)
            success, buf = cv2.imencode(".png", ann)
            if success:
                atomic_write(ANNOT_IMG_FILE, buf.tobytes())
            else:
                print("[ERROR] Failed to encode annotated image")

        elapsed = time.time() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)


if __name__ == "__main__":
    main()
