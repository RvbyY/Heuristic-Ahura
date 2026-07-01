#!/usr/bin/env python3
import depthai as dai
import cv2
import os
import time
import argparse

# --- CONFIGURATION ---
IMG_W, IMG_H = 320, 192
FPS          = 30
SAVE_DIR     = os.path.abspath("data")

def create_pipeline():
    p = dai.Pipeline()
    cam = p.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(IMG_W, IMG_H)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(FPS)
    xout = p.createXLinkOut()
    xout.setStreamName("cam")
    cam.preview.link(xout.input)
    return p

def open_device(pipeline):
    dev = dai.Device(pipeline)
    info = dev.getDeviceInfo()
    print(f"[INFO] Device {info.getMxId()} prêt à {FPS} FPS")
    q_cam = dev.getOutputQueue("cam", maxSize=4, blocking=True)
    return dev, q_cam

def atomic_write(path, img):
    base, ext = os.path.splitext(path)
    tmp = f"{base}.tmp{ext}"
    cv2.imwrite(tmp, img)
    os.replace(tmp, path)

def main():
    parser = argparse.ArgumentParser(
        description="Capture preview VPU et mise à jour frame.png"
    )
    parser.add_argument(
        "-s", "--save",
        action="store_true",
        help="Enregistrer et mettre à jour data/frame.png"
    )
    args = parser.parse_args()

    pipeline = create_pipeline()
    bench_start = time.time()
    frame_counter = 0

    try:
        dev, q_cam = open_device(pipeline)
        while True:
            cam_pkt = q_cam.get()
            frame = cam_pkt.getCvFrame()
            if frame is None or frame.size == 0:
                print("[ERROR] frame vide reçu, arrêt.")
                break

            frame_counter += 1

            if args.save:
                frame_path = os.path.join(SAVE_DIR, "frame.png")
                atomic_write(frame_path, frame)

            if frame_counter % 30 == 0:
                elapsed = time.time() - bench_start
                fps_measured = 30 / elapsed if elapsed > 0 else float('inf')
                print(f"[BENCH] 30 frames en {elapsed:.2f}s → {fps_measured:.1f} FPS")
                bench_start = time.time()

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[INFO] Arrêt par utilisateur.")
    finally:
        try: dev.close()
        except: pass
        cv2.destroyAllWindows()
        print("[INFO] Fin du script.")

if __name__ == "__main__":
    main()
