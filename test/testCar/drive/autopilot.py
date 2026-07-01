import os
import time
import json
import numpy as np
from collections import deque
from pyvesc.VESC import VESC

##  Const ##
ACTION_CFG_PATH = "action_config.json"
NB_RAYCAST = 40
## !Const ##

##  Helper ##
def load_config():
    with open(ACTION_CFG_PATH) as f:
        return json.load(f)
## !Helper ##

##  Global ##
cfg = load_config()
vision_history = deque(maxlen=cfg["HISTORY_SIZE"])
## !Global ##

def get_action(vis: np.ndarray, cfg: dict, is_reversing: bool):
    # --- ne pas toucher à cette fonction ---
    max_speed           = cfg["MAX_SPEED"]
    min_speed           = cfg["MIN_SPEED"]
    max_steering        = cfg["STEERING"]
    steering_threshold  = cfg["STEERING_THRESHOLD"]
    max_vision          = cfg["MAX_VISION"]
    min_vision          = cfg["MIN_VISION"]
    brake_threshold     = cfg["BRAKE_THRESHOLD"]
    brake_force         = cfg["BRAKE_FORCE"]
    start_reverse       = cfg["START_REVERSE"]
    stop_reverse        = cfg["STOP_REVERSE"]
    boost               = cfg["BOOST"]
    boost_threshold     = cfg["BOOST_THRESHOLD"]

    current_speed = steering = dist_delta = 0.0

    if vis is None:
        return np.array([[0.0, 0.0]], dtype=np.float32), is_reversing

    mean_vis = vis.mean()
    half = len(vis) // 2

    front_mean = vis[half-1:half+1].mean()
    left_mean  = vis[:half].mean()
    right_mean = vis[half:].mean()

    vision_history.append(front_mean)
    dist_delta = vision_history[0] - front_mean

    do_boost = boost if abs(dist_delta) < boost_threshold else 0.0

    if (left_mean < start_reverse or right_mean < start_reverse) and not is_reversing:
        is_reversing = True
    elif (left_mean > stop_reverse and right_mean > stop_reverse) and is_reversing:
        is_reversing = False

    if is_reversing:
        current_speed = -1.0
    else:
        if dist_delta > brake_threshold:
            current_speed = brake_force
        else:
            norm = (front_mean - min_vision) / (max_vision - min_vision)
            norm = np.clip(norm, 0.0, 1.0)
            current_speed = min_speed + norm * (max_speed - min_speed)

    if mean_vis < steering_threshold:
        x = int(vis.argmax())
        a, b = 0, len(vis)
        c, d = -max_steering, max_steering
        steering = (x - a) / (b - a) * (d - c) + c
    else:
        diff = left_mean - right_mean
        steering = np.clip(diff / (max_vision - min_vision), -1.0, 1.0) * max_steering
        steering = -steering

    if is_reversing:
        steering = steering - np.sign(steering)
        steering = np.clip(steering, -1.0, 1.0)

    current_speed += do_boost

    print(f"{mean_vis:6.1f} | speed={current_speed:6.3f} steer={steering:6.3f}")
    return np.array([[current_speed, steering]], dtype=np.float32), is_reversing
## !Algo ##

def main():
    vesc = VESC(serial_port="/dev/ttyACM0")

    is_reversing     = False
    last_cfg_mtime   = 0
    last_dist_mtime  = 0
    config           = load_config()
    last_vis         = None

    try:
        while True:
            cfg_mtime = os.path.getmtime(ACTION_CFG_PATH)
            if cfg_mtime != last_cfg_mtime:
                config = load_config()
                last_cfg_mtime = cfg_mtime
                print("🔄 Config rechargée")

            path = os.path.join(os.getcwd(), "camera", "cam_mask", "distance.txt")
            try:
                if os.path.exists(path):
                    dist_mtime = os.path.getmtime(path)
                    if dist_mtime != last_dist_mtime:
                        last_dist_mtime = dist_mtime
                        with open(path, "r") as df:
                            line = df.read().strip()

                        if not line:
                            print("⚠️ Le fichier distance.txt est vide")
                        else:
                            parts = [float(x) for x in line.split(",")]
                            if len(parts) != NB_RAYCAST:
                                print(f"⚠️ Nombre de valeurs incorrect : attendu {NB_RAYCAST}, reçu {len(parts)}")
                            else:
                                last_vis = np.array(parts, dtype=np.float32)
                                print("📊 distances raycast (mise à jour) :")
                                cols, total = 8, len(parts)
                                rows = total // cols
                                for r in range(rows):
                                    row_vals = parts[r*cols:(r+1)*cols]
                                    print(" | ".join(f"{v:6.2f}" for v in row_vals))
                                rem = total % cols
                                if rem:
                                    tail = parts[rows*cols:]
                                    print(" | ".join(f"{v:6.2f}" for v in tail))
            except Exception as e:
                print(f"⚠️ Erreur lecture distances : {e}")

            action, is_reversing = get_action(last_vis, config, is_reversing)
            speed_cmd, steer_cmd = action[0]
            steer_cmd = -steer_cmd
            steer_cmd = steer_cmd / 2 + 0.5
            speed_cmd = np.clip(speed_cmd, 0.03, 0.05)
            vesc.set_servo(steer_cmd)
            vesc.set_duty_cycle(speed_cmd)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🔴 Arrêt manuel")

    finally:
        vesc.set_duty_cycle(0)
        vesc.set_servo(0.5)
        print("✅ VESC remis à zéro")

if __name__ == "__main__":
    main()
