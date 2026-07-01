#!/usr/bin/env bash
set -euo pipefail

IMAGE="robocar-drive:latest"
HOST_CAM_DIR="$HOME/robocar/camera"
CONTAINER_APP_DIR="/app"

echo "Démarrage du container $IMAGE…"
sudo docker run --privileged \
  -e SDL_VIDEODRIVER=dummy \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  -v ~/robocar/ai-drive/model:/app/model \
  -v ~/robocar/ai-drive/data:/app/data \
  -v "$HOST_CAM_DIR":"$CONTAINER_APP_DIR/camera":ro \
  "$IMAGE"
