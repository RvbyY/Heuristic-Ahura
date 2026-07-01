#!/usr/bin/env bash
set -euo pipefail

IMAGE="robocar-coord-drive:latest"
HOST_PLOT_DIR="$HOME/robocar/coord-drive/plot"
CONTAINER_APP_DIR="/app"

echo "Démarrage du container $IMAGE…"
sudo docker run --privileged \
  -e SDL_VIDEODRIVER=dummy \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  -v "$HOST_PLOT_DIR":"$CONTAINER_APP_DIR/plot" \
  "$IMAGE"
