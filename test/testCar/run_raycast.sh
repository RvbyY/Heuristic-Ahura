#!/usr/bin/env bash
set -euo pipefail

IMAGE="robocar-raycast:latest"

echo "Démarrage du container $IMAGE…"

sudo docker run --rm --runtime nvidia \
     --gpus all --privileged \
     -v /dev/bus/usb:/dev/bus/usb \
     -v ~/robocar/vpu/data:/root/robocar/vpu/data \
     -v ~/robocar/camera/data:/root/robocar/camera/data \
     -e NVIDIA_VISIBLE_DEVICES=all \
     "$IMAGE"
