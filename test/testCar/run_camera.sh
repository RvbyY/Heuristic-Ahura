#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAM_SCRIPT="$SCRIPT_DIR/camera/CAM_LIVE.py"
RUN_DIR="$HOME/robocar/vpu"

mkdir -p "$RUN_DIR/data"

echo "Démarrage de la capture caméra (frame.png -> $RUN_DIR/data)…"
cd "$RUN_DIR"
python3 "$CAM_SCRIPT" --save
