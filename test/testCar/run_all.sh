#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$HOME/robocar/logs"
mkdir -p "$LOG_DIR"

cleanup() {
    echo
    echo "Arrêt caméra + raycast…"
    pkill -f CAM_LIVE.py 2>/dev/null || true
    rc_id="$(sg docker -c "docker ps -q --filter ancestor=robocar-raycast:latest")"
    if [[ -n "$rc_id" ]]; then
        sg docker -c "docker stop $rc_id" >/dev/null
    fi
}
trap cleanup EXIT INT TERM

echo "Démarrage caméra en arrière-plan (log: $LOG_DIR/camera.log)…"
"$SCRIPT_DIR/run_camera.sh" > "$LOG_DIR/camera.log" 2>&1 &

echo "Démarrage raycast en arrière-plan (log: $LOG_DIR/raycast.log)…"
"$SCRIPT_DIR/run_raycast.sh" > "$LOG_DIR/raycast.log" 2>&1 &

sleep 3
echo
echo "Caméra + raycast tournent en arrière-plan. Pour suivre les logs :"
echo "  tail -f $LOG_DIR/camera.log"
echo "  tail -f $LOG_DIR/raycast.log"
echo
echo "Démarrage drive en premier plan (Ctrl+C arrête tout)…"
"$SCRIPT_DIR/run_drive.sh"
