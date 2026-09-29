#!/usr/bin/env bash
set -e

# ============================================================
# VNC + noVNC startup for Linux Docker container
# ============================================================

DISPLAY="${DISPLAY:-:0}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PASSWORD="${VNC_PASSWORD:-falcon12}"

echo "Starting Xvfb on ${DISPLAY}..."

Xvfb "${DISPLAY}" \
    -screen 0 1920x1080x24 \
    -ac \
    +extension GLX \
    +render \
    -noreset &

sleep 1

echo "Starting Fluxbox..."

fluxbox &

sleep 1

echo "Starting x11vnc on port ${VNC_PORT}..."

x11vnc \
    -display "${DISPLAY}" \
    -forever \
    -shared \
    -passwd "${VNC_PASSWORD}" \
    -rfbport "${VNC_PORT}" \
    -bg \
    -o /var/log/x11vnc.log

echo "Starting noVNC on port ${NOVNC_PORT}..."

websockify \
    --web=/usr/share/novnc \
    "${NOVNC_PORT}" \
    "localhost:${VNC_PORT}" &

echo
echo "========================================"
echo " VNC environment started"
echo "========================================"
echo
echo "Display: ${DISPLAY}"
echo "VNC:     localhost:${VNC_PORT}"
echo "noVNC:   http://localhost:${NOVNC_PORT}"
echo
echo "Starting application:"
echo "  $*"
echo

# Pass the original command to the PX4/ROS 2 entrypoint.
exec /usr/local/bin/ros2-entrypoint.sh "$@"