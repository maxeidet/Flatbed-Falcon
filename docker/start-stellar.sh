#!/usr/bin/env bash
# Starts PX4 + Gazebo + the DDS agent on a Stellar (Ubuntu) computer, using the
# rootless Docker daemon you already have there. No AppleScript, no second
# terminal window needed: the container runs in the background and PX4 runs in
# THIS terminal (you get the pxh> shell). Open another terminal for the backend:
#   ./ros2_ws/run.sh
#
# Default: --native-gui --gpu -w hive_base (Gazebo window on this computer's own
# screen, rendered on the GPU).
#
# Usage:
#   ./start-stellar.sh                        # = --native-gui --gpu -w hive_base, drone gz_x500_gimbal
#   ./start-stellar.sh -m gz_x500_depth       # drone with a depth camera
#   ./start-stellar.sh -w baylands            # another world
#   ./start-stellar.sh --vnc                  # GUI through a browser on this computer instead
#   ./start-stellar.sh --headless             # no GUI, no ports published
#   ./start-stellar.sh --no-gpu               # do not give the container the GPU(s), see below
#
# --native-gui (default): the Gazebo window opens on the screen you are sitting at
#   (course students have no remote access, so we always are). No VNC involved.
# --vnc: open http://localhost:6080/vnc.html in a browser ON THIS COMPUTER. Ports are
#   bound to 127.0.0.1 only and protected by a random password printed below. Another
#   user may already hold 6080/5900 on a shared computer, so they can be overridden:
#     HOST_NOVNC_PORT=6081 HOST_VNC_PORT=5901 ./start-stellar.sh --vnc
# The last of --native-gui / --vnc / --headless on the command line wins.
#
# GPU (default, --gpu): passes `--device nvidia.com/gpu=all` (Stellar rootless Docker
# guide), which gives the container ALL GPUs in the computer, and prints nvidia-smi
# first: check that nobody else is using the GPU (Stellar rules). --no-gpu turns it
# off. Without an NVIDIA driver on the computer the script falls back to --no-gpu.
# The VNC display is software-rendered, so Gazebo only renders on the GPU with
# --native-gui.
# Untested on Stellar until tried: while the GUI is up, `nvidia-smi` on the host
# should list gz as a graphics (G) process. If it does not, rendering is still software.
#
# Stop: Ctrl+C or type `shutdown` at the pxh> prompt. The container is removed
# automatically. Agent logs: docker logs -f flatbed-falcon-sim
#
# See README.md in this folder for all valid PX4_SIM_MODEL and world values, and
# README-stellar.md for the full Stellar guide (including cleanup after a session).
set -e

MODEL="gz_x500_gimbal"
WORLD="hive_base"
GUI_MODE="native"   # native | vnc | headless
USE_GPU=1          # --no-gpu sets 0
GPU_FLAG_GIVEN=0
HOST_NOVNC_PORT="${HOST_NOVNC_PORT:-6080}"
HOST_VNC_PORT="${HOST_VNC_PORT:-5900}"

while [ $# -gt 0 ]; do
  case "$1" in
    -m|--model)
      MODEL="$2"
      shift 2
      ;;
    -w|--world)
      WORLD="$2"
      shift 2
      ;;
    --native-gui)
      GUI_MODE="native"
      shift
      ;;
    --vnc)
      GUI_MODE="vnc"
      shift
      ;;
    --headless)
      GUI_MODE="headless"
      shift
      ;;
    --gpu)
      USE_GPU=1
      GPU_FLAG_GIVEN=1
      shift
      ;;
    --no-gpu)
      USE_GPU=0
      shift
      ;;
    -h|--help)
      grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "Unknown flag: $1"
      exit 1
      ;;
  esac
done

if [ "$(uname -s)" != "Linux" ]; then
  echo "start-stellar.sh is for the Stellar (Ubuntu) computers. On macOS use start-dev.sh."
  exit 1
fi

if [ "$GUI_MODE" = "native" ] && [ -z "${DISPLAY:-}" ]; then
  echo "--native-gui (the default) needs a graphical session on this computer, but DISPLAY is not set."
  echo "Use --vnc (browser on this computer) or --headless (no GUI) instead."
  exit 1
fi

if [ "$USE_GPU" = "1" ] && ! command -v nvidia-smi >/dev/null 2>&1; then
  if [ "$GPU_FLAG_GIVEN" = "1" ]; then
    echo "nvidia-smi not found: no NVIDIA driver on this computer, so --gpu cannot work here."
    exit 1
  fi
  echo "nvidia-smi not found: no NVIDIA driver on this computer, running without the GPU."
  USE_GPU=0
fi

cd "$(dirname "$0")"
REPO_ROOT="$(cd .. && pwd)"

if ! docker info >/dev/null 2>&1; then
  echo "Cannot reach your Docker daemon. Try: systemctl --user start docker"
  echo "(or log out and in again). See the Stellar rootless Docker guide."
  exit 1
fi

if [ "$USE_GPU" = "1" ]; then
  echo "GPU status (the Stellar rules say to check that nobody else is using it):"
  nvidia-smi
  echo "Note: --device nvidia.com/gpu=all gives the container ALL GPUs in this computer."
  echo ""
fi

echo "Building image (flatbed-falcon-px4)..."
docker build -t flatbed-falcon-px4 .

docker rm -f flatbed-falcon-sim >/dev/null 2>&1 || true

cleanup() {
  echo ""
  echo "Stopping container..."
  docker rm -f flatbed-falcon-sim >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# Classic VNC auth only uses the first 8 characters. Exported and passed with
# `-e VNC_PASSWORD` (no value) so it never shows up in the process list.
export VNC_PASSWORD="$(LC_ALL=C tr -dc 'A-Za-z0-9' </dev/urandom | head -c 8)"

PORT_ARGS=()
GUI_ARGS=()
GPU_ARGS=()

if [ "$GUI_MODE" = "native" ]; then
  # Draw on this computer's own X display. Container root is our own user under
  # rootless Docker, so the display's cookie file is readable.
  GUI_ARGS=(-e "DISPLAY=${DISPLAY}" -e NO_VNC=1 -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix)
  XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
  if [ -f "$XAUTH_FILE" ]; then
    GUI_ARGS+=(-e XAUTHORITY=/tmp/.Xauthority -v "${XAUTH_FILE}":/tmp/.Xauthority:ro)
  fi
elif [ "$GUI_MODE" = "vnc" ]; then
  PORT_ARGS=(-p "127.0.0.1:${HOST_NOVNC_PORT}:6080" -p "127.0.0.1:${HOST_VNC_PORT}:5900")
fi

if [ "$USE_GPU" = "1" ]; then
  GPU_ARGS=(--device nvidia.com/gpu=all)
fi

if ! docker run -d --rm --name flatbed-falcon-sim \
    "${PORT_ARGS[@]}" "${GUI_ARGS[@]}" "${GPU_ARGS[@]}" \
    -e VNC_PASSWORD \
    -v "${REPO_ROOT}":/workspace \
    flatbed-falcon-px4 MicroXRCEAgent udp4 -p 8888 >/dev/null; then
  echo "Could not start the container. If a port is taken by another user, try:"
  echo "  HOST_NOVNC_PORT=6081 HOST_VNC_PORT=5901 $0 --vnc"
  exit 1
fi

for _ in $(seq 1 30); do
  [ "$(docker inspect -f '{{.State.Running}}' flatbed-falcon-sim 2>/dev/null)" = "true" ] && break
  sleep 1
done

if [ "$USE_GPU" = "1" ]; then
  echo ""
  echo "GPU inside the container:"
  docker exec flatbed-falcon-sim nvidia-smi -L \
    || echo "  WARNING: no GPU visible inside the container. Is CDI set up for your rootless Docker? See the Stellar guide."
fi

echo ""
if [ "$GUI_MODE" = "native" ]; then
  echo "  GUI: the Gazebo window opens on this computer's own screen (no VNC)."
elif [ "$GUI_MODE" = "vnc" ]; then
  echo "  GUI (browser on this computer):  http://localhost:${HOST_NOVNC_PORT}/vnc.html"
  echo "  VNC client:                      vnc://127.0.0.1:${HOST_VNC_PORT}"
  echo "  VNC password:                    ${VNC_PASSWORD}"
else
  echo "  Headless: no GUI and no ports published."
fi
echo "  Backend (in a second terminal):  ${REPO_ROOT}/ros2_ws/run.sh"
echo ""
echo "Starting PX4 + Gazebo (model: ${MODEL}, world: ${WORLD}). Ctrl+C or 'shutdown' stops everything."

HEADLESS_ENV=""
if [ "$GUI_MODE" = "headless" ]; then
  HEADLESS_ENV="HEADLESS=1 "
fi

docker exec -it flatbed-falcon-sim bash -c "cp /workspace/worlds/*.sdf /opt/px4-gazebo/share/gz/worlds/ 2>/dev/null || true; export ${HEADLESS_ENV}PX4_GZ_WORLD=${WORLD} PX4_SIM_MODEL=${MODEL} PX4_GZ_MODEL_POSE=0,0,0,0,0,0; /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0"
