#!/usr/bin/env bash
# Replays a Gazebo log saved with `start-dev.sh --record` (or start-stellar.sh --record)
# and points the GUI camera at the drone, so the run can be filmed from a chase view.
#
# Usage (from the host):
#   ./docker/replay.sh                          # newest log in logs/gz/, follow the drone
#   ./docker/replay.sh 20261016-101500          # a specific log (name or path under logs/gz/)
#   ./docker/replay.sh -f truck                 # follow the truck instead
#   ./docker/replay.sh -o -12,0,6               # camera offset from the target in metres (x,y,z)
#
# The sim container must be running (start-dev.sh / start-stellar.sh), but the live sim
# must not: stop PX4 first (Ctrl+C in its terminal, or `shutdown` at pxh>).
#
# Filming: in the Gazebo window click the video camera icon (top left) and pick mp4,
# press play (bottom left), and click the icon again when done. Save under /workspace/
# so the mp4 lands in this repo. The recorder runs on sim time in lockstep with the
# replay, so a slow computer makes the export slower but the video stays smooth.
set -e

FOLLOW="x500_gimbal_0"
OFFSET="-8,0,4"
LOG=""

while [ $# -gt 0 ]; do
  case "$1" in
    -f|--follow) FOLLOW="$2"; shift 2 ;;
    -o|--offset) OFFSET="$2"; shift 2 ;;
    -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "Unknown flag: $1"; exit 1 ;;
    *) LOG="$1"; shift ;;
  esac
done

cd "$(dirname "$0")/.."

if [ -z "$LOG" ]; then
  LOG=$(ls -d logs/gz/*/ 2>/dev/null | sort | tail -n 1)
fi
NAME=$(basename "${LOG%/}")
if [ -z "$NAME" ] || [ ! -d "logs/gz/$NAME" ]; then
  echo "No log found in logs/gz/. Record one first: ./docker/start-dev.sh --record"
  exit 1
fi

if [ "$(docker inspect -f '{{.State.Running}}' flatbed-falcon-sim 2>/dev/null)" != "true" ]; then
  echo "The sim container is not running. Start it with ./docker/start-dev.sh (or start-stellar.sh)."
  exit 1
fi
if docker exec flatbed-falcon-sim pgrep -f "gz sim" >/dev/null; then
  echo "Gazebo is still running in the container. Stop PX4 first (Ctrl+C in its terminal)."
  exit 1
fi

IFS=, read -r OX OY OZ <<< "$OFFSET"
echo "Replaying logs/gz/$NAME, camera follows $FOLLOW at offset ($OX, $OY, $OZ)."

docker exec -it \
  -e LOG_DIR="/workspace/logs/gz/$NAME" -e FOLLOW="$FOLLOW" -e OX="$OX" -e OY="$OY" -e OZ="$OZ" \
  flatbed-falcon-sim bash -c '
  # Same model paths PX4 gives Gazebo, in case a mesh was not copied into the log.
  export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:/opt/px4-gazebo/share/gz/models:/opt/px4-gazebo/share/gz/worlds"

  # Default GUI layout, with the video recorder switched to sim time + lockstep.
  GUI_ARGS=()
  DEFAULT_GUI=$(ls /usr/share/gz/gz-sim*/gui/gui.config 2>/dev/null | tail -n 1)
  if [ -n "$DEFAULT_GUI" ] && python3 /workspace/docker/replay_gui_config.py "$DEFAULT_GUI" /tmp/replay.gui.config; then
    GUI_ARGS=(--gui-config /tmp/replay.gui.config)
  else
    echo "Could not prepare the GUI config: the video recorder will use wall-clock time (may be choppy)."
  fi

  # Lock the camera on the target once the GUI is up.
  (
    for _ in $(seq 1 180); do
      if gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
          --timeout 1000 --req "data: \"$FOLLOW\"" 2>/dev/null | grep -q "data: true"; then
        gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
          --timeout 1000 --req "x: $OX, y: $OY, z: $OZ" >/dev/null 2>&1
        echo "Camera is following $FOLLOW. Start the video recorder, then press play."
        exit 0
      fi
      sleep 1
    done
    echo "Could not lock the camera on $FOLLOW: right-click it in the entity tree and pick Follow."
  ) &

  gz sim --playback "$LOG_DIR" "${GUI_ARGS[@]}"
'
