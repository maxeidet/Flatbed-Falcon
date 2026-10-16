#!/usr/bin/env bash
# Replays a Gazebo log saved with `start-dev.sh --record` (or start-stellar.sh --record)
# and puts the GUI camera at one angle, so the same run can be filmed once per angle.
# The run itself costs almost nothing to record; the filming happens here afterwards.
#
# Usage (from the host):
#   ./docker/replay.sh                          # newest log in logs/gz/, chase the drone
#   ./docker/replay.sh -a bed_closeup           # one of the angles below
#   ./docker/replay.sh 20261016-101500 -a tripod_z0   # a specific log (name under logs/gz/)
#   ./docker/replay.sh -a truck_chase -p 0.05   # camera smoothing: lower = softer, laggier (0.1)
#   ./docker/replay.sh -v                       # print Gazebo's full log, to see why it closed
#
# Angles (the same as film.sh's cameras, so clips cut together):
#   drone_chase  behind and above the drone          truck_chase  behind and above the truck
#   truck_side   alongside the truck                 overhead     35 m above the truck
#   bed_closeup  cab roof, turned towards the drone  tripod_z0 / tripod_z1 / tripod_z2
#                                                    fixed beside a landing zone, panning after the drone
# The drone's gimbal camera cannot be replayed: film it live with `film.sh -s gimbal`.
#
# On a Mac the sim container must be running (start-dev.sh) but the live sim must not:
# stop PX4 first (Ctrl+C in its terminal). On Stellar, PX4 stopping removes the container,
# so the script starts a temporary one that draws on this computer's screen.
#
# Filming: in the Gazebo window click the video camera button (in its own small panel,
# or top left) and pick mp4,
# press play (bottom left), and click the icon again when done. Save under /workspace/
# so the mp4 lands in this repo. The recorder runs on sim time in lockstep with the
# replay, so a slow computer makes the export slower but the video stays smooth.
set -e

ANGLE="drone_chase"
PGAIN="0.1"
LOG=""
VERBOSE="1"

while [ $# -gt 0 ]; do
  case "$1" in
    -a|--angle) ANGLE="$2"; shift 2 ;;
    -p|--pgain) PGAIN="$2"; shift 2 ;;
    -v|--verbose) VERBOSE="4"; shift ;;
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

CONTAINER="flatbed-falcon-sim"
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
  if [ "$(uname -s)" = "Linux" ] && [ -n "${DISPLAY:-}" ]; then
    # Stellar: start-stellar.sh removes its container when PX4 stops, so start a
    # temporary one from the same image that draws on this computer's screen.
    CONTAINER="flatbed-falcon-replay"
    GPU_ARGS=()
    command -v nvidia-smi >/dev/null 2>&1 && GPU_ARGS=(--device nvidia.com/gpu=all)
    XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
    XAUTH_ARGS=()
    [ -f "$XAUTH_FILE" ] && XAUTH_ARGS=(-e XAUTHORITY=/tmp/.Xauthority -v "${XAUTH_FILE}":/tmp/.Xauthority:ro)
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    trap 'docker rm -f "$CONTAINER" >/dev/null 2>&1 || true' EXIT
    docker run -d --rm --name "$CONTAINER" "${GPU_ARGS[@]}" "${XAUTH_ARGS[@]}" \
      -e "DISPLAY=${DISPLAY}" -e NO_VNC=1 -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$(pwd)":/workspace flatbed-falcon-px4 sleep infinity >/dev/null
  else
    echo "The sim container is not running. Start it with ./docker/start-dev.sh (or start-stellar.sh)."
    exit 1
  fi
fi
if docker exec "$CONTAINER" pgrep -f "gz sim" >/dev/null; then
  echo "Gazebo is still running in the container. Stop PX4 first (Ctrl+C in its terminal)."
  exit 1
fi

echo "Replaying logs/gz/$NAME from the $ANGLE angle."

docker exec -it \
  -e LOG_DIR="/workspace/logs/gz/$NAME" -e ANGLE="$ANGLE" -e PGAIN="$PGAIN" -e VERBOSE="$VERBOSE" \
  "$CONTAINER" bash -c '
  # Same model paths PX4 gives Gazebo, in case a mesh was not copied into the log.
  export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:/opt/px4-gazebo/share/gz/models:/opt/px4-gazebo/share/gz/worlds"

  # Gazebo only replays properly with its own playback layout (passing --gui-config
  # makes it show the start menu instead), so write ours where it looks for that layout:
  # the installed playback layout plus a video recorder on sim time + lockstep.
  PB_GUI=$(ls /usr/share/gz/gz-sim*/gui/playback_gui.config 2>/dev/null | tail -n 1)
  if [ -n "$PB_GUI" ]; then
    MAJOR=$(basename "$(dirname "$(dirname "$PB_GUI")")" | tr -dc "0-9")
    mkdir -p "$HOME/.gz/sim/$MAJOR"
    python3 /workspace/docker/replay_gui_config.py "$PB_GUI" "$HOME/.gz/sim/$MAJOR/playback_gui.config" \
      || echo "Could not add the video recorder to the replay window."
  fi

  # Replays only need these two systems. PX4 points GZ_SIM_SERVER_CONFIG_PATH at its own
  # list, which adds the contact system, and that crashes on the replayed bed sensor.
  cat > /tmp/replay_server.config <<CFG
<server_config><plugins>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin entity_name="*" entity_type="world" filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
</plugins></server_config>
CFG
  export GZ_SIM_SERVER_CONFIG_PATH=/tmp/replay_server.config

  # Put the camera at the angle once the GUI is up.
  python3 /workspace/docker/replay_camera.py "$ANGLE" "$PGAIN" &

  gz sim -v "$VERBOSE" --playback "$LOG_DIR"
  CODE=$?
  [ "$CODE" = 0 ] || echo "Gazebo stopped with exit code $CODE. Run again with -v to see why."
'
