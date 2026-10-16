#!/usr/bin/env bash
# Films every angle of one recorded run with no clicking, one replay after another:
# videos/<log>/drone_chase.mp4, truck_chase.mp4, ... Leave the Gazebo windows alone
# while it runs: each one films itself and closes.
#
# Usage (from the host, after the run, with PX4 stopped):
#   ./docker/replay_all.sh                        # newest log in logs/gz/, all angles
#   ./docker/replay_all.sh -s truck_chase,overhead   # only some angles
#   ./docker/replay_all.sh 20261016-155656        # a specific log (name under logs/gz/)
#   FILM_FPS=20 ./docker/replay_all.sh            # fewer frames, faster (default 25)
#
# Every frame is rendered and saved, so the videos have the run's real speed however
# slow the computer is, but filming takes longer than the run itself.
# Needs ffmpeg on this computer, or: python3 -m pip install --user imageio-ffmpeg
set -e

ANGLES="drone_chase,truck_chase,truck_side,overhead,bed_closeup,tripod_z0,tripod_z1,tripod_z2"
LOG_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -s|--shots) ANGLES="$2"; shift 2 ;;
    -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "Unknown flag: $1"; exit 1 ;;
    *) LOG_ARGS=("$1"); shift ;;
  esac
done

cd "$(dirname "$0")/.."
FAILED=()
for ANGLE in ${ANGLES//,/ }; do
  echo ""
  echo "=== $ANGLE ==="
  ./docker/replay.sh "${LOG_ARGS[@]}" -a "$ANGLE" --auto || FAILED+=("$ANGLE")
done

echo ""
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "All angles filmed. The videos are in videos/<log name>/."
else
  echo "These angles failed: ${FAILED[*]}. Try one alone: ./docker/replay.sh -a ${FAILED[0]} --auto -v"
  exit 1
fi
