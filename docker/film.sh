#!/usr/bin/env bash
# Films one run from several angles at once, one mp4 per camera, for editing together.
#
#   ./docker/start-dev.sh --film          # 1. sim with the film cameras (also start-stellar.sh --film)
#   ./ros2_ws/run.sh                      # 2. the nodes, as usual
#   ./docker/film.sh                      # 3. record; Enter stops, or it stops by itself after touchdown
#
# Options:
#   -s truck_chase,tripod_z0    only these shots (default: all). The shots:
#        truck_chase  behind and above the truck    truck_side   alongside the truck
#        bed_closeup  cab roof, looking at the bed  overhead     straight down from 35 m
#        tripod_z0 / tripod_z1 / tripod_z2   fixed cameras beside the three landing zones
#   -t 20                       seconds (sim time) to keep filming after touchdown; 0 = only Enter
#
# Videos go to videos/<timestamp>/<shot>.mp4. They record on sim time, so they are smooth
# and real speed, but every camera slows the sim down while recording (on a Mac a lot):
# pick a few shots with -s if the run gets too slow, or lower resolution and frame rate when starting:
#   FILM_RES=960x540 FILM_FPS=15 ./docker/start-stellar.sh --film     (default 1280x720, 25 fps)
set -e

SHOTS="all"
AFTER=20

while [ $# -gt 0 ]; do
  case "$1" in
    -s|--shots) SHOTS="$2"; shift 2 ;;
    -t|--after-touchdown) AFTER="$2"; shift 2 ;;
    -h|--help) grep '^#' "$0" | tail -n +2 | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown flag: $1"; exit 1 ;;
  esac
done

if [ "$(docker inspect -f '{{.State.Running}}' flatbed-falcon-sim 2>/dev/null)" != "true" ]; then
  echo "The sim container is not running. Start it with ./docker/start-dev.sh --film"
  exit 1
fi

OUT="/workspace/videos/$(date +%Y%m%d-%H%M%S)"
docker exec -it flatbed-falcon-sim python3 /workspace/docker/film.py "$OUT" "$SHOTS" "$AFTER"
