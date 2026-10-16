#!/usr/bin/env bash
# Runs INSIDE the sim container, before PX4 starts Gazebo (start-dev.sh / start-stellar.sh
# call it with --record). Adds Gazebo's LogRecord system to the container's copy of the
# world, so the run's poses are saved to logs/gz/<timestamp>/ in the repo. The world file
# in worlds/ is never touched, so normal runs record nothing.
# Replay and film it afterwards with docker/replay.sh.
set -e

WORLD="${1:-hive_base}"
SDF="/opt/px4-gazebo/share/gz/worlds/${WORLD}.sdf"
OUT="/workspace/logs/gz/$(date +%Y%m%d-%H%M%S)"

if [ ! -f "$SDF" ]; then
  echo "enable-log-record: $SDF not found, recording is off for this run."
  exit 0
fi

mkdir -p "$(dirname "$OUT")"
# No record_resources: copying the meshes crashes Gazebo 8 (LogRecord::SaveModels segfaults)
# when PX4 spawns the drone. The replay finds the meshes through the same model paths instead.
# record_period limits the state to 50 Hz instead of every 1 ms physics step, which keeps
# the log small and is still smooth for video.
python3 - "$SDF" "$OUT" <<'PY'
import re, sys
sdf, out = sys.argv[1], sys.argv[2]
text = open(sdf).read()
text = re.sub(r'\s*<plugin filename="gz-sim-log-system".*?</plugin>', '', text, flags=re.S)
plugin = (
    '\n    <plugin filename="gz-sim-log-system" name="gz::sim::systems::LogRecord">'
    f'<record_path>{out}</record_path>'
        '<record_period>0.02</record_period></plugin>'
)
text, n = re.subn(r'(<world name="[^"]*">)', r'\1' + plugin, text, count=1)
if n != 1:
    sys.exit(f"enable-log-record: no <world> element in {sdf}")
open(sdf, 'w').write(text)
PY
echo "Recording this run to logs/gz/$(basename "$OUT")/ (replay with docker/replay.sh)"
