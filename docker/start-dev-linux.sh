#!/usr/bin/env bash

set -e

MODEL="gz_x500_gimbal"

while [ $# -gt 0 ]; do
  case "$1" in
    -m|--model)
      MODEL="$2"
      shift 2
      ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^#//'
      exit 0
      ;;
    *)
      echo "Unknown flag: $1"
      exit 1
      ;;
  esac
done

cd "$(dirname "$0")"

echo "Building image (flatbed-falcon-px4)..."
docker build -t flatbed-falcon-px4 .

docker rm -f flatbed-falcon-sim >/dev/null 2>&1 || true

# Write the "wait for container, then exec PX4 inside it" logic to a temp
# script rather than inlining it, so we don't have to fight each terminal
# launcher's own quoting/escaping rules (cmd.exe and wt.exe in particular
# mangle long semicolon/quote-heavy -c strings).
WAIT_SCRIPT="$(mktemp /tmp/flatbed-falcon-wait-XXXXXX.sh)"
cat > "${WAIT_SCRIPT}" <<EOF
#!/usr/bin/env bash
until docker inspect -f '{{.State.Running}}' flatbed-falcon-sim 2>/dev/null | grep -q true; do
  sleep 1
done
docker exec -it flatbed-falcon-sim bash -c 'export PX4_SIM_MODEL=${MODEL} PX4_GZ_MODEL_POSE=0,0,0,0,0,0; /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0'
exec bash
EOF
chmod +x "${WAIT_SCRIPT}"

# Try to auto-open a terminal that waits for the container and starts PX4
# (mirrors what start-dev.sh does with Terminal.app on macOS). Falls back to
# printing manual instructions if no supported terminal emulator is found.
OPENED_TERMINAL=0
if command -v wt.exe >/dev/null 2>&1; then
  echo "Opening a new Windows Terminal tab that waits for the container and starts PX4 (model: ${MODEL})..."
  wt.exe new-tab bash "${WAIT_SCRIPT}" >/dev/null 2>&1 && OPENED_TERMINAL=1
elif command -v cmd.exe >/dev/null 2>&1; then
  echo "Opening a new terminal window that waits for the container and starts PX4 (model: ${MODEL})..."
  cmd.exe /c start bash "${WAIT_SCRIPT}" >/dev/null 2>&1 && OPENED_TERMINAL=1
elif command -v gnome-terminal >/dev/null 2>&1; then
  echo "Opening a new gnome-terminal window that waits for the container and starts PX4 (model: ${MODEL})..."
  gnome-terminal -- bash "${WAIT_SCRIPT}" >/dev/null 2>&1 && OPENED_TERMINAL=1
elif command -v konsole >/dev/null 2>&1; then
  echo "Opening a new konsole window that waits for the container and starts PX4 (model: ${MODEL})..."
  konsole -e bash "${WAIT_SCRIPT}" >/dev/null 2>&1 && OPENED_TERMINAL=1
elif command -v xterm >/dev/null 2>&1; then
  echo "Opening a new xterm window that waits for the container and starts PX4 (model: ${MODEL})..."
  xterm -e bash "${WAIT_SCRIPT}" >/dev/null 2>&1 && OPENED_TERMINAL=1
fi

if [ "${OPENED_TERMINAL}" -eq 0 ]; then
  echo
  echo "========================================"
  echo " Couldn't auto-open a terminal for PX4"
  echo "========================================"
  echo
  echo "Open another terminal and run:"
  echo
  echo "  docker exec -it flatbed-falcon-sim bash -c \\"
  echo "    'export PX4_SIM_MODEL=${MODEL}; \\"
  echo "     export PX4_GZ_MODEL_POSE=0,0,0,0,0,0; \\"
  echo "     /usr/local/bin/ros2-entrypoint.sh px4-gazebo -i 0'"
  echo
fi

echo
echo "VNC: http://localhost:6080"
echo
echo "Starting the container (Ctrl+C here shuts everything down)..."
docker run --rm -it --name flatbed-falcon-sim \
  -p 6080:6080 -p 5900:5900 \
  -v "$(pwd)/..":/workspace \
  flatbed-falcon-px4 MicroXRCEAgent udp4 -p 8888