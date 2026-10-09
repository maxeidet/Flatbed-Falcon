#!/usr/bin/env bash
# Opens RViz with the drone's gimbal camera (/falcon/camera/image).
# Run from the host while the sim and the nodes are running (start-*.sh + run.sh): the
# camera topic comes from run.sh's bridge. RViz opens where the Gazebo GUI is: this
# computer's screen with start-stellar.sh (default --native-gui), the VNC desktop with
# start-dev.sh or --vnc. It cannot open with --headless.
docker exec -it flatbed-falcon-sim bash -c '
  source /opt/ros/jazzy/setup.bash
  source /workspace/ros2_ws/install/setup.bash
  layout=$(ros2 pkg prefix flatbed_falcon)/share/flatbed_falcon/config/camera.rviz
  if [ ! -f "$layout" ]; then
    echo "$layout is missing: run ./ros2_ws/run.sh first (it builds and installs it)."
    exit 1
  fi
  rviz2 -d "$layout"
'
