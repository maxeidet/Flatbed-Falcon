#!/usr/bin/env bash
# Builds the workspace inside the running sim container and starts all nodes.
# Run from the host in a second terminal, after docker/start-dev.sh (or start-stellar.sh).
docker exec -it flatbed-falcon-sim bash -c "
  source /opt/ros/jazzy/setup.bash
  source /opt/px4_ros2/install/local_setup.bash
  cd /workspace/ros2_ws
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch flatbed_falcon landing.launch.py
"
