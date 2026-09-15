#!/usr/bin/env bash
# Record key topics with rosbag2 into bags/<timestamp>/.
set -euo pipefail
cd "$(dirname "$0")/.."
STAMP="$(date +%Y%m%d_%H%M%S)"
exec docker compose run --rm dev bash -lc \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 bag record -o bags/run_${STAMP} \
     /scan /odom /imu /battery_state /cmd_vel /amcl_pose \
     /mission_state /dock_relative_pose /dock_contact /charging_active \
     /docking_success /docking_status /tf /tf_static"
