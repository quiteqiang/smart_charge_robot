#!/usr/bin/env bash
# One-key full demo: simulator + AMCL + Nav2 + battery + mission + docking.
# Headless by default; pass `--rviz` on a machine with an X display.
set -euo pipefail
cd "$(dirname "$0")/.."

RVIZ="false"
if [[ "${1:-}" == "--rviz" ]]; then
  RVIZ="true"
fi

exec docker compose run --rm dev bash -lc \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 launch smart_charge_bringup full_demo.launch.py use_rviz:=${RVIZ}"
