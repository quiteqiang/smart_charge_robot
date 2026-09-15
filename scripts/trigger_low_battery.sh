#!/usr/bin/env bash
# Demo helper: force SOC low so the mission interrupts and goes charging.
set -euo pipefail
cd "$(dirname "$0")/.."
SOC="${1:-0.20}"
exec docker compose run --rm dev bash -lc \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 service call /set_soc smart_charge_msgs/srv/SetSoc \"{soc: ${SOC}}\""
