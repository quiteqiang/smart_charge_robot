#!/usr/bin/env bash
# Insert/remove the dynamic obstacle on the driving route (obstacle-avoidance demo).
set -euo pipefail
cd "$(dirname "$0")/.."
ON="${1:-true}"
exec docker compose run --rm dev bash -lc \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   ros2 topic pub --once /sim/obstacles visualization_msgs/msg/Marker \
   \"{header: {frame_id: 'map'}, ns: 'dyn', id: 1, type: 1, action: $([[ $ON == true ]] && echo 0 || echo 2), pose: {position: {x: 8.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}, scale: {x: 0.8, y: 0.8, z: 0.8}, color: {r: 1.0, g: 0.2, b: 0.2, a: 0.8}}\""
