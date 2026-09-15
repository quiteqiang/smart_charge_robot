#!/usr/bin/env bash
# Build the colcon workspace inside the dev container.
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose run --rm dev bash -lc \
  "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --event-handlers console_direct+"
