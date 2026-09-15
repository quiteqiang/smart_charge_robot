#!/usr/bin/env bash
# Enter an interactive dev container shell (workspace mounted at /home/ubuntu/ws).
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose run --rm dev bash
