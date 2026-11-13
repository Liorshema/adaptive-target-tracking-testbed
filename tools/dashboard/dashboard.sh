#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$workspace"
if [[ ! -f install/setup.bash || ! -x build/dashboard_venv/bin/python ]]; then
  echo 'Dashboard error: build the ROS workspace and install the existing Dashboard virtual environment first.' >&2
  exit 1
fi
source install/setup.bash
exec build/dashboard_venv/bin/python -m robot_dashboard.launcher "$@"
