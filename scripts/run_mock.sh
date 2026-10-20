#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/scripts/host_env.sh"
cd "$root"
ros2 run rpo_cat_control mock_robot --ros-args -p profile:="$root/config/rpo_23.yaml"
