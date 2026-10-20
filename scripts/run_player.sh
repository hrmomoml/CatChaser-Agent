#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/scripts/host_env.sh"
cd "$root"
ros2 run rpo_cat_control motion_player --ros-args -p profile:="$root/config/rpo_23.yaml" -p library:="$root/artifacts" "$@"
