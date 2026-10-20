#!/usr/bin/env bash
# Source this file in every ROS terminal; it changes this shell only.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${RPO_ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=${RPO_ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rpo_project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -f "$rpo_project_root/install/setup.bash" ]]; then
  source "$rpo_project_root/install/setup.bash"
fi
unset rpo_project_root
