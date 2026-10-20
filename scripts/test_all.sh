#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cd "$root"
python3 -m unittest discover -s tests -p test_contracts.py -v
python3 -m unittest discover -s tests -p test_exported_motion.py -v
guard_binary=$(mktemp /tmp/rpo-arm-guard.XXXXXX)
trap 'rm -f "$guard_binary"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -I patches tests/test_external_arm_guard.cpp -o "$guard_binary"
"$guard_binary"
export ROS_DOMAIN_ID=${RPO_TEST_DOMAIN_ID:-76}
export ROS_LOCALHOST_ONLY=1
python3 -m unittest discover -s tests -p test_ros_integration.py -v
