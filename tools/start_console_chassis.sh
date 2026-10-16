#!/usr/bin/env bash
set -eo pipefail
console_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$console_root"
source /opt/ros/humble/setup.bash
source "$console_root/install/setup.bash"
export ROS_DOMAIN_ID=2
exec /usr/bin/python3 tools/console_chassis.py
