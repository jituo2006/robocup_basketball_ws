#!/usr/bin/env bash
# 场上取两颗篮球：中线横扫 → 传球点 → (3.725,4.800) → 向(3.725,0)找第二球。
set -euo pipefail
first_pass_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$first_pass_root"
# Help and plan inspection are available without ROS or robot nodes.
for first_pass_option in "$@"; do
  case "$first_pass_option" in
    --help|-h|--plan) exec python3 tools/test_perimeter_ball.py --route first-pass "$@" ;;
  esac
done
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}"
exec python3 tools/test_perimeter_ball.py --route first-pass --takeover --yes "$@"
