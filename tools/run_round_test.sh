#!/usr/bin/env bash
# 单轮完整测试；先在另一个终端启动 boot.sh，再运行本脚本。
set -euo pipefail
if [[ $# -lt 1 || ! "$1" =~ ^[123]$ ]]; then
  echo '用法: bash tools/run_round_test.sh 1|2|3 [pass|shoot] [额外路线参数]'
  echo '例: bash tools/run_round_test.sh 1 pass --no-route-intake'
  echo '第一轮先测找篮球、吸球、到传球区停车: bash tools/run_round_test.sh 1 pass --pickup-only'
  exit 2
fi
round_number="$1"; shift
round_task=pass
if [[ "${1:-}" == pass || "${1:-}" == shoot ]]; then round_task="$1"; shift; fi
round_mode=cycle
round_extra_args=()
for option in "$@"; do
  case "$option" in
    --pickup-only)
      if [[ "$round_number" != 1 || "$round_task" != pass ]]; then
        echo '--pickup-only 用于第一轮找篮球并到传球区停车。'; exit 2
      fi
      round_mode=collect ;;
    --route|--route=*|--round-mode|--round-mode=*|--round-step|--round-step=*|--round-task|--round-task=*)
      echo '整轮脚本固定运行所选回合的两次处理球及回位；单阶段/空场实验请用 test_perimeter_ball.py。'
      exit 2 ;;
    *) round_extra_args+=("$option") ;;
  esac
done
round_test_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$round_test_root"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID=2
if [[ "$round_mode" == collect ]]; then
  echo '第一轮取球测试：找篮球 → 吸球 → 保持吸球到传球区停车；不自动放球和取第二颗。'
else
  echo "整轮测试：第${round_number}轮 ${round_task}，两次处理球后回位。首球默认场上取球，规则模式加 --round-first-ball preloaded。"
fi
exec python3 tools/test_perimeter_ball.py --route "round${round_number}" \
  --round-task "$round_task" --round-mode "$round_mode" --round-release-kind drop \
  --round-first-ball search --takeover --yes "${round_extra_args[@]}"
