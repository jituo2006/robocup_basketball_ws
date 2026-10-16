#!/usr/bin/env bash
# A/B三轮投篮公共入口；只在显式执行脚本时运行实车流程。
set -euo pipefail
shoot_side="${1:-}"; shoot_round="${2:-}"
if [[ ! "$shoot_side" =~ ^[AB]$ || ! "$shoot_round" =~ ^[123]$ ]]; then
  echo '用法：run_shoot_round.sh A|B 1|2|3 [--no-mechanism] [路线参数]' >&2
  exit 2
fi
shift 2
shoot_mechanism=--mechanism
shoot_extra=()
for option in "$@"; do
  case "$option" in
    --mechanism|--route-intake) shoot_mechanism=--mechanism ;;
    --no-mechanism|--no-route-intake) shoot_mechanism=--no-mechanism ;;
    --start-point|--start-point=*|--route|--route=*|--round-task|--round-task=*|--round-mode|--round-mode=*|--round-step|--round-step=*|--round-first-ball|--round-first-ball=*|--round-release-kind|--round-release-kind=*|--pickup-only)
      echo '本入口固定场地、回合、投篮任务和完整两球流程；单阶段联调请使用 test_perimeter_ball.py。' >&2
      exit 2 ;;
    *) shoot_extra+=("$option") ;;
  esac
done
shoot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
shoot_first=search
if [[ "$shoot_round" == 1 ]]; then shoot_first=preloaded; fi
# 复用现有配置校验，未配置真实投篮动作时在加载ROS之前退出。
python3 - "$shoot_root/tools" "$shoot_side" "$shoot_mechanism" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
from match_round_route import load_config, validate_mode
cfg = load_config(start_point=sys.argv[2])
validate_mode(cfg, 'shoot', 'cycle', 'match', mechanism_enabled=sys.argv[3] == '--mechanism')
print(f"起步静止{cfg['start_delay_s']:g}秒；投篮点={cfg['release']['shoot']['point']}，瞄准={cfg['release']['shoot']['aim']}")
PY
echo "${shoot_side}区第${shoot_round}轮投篮，首球=${shoot_first}，机构=${shoot_mechanism}，两球后回位。"
exec bash "$shoot_root/tools/run_round_test.sh" "$shoot_round" shoot \
  --start-point "$shoot_side" --round-first-ball "$shoot_first" \
  --round-release-kind match "$shoot_mechanism" ${shoot_extra[@]+"${shoot_extra[@]}"}
