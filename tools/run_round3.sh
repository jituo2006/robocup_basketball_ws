#!/usr/bin/env bash
# 第3回合完整传球流程：无预装球，取两颗篮球、分别传球，再回位。
# 默认开启机构；--no-mechanism 可关闭机构。固定角度由 match_rounds.json
# release.pass.aim_mode=fixed / search_heading_deg 配置，不要求 aim 坐标。
set -euo pipefail
round_mechanism=--mechanism
round_args=()
for option in "$@"; do
  case "$option" in
    --mechanism) round_mechanism=--mechanism ;;
    --no-mechanism) round_mechanism=--no-mechanism ;;
    --pickup-only|--round-first-ball|--round-first-ball=*)
      echo '第3轮比赛入口固定无预装球、完整流程；只测取球请用 run_round_test.sh 3 pass --pickup-only。'; exit 2 ;;
    *) round_args+=("$option") ;;
  esac
done
if [[ "$round_mechanism" == --mechanism ]]; then
  _rounds_json="$(dirname "${BASH_SOURCE[0]}")/match_rounds.json"
  python3 - "$_rounds_json" <<'PYCHECK'
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1]).resolve().parent))
from match_round_route import load_config, validate_mode
try:
    validate_mode(load_config(sys.argv[1]), 'pass', 'cycle', 'match', mechanism_enabled=True)
except (ValueError, KeyError, TypeError) as exc:
    print(f'无法启动正式传球：{exc}', file=sys.stderr)
    sys.exit(2)
PYCHECK
fi
echo '第3轮全自动：静止5秒 → 右转90°入场 → 取第1球 → 动作点 → 取第2球 → 动作点 → 回位。'
echo "机构开关：${round_mechanism}（关闭时跳过全部吸球/出球/瞄准）"
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_round_test.sh" 3 pass \
  --round-release-kind match \
  "$round_mechanism" ${round_args[@]+"${round_args[@]}"}
