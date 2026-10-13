#!/usr/bin/env bash
# 第3轮比赛流程（投篮）：中线无球 → 内线 X=9 / 三分弧取 2 颗排球 → 投篮点 Q → 回位
# 其余说明见 run_round3.sh / run_round2.sh 的注释。
set -euo pipefail
shoot_mechanism=--no-mechanism
shoot_args=()
for option in "$@"; do
  case "$option" in
    --mechanism) shoot_mechanism=--mechanism ;;
    --no-mechanism) shoot_mechanism=--no-mechanism ;;
    --pickup-only|--round-first-ball|--round-first-ball=*)
      echo '第3轮投篮入口固定无预装球、完整流程。'; exit 2 ;;
    *) shoot_args+=("$option") ;;
  esac
done
if [[ "$shoot_mechanism" == --mechanism ]]; then
  _rounds_json="$(dirname "${BASH_SOURCE[0]}")/match_rounds.json"
  python3 - "$_rounds_json" <<'PYCHECK'
import json, sys, pathlib
# ⚠️ heredoc 里没有 __file__ ✗，路径必须由 shell 传进来 ✓
p = pathlib.Path(sys.argv[1])
cfg = json.loads(p.read_text(encoding='utf-8'))
rel = cfg['release']['shoot']
missing = []
if not rel.get('commands'):
    missing.append('release.shoot.commands 为空')
if rel.get('aim') is None:
    missing.append('release.shoot.aim 为 null')
if missing:
    print('无法启动 match 模式（真投篮）：' + '；'.join(missing), file=sys.stderr)
    print('请先在 tools/match_rounds.json 里填好这两项；', file=sys.stderr)
    print('或先用 --no-mechanism 跑通路线。', file=sys.stderr)
    sys.exit(2)
PYCHECK
fi
echo '第3轮投篮全自动：静止5秒 → 右转90° → 内线X=9/三分弧取2颗排球 → Q投篮点 → 回位。'
echo "机构开关：${shoot_mechanism}"
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_round_test.sh" 3 shoot \
  --round-release-kind match \
  "$shoot_mechanism" ${shoot_args[@]+"${shoot_args[@]}"}
