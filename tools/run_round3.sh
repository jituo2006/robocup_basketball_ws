#!/usr/bin/env bash
# 第3轮比赛流程（pass）：无中线球 → 三分弧/内线取2球 → 传球点 → 回位
#
# ⚠️ 与第1轮的区别（照 run_first_round.sh 的样板，只改这几处）：
#   * 【没有预装球】—— preloaded 分支只在 round==1 生效
#     （match_round_route.py:354），所以第3轮第1阶段就要完整
#     走 search → align → approach → carry → release
#   * 【搜索顺序】search_order(3, pass) = ['arc', 'inner']
#     （match_round_route.py:132-139，已按规则用球表写好）
#
# ⚠️ 机构开关默认【关闭】：先用 --no-mechanism 把整条路线跑通
#    （只走位姿，不发任何串口）。要真出球用 --mechanism。
#    但注意：真出球（match 模式）需要 match_rounds.json 里
#        release.pass.commands 非空 且 aim 非 null
#    否则 match_round_route.py:126-129 会直接抛
#        ValueError('双次流程须先配置真实出球 commands 和 aim')
set -euo pipefail
round_mechanism=--no-mechanism
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
import json, sys, pathlib
# ⚠️ heredoc 里没有 __file__ ✗，路径必须由 shell 传进来 ✓
p = pathlib.Path(sys.argv[1])
cfg = json.loads(p.read_text(encoding='utf-8'))
rel = cfg['release']['pass']
missing = []
if not rel.get('commands'):
    missing.append(f"release.pass.commands 为空")
if rel.get('aim') is None:
    missing.append(f"release.pass.aim 为 null")
if missing:
    print('无法启动 match 模式（真出球）：' + '；'.join(missing), file=sys.stderr)
    print('请先在 tools/match_rounds.json 里填好这两项；', file=sys.stderr)
    print('或先用 --no-mechanism 跑通路线。', file=sys.stderr)
    sys.exit(2)
PYCHECK
fi
echo '第3轮全自动：静止5秒 → 右转90°入场 → 取第1球 → 动作点 → 取第2球 → 动作点 → 回位。'
echo "机构开关：${round_mechanism}（关闭时跳过全部吸球/出球/瞄准）"
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_round_test.sh" 3 pass \
  --round-release-kind match \
  "$round_mechanism" ${round_args[@]+"${round_args[@]}"}
