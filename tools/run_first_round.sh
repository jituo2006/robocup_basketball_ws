#!/usr/bin/env bash
# 第一轮比赛流程：首球预装 → 传球区 → 中线取第二球 → 传球区 → 回A区。
# 当前默认不连接机构；开启后执行配置的真实传球指令。
set -euo pipefail
first_round_mechanism=--no-mechanism
first_round_args=()
for option in "$@"; do
  case "$option" in
    --mechanism) first_round_mechanism=--mechanism ;;
    --no-mechanism) first_round_mechanism=--no-mechanism ;;
    --pickup-only|--round-first-ball|--round-first-ball=*|--round-release-kind|--round-release-kind=*)
      echo '第一轮比赛入口固定首球预装和完整流程；只测取球请使用分阶段入口。'; exit 2 ;;
    *) first_round_args+=("$option") ;;
  esac
done
echo '第一轮全自动：静止5秒 → 首球到传球区 → 中线取第二球 → 再到传球区 → 回位。'
echo "机构开关：${first_round_mechanism}（关闭时跳过全部机构动作和发射瞄准）"
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_round_test.sh" 1 pass \
  --round-release-kind match --round-first-ball preloaded \
  "$first_round_mechanism" ${first_round_args[@]+"${first_round_args[@]}"}
