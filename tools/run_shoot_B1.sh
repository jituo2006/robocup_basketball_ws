#!/usr/bin/env bash
# B区第1轮投篮：首球预装，两球完成后回位。
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_shoot_round.sh" B 1 "$@"
