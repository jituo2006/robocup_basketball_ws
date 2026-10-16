#!/usr/bin/env bash
# A区第3轮投篮：场上取首球，两球完成后回位。
set -euo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_shoot_round.sh" A 3 "$@"
