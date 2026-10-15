#!/usr/bin/env bash
# 切换出发点 A / B —— 一次把【两个文件】改好。
#
# 为什么要一个脚本：
#   出发点是 B 时有两处必须同时改：
#     ① tools/match_rounds.json  的 start_point     —— 路线整份关于场地中线镜像
#     ② src/rb_localization/config/localization.yaml 的 start_pose
#        （如果它上面还有 initial_pose，也一起改）
#   ②最要命：定位配置里 zero_on_first=true，start_pose 就是定位原点。
#   车摆在 B 而 start_pose 还写着 A，定位整体会错 8.5 米，而且是静默的 ——
#   车会以为自己在另一片场地上，所有目标点全偏，可能直接冲出场。
#
# 用法：
#   bash tools/set_start_point.sh A     # 从 A 区出发（默认）
#   bash tools/set_start_point.sh B     # 从 B 区出发
#   bash tools/set_start_point.sh       # 只打印当前设置，不改任何东西
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"
ROUNDS="$HERE/match_rounds.json"
LOC="$WS/src/rb_localization/config/localization.yaml"

python3 - "$ROUNDS" "$LOC" "${1:-}" <<'PY'
import json
import re
import shutil
import sys
from pathlib import Path

rounds_p, loc_p, want = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3].strip().upper()

cfg = json.loads(rounds_p.read_text(encoding='utf-8'))
points = cfg.get('start_points') or {}
# start_points 缺了就按 y 对称推一个
if 'A' not in points and isinstance(cfg.get('start'), list):
    points['A'] = list(cfg['start'])
if 'B' not in points and 'A' in points:
    points['B'] = [points['A'][0], -points['A'][1]]

cur = str(cfg.get('start_point', 'A')).upper()
if not want:
    print(f'当前出发点：{cur}')
    for k in ('A', 'B'):
        if k in points:
            print(f'  {k} 区摆放坐标 {points[k]}')
    print('用法：bash tools/set_start_point.sh A|B')
    sys.exit(0)

if want not in ('A', 'B'):
    print(f'出发点只能是 A 或 B，收到 {want!r}', file=sys.stderr)
    sys.exit(2)
if want not in points:
    print(f'match_rounds.json 的 start_points 里没有 {want}', file=sys.stderr)
    sys.exit(2)

xy = points[want]
yaw = 0.0
# A/B 是关于场地中线对称的两侧，两侧车头都朝 +x（用户确认全场 y 对称）

print(f'出发点 {cur} → {want}')
print(f'  摆放坐标 x={xy[0]}  y={xy[1]}  yaw={yaw}')

# ── ① match_rounds.json ──
cfg['start_point'] = want
rounds_p.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
print(f'  ✅ {rounds_p.name}  start_point = {want}')

# ── ② localization.yaml 的 start_pose / initial_pose ──
if not loc_p.is_file():
    print(f'  ⚠️ 找不到 {loc_p}，只改了路线；请手工确认定位起点', file=sys.stderr)
    sys.exit(1)

text = loc_p.read_text(encoding='utf-8')
shutil.copy2(loc_p, str(loc_p) + '.bak')
print(f'  备份 → {loc_p.name}.bak')

changed = []
for key in ('start_pose', 'initial_pose'):
    pat = re.compile(r'^([ \t]*)' + key + r':[ \t]*$', re.M)

    def repl(m, _xy=xy, _yaw=yaw, _key=key):
        indent = m.group(1)
        unit = indent + '  '
        changed.append(_key)
        # ⚠️ 结尾必须有换行：被匹配掉的原文里含 yaw 那一行的换行，
        #    替换串不带换行会把下一段（比如 ekf:）粘到 yaw 后面。
        return (f'{indent}{_key}:\n'
                f'{unit}x: {_xy[0]}\n'
                f'{unit}y: {_xy[1]}\n'
                f'{unit}yaw: {_yaw}\n')

    # 连同它下面的 x/y/yaw 行一起换掉
    pat_full = re.compile(r'^([ \t]*)' + key + r':[ \t]*\n'
                          r'(?:[ \t]+(?:x|y|z|yaw|roll|pitch):[^\n]*\n)+', re.M)
    text, n = pat_full.subn(repl, text)
    if n == 0 and pat.search(text):
        print(f'  ⚠️ {key} 的结构和预期不同，没有改；请手工确认', file=sys.stderr)

loc_p.write_text(text, encoding='utf-8')
for k in changed:
    print(f'  ✅ localization.yaml  {k} → x={xy[0]} y={xy[1]} yaw={yaw}')
if not changed:
    print('  ⚠️ localization.yaml 里没找到 start_pose / initial_pose；请手工确认', file=sys.stderr)

# ── 核对 ──
print()
print('核对：')
back = json.loads(rounds_p.read_text(encoding='utf-8'))
print(f'  match_rounds.json  start_point = {back.get("start_point")}  '
      f'{"✅" if back.get("start_point") == want else "❌"}')
after = loc_p.read_text(encoding='utf-8')
ok = True
for key in ('start_pose', 'initial_pose'):
    m = re.search(r'^[ \t]*' + key + r':[ \t]*\n'
                  r'[ \t]+x:[ \t]*(\S+)[ \t]*\n'
                  r'[ \t]+y:[ \t]*(\S+)[ \t]*\n'
                  r'[ \t]+yaw:[ \t]*(\S+)', after, re.M)
    if not m:
        print(f'  {key} 读不出来 ⚠️')
        continue
    good = (float(m.group(1)) == float(xy[0]) and float(m.group(2)) == float(xy[1]))
    ok = ok and good
    print(f'  localization.yaml  {key} = x={m.group(1)} y={m.group(2)} yaw={m.group(3)}  '
          f'{"✅" if good else "❌"}')
print()
print('✅ 两处已同步' if ok else '❌ 有地方没对上，请手工检查')
sys.exit(0 if ok else 1)
PY
