#!/usr/bin/env bash
# 切换出发点 A / B —— 一次把所有该改的文件都改好。
#
# 为什么要一个脚本：
#   出发点是 B 时有【两处必须同时改】：
#     ① tools/match_rounds.json  的 start_point     —— 路线整份关于场地中线镜像
#     ② src/rb_localization/config/localization.yaml 的 start_pose
#        （如果它上面还有 initial_pose，也一起改）
#   ②最要命：定位配置里 zero_on_first=true，start_pose 就是定位原点。
#   车摆在 B 而 start_pose 还写着 A，定位整体会错 8.5 米，而且是静默的 ——
#   车会以为自己在另一片场地上，所有目标点全偏，可能直接冲出场。
#
# 为什么还要动 install/ 那一份：
#   launch 里用 get_package_share_directory('rb_localization') 读配置，
#   那个路径指向 install/，不是 src/。colcon 默认把 yaml【拷贝】过去，
#   所以只改 src/ 的话节点根本看不到，必须重编才生效。
#   本脚本会把 install/ 那一份也一起改（存在且不是软链时），
#   于是切换出发点只要【重启定位】，不用重新 colcon build。
#   （如果工作区是 --symlink-install 建的，install/ 那份就是指向 src 的软链，
#     写 src 已经等于写它，脚本会自动跳过。）
#
# 用法：
#   bash tools/set_start_point.sh A     # 从 A 区出发
#   bash tools/set_start_point.sh B     # 从 B 区出发
#   bash tools/set_start_point.sh       # 只打印当前设置，不改任何东西
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$HERE")"
ROUNDS="$HERE/match_rounds.json"
LOC="$WS/src/rb_localization/config/localization.yaml"
INST="$WS/install/rb_localization/share/rb_localization/config/localization.yaml"

# 节点实际读的那一份：install/ 存在且不是软链时，两份都要改。
LOC_ARGS=("$LOC")
INST_NOTE="工作区没编过（没有 install/），只改了源码那一份"
if [[ -L "$INST" ]]; then
  INST_NOTE="install/ 那份是软链（--symlink-install），写源码已经等于写它"
elif [[ -f "$INST" ]]; then
  LOC_ARGS+=("$INST")
  INST_NOTE="install/ 那份也改了 ⇒ 不用重新 colcon build，重启定位即可"
fi

python3 - "$ROUNDS" "$INST_NOTE" "${1:-}" "${LOC_ARGS[@]}" <<'PY'
import json
import re
import shutil
import sys
from pathlib import Path

rounds_p = Path(sys.argv[1])
inst_note = sys.argv[2]
want = sys.argv[3].strip().upper()
locs = [Path(p) for p in sys.argv[4:]]

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
    for p in locs:
        if not p.is_file():
            continue
        txt = p.read_text(encoding='utf-8')
        m = re.search(r'^[ \t]*start_pose:[ \t]*\n'
                      r'[ \t]+x:[ \t]*(\S+)[ \t]*\n'
                      r'[ \t]+y:[ \t]*(\S+)[ \t]*\n'
                      r'[ \t]+yaw:[ \t]*(\S+)', txt, re.M)
        tag = 'install/（节点实际读的）' if 'install' in str(p) else 'src/'
        print(f'  {tag:24s} start_pose = '
              + (f'x={m.group(1)} y={m.group(2)} yaw={m.group(3)}' if m else '读不出来'))
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

# ── ② localization.yaml 的 start_pose / initial_pose（可能有多份）──
wrote = []
for loc_p in locs:
    if not loc_p.is_file():
        print(f'  ⚠️ 找不到 {loc_p}，跳过', file=sys.stderr)
        continue
    text = loc_p.read_text(encoding='utf-8')
    shutil.copy2(loc_p, str(loc_p) + '.bak')
    tag = 'install/' if 'install' in str(loc_p) else 'src/'
    print(f'  {tag:9s} 备份 → {loc_p.name}.bak')

    changed = []
    for key in ('start_pose', 'initial_pose'):
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

        pat_full = re.compile(r'^([ \t]*)' + key + r':[ \t]*\n'
                              r'(?:[ \t]+(?:x|y|z|yaw|roll|pitch):[^\n]*\n)+', re.M)
        text, n = pat_full.subn(repl, text)
        if n == 0:
            print(f'  ⚠️ {loc_p.name} 里 {key} 的结构和预期不同，没有改；请手工确认',
                  file=sys.stderr)
    # 顺手更新那句说明注释，否则值改了、注释还写着「当前 A 起点」，以后看着糊涂。
    # 匹配不到就跳过，不算错误。
    text, n_note = re.subn(
        r'^([ \t]*)#[ \t]*当前 [AB] 起点.*$',
        lambda m: (f'{m.group(1)}# 当前 {want} 起点：({xy[0]}, {xy[1]})，'
                   f'与 match_rounds.json 的 start_points.{want} 一致'
                   f'（由 tools/set_start_point.sh 写入）。'),
        text, flags=re.M)

    loc_p.write_text(text, encoding='utf-8')
    if changed:
        wrote.append((loc_p, changed))
        print(f'  ✅ {tag:9s} {", ".join(changed)} → x={xy[0]} y={xy[1]} yaw={yaw}'
              + (f'；说明注释也更新了' if n_note else ''))

if not wrote:
    print('  ⚠️ 没有任何 localization.yaml 被改；请手工确认定位起点', file=sys.stderr)
    sys.exit(1)

# ── 核对：每一份都读回来比一遍 ──
print()
print('核对：')
back = json.loads(rounds_p.read_text(encoding='utf-8'))
ok = back.get('start_point') == want
print(f'  match_rounds.json  start_point = {back.get("start_point")}  '
      f'{"✅" if ok else "❌"}')
for loc_p, changed in wrote:
    tag = 'install/' if 'install' in str(loc_p) else 'src/'
    after = loc_p.read_text(encoding='utf-8')
    for key in changed:
        m = re.search(r'^[ \t]*' + key + r':[ \t]*\n'
                      r'[ \t]+x:[ \t]*(\S+)[ \t]*\n'
                      r'[ \t]+y:[ \t]*(\S+)[ \t]*\n'
                      r'[ \t]+yaw:[ \t]*(\S+)', after, re.M)
        if not m:
            print(f'  {tag}{key} 读不出来 ⚠️')
            ok = False
            continue
        good = (float(m.group(1)) == float(xy[0]) and float(m.group(2)) == float(xy[1]))
        ok = ok and good
        print(f'  {tag:9s} {key} = x={m.group(1)} y={m.group(2)} yaw={m.group(3)}  '
              f'{"✅" if good else "❌"}')
print()
print(f'  {inst_note}')
print()
print('✅ 已同步' if ok else '❌ 有地方没对上，请手工检查')
sys.exit(0 if ok else 1)
PY
