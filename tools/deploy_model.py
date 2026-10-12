#!/usr/bin/env python3
"""换视觉模型：把训好的 ONNX 部署进 rb_perception，并同步所有引用它的地方。

为什么需要脚本而不是手工 cp
--------------------------
`src/rb_perception/test/test_deployed_model.py` 把"换模型"定义成一份**契约**，
手工换漏一处就会红：

  ① 模型文件要落进 `src/rb_perception/models/`（随仓库发布，不依赖主机外部路径）
  ② `models/model.json` 的 `sha256` 必须与模型文件**逐字节**一致
     （测试会重算哈希比对）
  ③ 所有 perception 配置的 `model_path` 都要指向**同一个**版本化文件名
  ④ 元数据类名必须是 `{0:'篮球',1:'排球',2:'定位柱'}`，
     且配置里 `name_map` 有 `"定位柱": pillar`、`class_real_size` 有 `pillar: 0.20`
  ⑤ 生产配置里不能再有启用的 stripe/blob 检测器（柱子改由 YOLO 出）

本脚本做 ①②③ 并**校验** ④⑤，任何一项不满足就中止并说明。

用法
----
    # 只检查，不写任何文件
    python3 tools/deploy_model.py --check \
        --model ~/yolo26_robocup/runs/ball_v10/weights/best.onnx

    # 真部署（会改 models/ 与 4 个 perception*.yaml）
    python3 tools/deploy_model.py \
        --model ~/yolo26_robocup/runs/ball_v10/weights/best.onnx \
        --name ball_pillar_20261013.onnx

    # 用对了就提交：
    git add src/rb_perception/models src/rb_perception/config
    git commit -m "feat(perception): 换 ball_v10 三类模型"
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

# 三类模型的固定元数据（ONNX metadata `names`）
EXPECTED_NAMES = {0: "篮球", 1: "排球", 2: "定位柱"}
EXPECTED_INPUT = [1, 3, 640, 640]
EXPECTED_OUTPUT = [1, 7, 8400]

CONFIG_DIR = Path("config")
MODELS_DIR = Path("models")
MANIFEST = "model.json"
ALL_PROFILES = ("perception.yaml", "perception_seek.yaml",
                "perception_rear.yaml", "perception_test.yaml")


# --------------------------------------------------------------------------
# 最小 ONNX 解析：只取 input/output shape 与 metadata names（不依赖 onnx 包）
# --------------------------------------------------------------------------
def _rv(b: bytes, i: int) -> tuple[int, int]:
    r = s = 0
    while True:
        x = b[i]
        i += 1
        r |= (x & 0x7F) << s
        if not x & 0x80:
            return r, i
        s += 7


def _fields(b: bytes):
    i, n = 0, len(b)
    while i < n:
        try:
            k, i = _rv(b, i)
        except IndexError:
            return
        fn, wt = k >> 3, k & 7
        if wt == 0:
            v, i = _rv(b, i)
            yield fn, wt, v
        elif wt == 2:
            ln, i = _rv(b, i)
            yield fn, wt, b[i:i + ln]
            i += ln
        elif wt == 5:
            yield fn, wt, b[i:i + 4]
            i += 4
        elif wt == 1:
            yield fn, wt, b[i:i + 8]
            i += 8
        else:
            return


def _first(b: bytes, want: int, wt_want: int = 2):
    for fn, wt, v in _fields(b):
        if fn == want and wt == wt_want:
            return v
    return None


def _tensor_shape(type_proto: bytes):
    tt = _first(type_proto, 1)          # TypeProto.tensor_type
    if tt is None:
        return None
    sp = _first(tt, 2)                  # TypeProto.Tensor.shape
    if sp is None:
        return None
    out = []
    for fn, wt, v in _fields(sp):
        if fn == 1 and wt == 2:         # Dimension
            dv = _first(v, 1, 0)
            out.append(dv if dv is not None else "?")
    return out


def inspect_onnx(path: Path) -> dict:
    """返回 {'names': {int:str}, 'input': [...], 'output': [...]}"""
    b = path.read_bytes()
    names: dict[int, str] = {}
    for fn, wt, v in _fields(b):
        if fn == 14 and wt == 2:        # metadata_props
            kv = {}
            for f2, w2, v2 in _fields(v):
                if w2 == 2:
                    kv[f2] = v2.decode("utf-8", "replace")
            if kv.get(1) == "names":
                for k, val in re.findall(r"(\d+)\s*:\s*'([^']*)'", kv.get(2, "")):
                    names[int(k)] = val
    io = {}
    graph = _first(b, 7)
    if graph is not None:
        for fn, wt, v in _fields(graph):
            if fn in (11, 12) and wt == 2:
                shp = None
                for f2, w2, v2 in _fields(v):
                    if f2 == 2 and w2 == 2:
                        shp = _tensor_shape(v2)
                io["input" if fn == 11 else "output"] = shp
    return {"names": names, "input": io.get("input"), "output": io.get("output"),
            "sha256": hashlib.sha256(b).hexdigest(), "size": len(b)}


# --------------------------------------------------------------------------
def check_profiles(root: Path, model_name: str) -> list[str]:
    """校验各 perception 配置是否满足契约；返回问题列表。"""
    problems: list[str] = []
    for prof in ALL_PROFILES:
        path = root / CONFIG_DIR / prof
        if not path.is_file():
            problems.append(f"{prof}: 不存在")
            continue
        text = path.read_text(encoding="utf-8")
        # ③ model_path 指向同一个版本化文件
        paths = re.findall(r"^\s*model_path:\s*[\"']?([^\"'\s#]+)", text, re.M)
        if not paths:
            problems.append(f"{prof}: 没有 model_path")
        for p in paths:
            if not p.endswith(model_name):
                problems.append(f"{prof}: model_path 仍指向 {p}（应为 …/{model_name}）")
        # ④ 三类映射
        if '"定位柱"' not in text and "定位柱" not in text:
            problems.append(f"{prof}: name_map 缺 \"定位柱\"")
        if not re.search(r"pillar:\s*pillar", text):
            problems.append(f"{prof}: name_map 缺 pillar: pillar")
        if not re.search(r"pillar:\s*0?\.20", text):
            problems.append(f"{prof}: class_real_size 缺 pillar: 0.20")
        # ⑤ 生产配置里 stripe/blob 必须全关
        if prof != "perception_test.yaml":
            for m in re.finditer(r"type:\s*(stripe|blob)\b(.*?)(?=\n\s*- label:|\Z)",
                                 text, re.S):
                block = m.group(2)
                em = re.search(r"enabled:\s*(true|false)", block)
                if em and em.group(1) == "true":
                    problems.append(f"{prof}: 还有启用的 {m.group(1)} 检测器（柱子应由 YOLO 出）")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description="部署新的 ONNX 视觉模型到 rb_perception")
    ap.add_argument("--model", required=True, help="新的 .onnx 路径")
    ap.add_argument("--name", default=None,
                    help="部署后的文件名（默认沿用当前 model.json 的名字加 _v2）")
    ap.add_argument("--check", action="store_true", help="只检查，不写文件")
    ap.add_argument("--force", action="store_true", help="跳过元数据不符的拦截")
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1] / "src" / "rb_perception"
    if not root.is_dir():
        print(f"❌ 找不到 {root}", file=sys.stderr)
        return 2

    model = Path(args.model).expanduser().resolve()
    if not model.is_file():
        print(f"❌ 模型不存在：{model}", file=sys.stderr)
        return 2

    info = inspect_onnx(model)
    print(f"模型 {model}")
    print(f"  sha256 : {info['sha256']}")
    print(f"  大小   : {info['size']} 字节 ({info['size']/1048576:.2f} MB)")
    print(f"  names  : {info['names']}")
    print(f"  input  : {info['input']}")
    print(f"  output : {info['output']}")

    # ①/④ 元数据契约
    bad = []
    if info["names"] != EXPECTED_NAMES:
        bad.append(f"类别名不符：{info['names']} ≠ {EXPECTED_NAMES}")
    if info["input"] != EXPECTED_INPUT:
        bad.append(f"输入形状不符：{info['input']} ≠ {EXPECTED_INPUT}")
    if info["output"] != EXPECTED_OUTPUT:
        bad.append(f"输出形状不符：{info['output']} ≠ {EXPECTED_OUTPUT}")
    if bad:
        print("\n⚠️ 元数据不符：", file=sys.stderr)
        for m in bad:
            print(f"   · {m}", file=sys.stderr)
        if not args.force:
            print("\n拒绝部署（要强行部署加 --force，但要先确认配置里的 name_map / "
                  "class_real_size / input_size 都跟着改）", file=sys.stderr)
            return 1

    # ② 现有 manifest
    man_path = root / MODELS_DIR / MANIFEST
    old_man = json.loads(man_path.read_text(encoding="utf-8")) if man_path.is_file() else {}
    old_file = old_man.get("file")
    print(f"\n当前部署：{old_file}  sha256={old_man.get('sha256','?')[:16]}…")

    if old_man.get("sha256") == info["sha256"]:
        print("✅ 内容与当前部署完全相同（哈希一致），无需替换。")
        return 0

    name = args.name or (old_file or "model.onnx")
    if name != old_file and (root / MODELS_DIR / name).exists():
        print(f"❌ {name} 已存在且内容不同，换个 --name", file=sys.stderr)
        return 1

    print(f"\n将要：")
    print(f"  ① 写入 {MODELS_DIR}/{name}")
    print(f"  ② 更新 {MODELS_DIR}/{MANIFEST}")
    print(f"  ③ 把 {len(ALL_PROFILES)} 个配置的 model_path 改成 ../models/{name}")

    if args.check:
        print("\n（--check：未写任何文件）")
        # 顺便报告当前配置的契约问题
        probs = check_profiles(root, old_file or "")
        if probs:
            print(f"\n⚠️ 当前配置有 {len(probs)} 处不符契约：")
            for p in probs:
                print(f"   · {p}")
        return 0

    # ① 部署模型
    (root / MODELS_DIR).mkdir(parents=True, exist_ok=True)
    shutil.copy2(model, root / MODELS_DIR / name)
    got = hashlib.sha256((root / MODELS_DIR / name).read_bytes()).hexdigest()
    if got != info["sha256"]:
        print(f"❌ 复制后哈希不一致：{got}", file=sys.stderr)
        return 1
    print(f"  ✅ 已写入 {name}（哈希校验一致）")

    # ② 更新 manifest
    manifest = {
        "file": name,
        "sha256": info["sha256"],
        "classes": {str(k): v for k, v in sorted(info["names"].items())},
        "input": info["input"],
        "output": info["output"],
        "export_date": dt.datetime.fromtimestamp(
            model.stat().st_mtime).astimezone().isoformat(),
    }
    man_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
    print(f"  ✅ 已更新 {MANIFEST}")

    # ③ 改各配置的 model_path（只改这一行，保留注释）
    changed = 0
    for prof in ALL_PROFILES:
        path = root / CONFIG_DIR / prof
        if not path.is_file():
            print(f"  ⚠️ {prof} 不存在，跳过")
            continue
        text = path.read_text(encoding="utf-8")
        new_text, n = re.subn(
            r"(^\s*model_path:\s*[\"']?)[^\"'\s#]+([\"']?\s*(?:#.*)?$)",
            lambda m: f"{m.group(1)}../models/{name}{m.group(2)}",
            text, flags=re.M)
        if n and new_text != text:
            path.write_text(new_text, encoding="utf-8")
            changed += 1
            print(f"  ✅ {prof}: model_path → ../models/{name}")
        elif n:
            print(f"  · {prof}: 已经是 {name}")
        else:
            print(f"  ⚠️ {prof}: 没找到 model_path")

    # ④⑤ 复查
    probs = check_profiles(root, name)
    print()
    if probs:
        print(f"⚠️ 仍有 {len(probs)} 处不符契约（模型已换，但配置要跟进）：")
        for p in probs:
            print(f"   · {p}")
        print("\n对照 src/rb_perception/test/test_deployed_model.py 修完后，跑：")
        print("   colcon test --packages-select rb_perception")
        return 1

    print("✅ 契约全部满足。接着：")
    print("   colcon build --symlink-install --packages-select rb_perception")
    print("   colcon test  --packages-select rb_perception")
    print(f"   git add src/rb_perception/models src/rb_perception/config")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
