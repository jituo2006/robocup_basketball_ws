#!/usr/bin/env python3
"""相机固定项现场调参：拖动滑块实时看效果，满意后一键写回 camera.yaml。

为什么需要它
------------
锁定自动曝光/白平衡是"让颜色标定长期有效"的关键。但本机实测这台相机在自动
模式下 `exposure_time_absolute` / `white_balance_temperature` 带 flags=inactive，
**读不到相机自动算出的值**，所以必须人工定一个值 —— 这个工具就是干这个的。

用法
----
    cd ~/robocup_basketball_ws && source install/setup.bash
    python3 tools/tune_camera.py            # 先停掉 rb_camera（同一设备不能两个进程）

操作
----
    拖滑块      实时生效（曝光/白平衡会自动切到手动模式）
    s           把当前所有值写回 src/rb_camera/config/camera.yaml
    a           在"自动曝光"和"手动曝光"之间切换（用来对比自动的效果）
    r           复位到 camera.yaml 里的值
    q / ESC     退出

调参目标
--------
    * 画面均值亮度落在 **80~160**（太暗检不到球，太亮会过曝丢细节）
    * R/G/B 三个均值尽量接近（偏得厉害说明白平衡不对，调 white_balance_temperature）
    * 在**真实场地、真实灯光**下调，并把球放到场内不同位置确认都能看清
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
CAMERA_YAML = WS / "src" / "rb_camera" / "config" / "camera.yaml"
sys.path.insert(0, str(WS / "tools"))
from calibrate_camera import _replace_mapping_values  # noqa: E402  （已测过的按行替换，保留注释）

# 与 camera_node.py 保持一致的别名表
ALIASES = {
    "exposure": ["exposure_time_absolute", "exposure_absolute"],
    "white_balance_temperature": ["white_balance_temperature"],
    "gain": ["gain"],
    "brightness": ["brightness"],
    "contrast": ["contrast"],
    "saturation": ["saturation"],
    "sharpness": ["sharpness"],
}
SLIDERS = ["exposure", "white_balance_temperature", "gain",
           "brightness", "contrast", "saturation", "sharpness"]
WIN = "tune_camera   s=save  a=auto/manual  r=reset  q=quit"


def list_ctrls(dev: str) -> dict[str, dict]:
    out = subprocess.run(["v4l2-ctl", "-d", dev, "--list-ctrls"],
                         capture_output=True, text=True).stdout
    ctrls: dict[str, dict] = {}
    for line in out.splitlines():
        m = re.match(r"\s*([a-z_0-9]+)\s+0x[0-9a-fA-F]+\s+\(\w+\)\s*:(.*)", line)
        if not m:
            continue
        d = {}
        for k, v in re.findall(r"(\w+)=(-?\d+)", m.group(2)):
            d[k] = int(v)
        ctrls[m.group(1)] = d
    return ctrls


def resolve(friendly: str, ctrls: dict) -> str | None:
    for c in ALIASES.get(friendly, []):
        if c in ctrls:
            return c
    return None


def set_ctrl(dev: str, name: str, value: int) -> bool:
    r = subprocess.run(["v4l2-ctl", "-d", dev, f"--set-ctrl={name}={int(value)}"],
                       capture_output=True, text=True)
    return r.returncode == 0


def get_ctrl(dev: str, name: str) -> int | None:
    o = subprocess.run(["v4l2-ctl", "-d", dev, "--get-ctrl", name],
                       capture_output=True, text=True).stdout
    m = re.search(r"[:=]\s*(-?\d+)", o)
    return int(m.group(1)) if m else None


def load_cfg() -> dict:
    try:
        return yaml.safe_load(CAMERA_YAML.read_text(encoding="utf-8")) or {}
    except Exception:  # noqa: BLE001
        return {}


def main() -> int:
    ap = argparse.ArgumentParser(description="相机固定项现场调参")
    ap.add_argument("--device", default="")
    ap.add_argument("--no-window", action="store_true", help="只打印当前控件值后退出")
    args = ap.parse_args()

    cfg = load_cfg()
    cam = cfg.get("camera", {}) or {}
    ctl = cfg.get("controls", {}) or {}
    dev = args.device or str(cam.get("device", "/dev/video0"))
    w, h = int(cam.get("width", 1280)), int(cam.get("height", 720))

    if not os.path.exists(dev):
        print(f"✗ {dev} 不存在（相机没插？）")
        return 1

    ctrls = list_ctrls(dev)
    if not ctrls:
        print(f"✗ 读不到 {dev} 的 V4L2 控件（装 v4l-utils？被别的进程占用？）")
        return 1

    if args.no_window:
        for f in SLIDERS:
            n = resolve(f, ctrls)
            if n:
                print(f"  {f:<26} ({n}) = {get_ctrl(dev, n)}  范围 {ctrls[n].get('min')}..{ctrls[n].get('max')}")
        return 0

    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("✗ 没有图形界面（SSH 里请加 -X，或在本机桌面上跑）")
        return 2

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"✗ 打不开 {dev}（rb_camera 还在跑？同一设备不能被两个进程同时用）")
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # 曝光/白平衡先切手动，否则拖滑块没反应（这是最容易踩的坑）
    if "auto_exposure" in ctrls:
        set_ctrl(dev, "auto_exposure", 1)
    if "white_balance_automatic" in ctrls:
        set_ctrl(dev, "white_balance_automatic", 0)

    # 建窗口 + 滑块（OpenCV 滑块只能是 0..N，所以用 offset 映射真实范围）
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    ranges: dict[str, tuple[str, int, int]] = {}
    pending: dict[str, int] = {}
    touched: set[str] = set()

    def make_cb(friendly: str):
        def cb(v):
            pending[friendly] = int(v)
            touched.add(friendly)
        return cb

    for friendly in SLIDERS:
        name = resolve(friendly, ctrls)
        if not name:
            print(f"  · 相机没有 {friendly} 控件，跳过")
            continue
        lo = int(ctrls[name].get("min", 0))
        hi = int(ctrls[name].get("max", 255))
        cur = get_ctrl(dev, name)
        if cur is None:
            cur = int(ctrls[name].get("value", lo))
        cur = max(lo, min(hi, cur))
        ranges[friendly] = (name, lo, hi)
        cv2.createTrackbar(friendly, WIN, cur - lo, hi - lo, make_cb(friendly))

    print(f"\n调参中（{dev} {w}x{h}）。目标：亮度 80~160、R/G/B 接近。")
    print("s=写回 camera.yaml   a=自动/手动曝光切换   r=复位   q=退出\n")

    auto_mode = False
    last_apply = 0.0
    info = ""
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            now = time.time()
            # 滑块改动节流到 ~10Hz 应用，拖动才顺滑（每次要 spawn v4l2-ctl）
            if pending and now - last_apply > 0.1:
                for friendly, val in list(pending.items()):
                    name, lo, _ = ranges[friendly]
                    set_ctrl(dev, name, lo + val)
                pending.clear()
                last_apply = now

            b, g, r = (float(frame[:, :, i].mean()) for i in range(3))
            lum = float(frame.mean())
            bal = "中性" if max(abs(r - g), abs(g - b), abs(r - b)) < 18 else (
                "偏红" if r > max(g, b) else "偏蓝" if b > max(g, r) else "偏绿")
            vis = frame.copy()
            tips = [f"亮度 {lum:.0f}  {'✅' if 80 <= lum <= 160 else '⚠️ 调到 80~160'}",
                    f"R/G/B {r:.0f}/{g:.0f}/{b:.0f}  {bal}",
                    f"曝光模式 {'自动' if auto_mode else '手动'}   " + (info or "") ]
            for i, t in enumerate(tips):
                cv2.putText(vis, t, (10, 28 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(vis, t, (10, 28 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(WIN, vis)

            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("a"):
                if "auto_exposure" in ctrls:
                    auto_mode = not auto_mode
                    set_ctrl(dev, "auto_exposure", 3 if auto_mode else 1)
                    info = "已切自动曝光（对比用）" if auto_mode else "已切回手动"
            if key == ord("r"):
                for friendly, (name, lo, hi) in ranges.items():
                    want = ctl.get(friendly, None)
                    if want is None:
                        continue
                    want = max(lo, min(hi, int(want)))
                    if set_ctrl(dev, name, want):
                        cv2.setTrackbarPos(friendly, WIN, want - lo)
                print("  已复位到 camera.yaml 的值")
            if key == ord("s"):
                if auto_mode:
                    print("  ⚠️ 当前是自动曝光，先按 a 切回手动再保存（否则值没意义）")
                else:
                    vals = {}
                    for friendly, (name, lo, hi) in ranges.items():
                        cur = get_ctrl(dev, name)
                        # 只写曝光/白平衡，以及用户真正拖过的其它项
                        if friendly in ("exposure", "white_balance_temperature") or friendly in touched:
                            vals[friendly] = int(cur) if cur is not None else lo
                    if vals:
                        text = CAMERA_YAML.read_text(encoding="utf-8")
                        text = _replace_mapping_values(text, "controls", vals)
                        CAMERA_YAML.write_text(text, encoding="utf-8")
                        print(f"  ✓ 已写回 {CAMERA_YAML.relative_to(WS)}: {vals}")
                        info = "已保存"
                    else:
                        print("  ⚠️ 没有可保存的值")
            try:
                if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:  # noqa: BLE001
                pass
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print("提示: 重新 build 后生效: colcon build --symlink-install --packages-select rb_camera")
    return 0


if __name__ == "__main__":
    sys.exit(main())
