#!/usr/bin/env python3
"""视觉颜色标定：对着你们的**真球**采样，生成能用的 HSV 阈值。

为什么需要
----------
`perception.yaml` 里的 hsv_ranges 是通用占位值（"典型橙色篮球"）。
2026 规则是**各队自带用球**，颜色图案不统一，必须按你们的球重新采。

用法
----
    # 实时采样（推荐）：空格=冻结画面并框选球，可多采几次，c=计算并写回，q=退出
    python3 tools/calibrate_color.py --label ball_basketball

    # 从已有图片 + 指定框
    python3 tools/calibrate_color.py --image shot.jpg --rect 600,300,140,140 \
        --label ball_volleyball

    # 只看结果不写回
    python3 tools/calibrate_color.py --label ball_basketball --dry-run

产物
----
  * 写回 src/rb_perception/config/perception.yaml 的对应 hsv_ranges（保留注释）
  * 备份原文件到 perception.yaml.bak
  * 生成对照图 test_artifacts/color_calib_<label>.png（原图 | 掩码 | 叠加）
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
PERCEPTION_YAML = WS / "src" / "rb_perception" / "config" / "perception.yaml"
sys.path.insert(0, str(WS / "src" / "rb_perception"))


# ---------------------------------------------------------------------------
# HSV 范围推导
# ---------------------------------------------------------------------------
def _circular_smooth(hist: np.ndarray, k: int = 3) -> np.ndarray:
    """在色相环上做平滑（H 是 0..179 的环，0 和 179 是相邻的）。"""
    if k <= 1:
        return hist
    kernel = np.ones(2 * k + 1) / (2 * k + 1)
    padded = np.concatenate([hist[-k:], hist, hist[:k]])
    return np.convolve(padded, kernel, mode="same")[k:-k]


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """在**环**上找连续 True 段，返回 [(lo, hi), ...]（闭区间，允许跨 179/0）。"""
    n = len(mask)
    if mask.all():
        return [(0, n - 1)]
    if not mask.any():
        return []
    # 找到一段 False 作为起点，避免把环切断
    start = int(np.argmin(mask))
    out, i = [], 0
    idx = (np.arange(n) + start) % n
    while i < n:
        if mask[idx[i]]:
            j = i
            while j + 1 < n and mask[idx[j + 1]]:
                j += 1
            out.append((int(idx[i]), int(idx[j])))
            i = j + 1
        else:
            i += 1
    return out


def _in_circular(x: np.ndarray, lo: int, hi: int) -> np.ndarray:
    if lo <= hi:
        return (x >= lo) & (x <= hi)
    return (x >= lo) | (x <= hi)


def _roi_to_hsv_samples(roi_bgr: np.ndarray, shape: str = "ellipse",
                        inset: float = 0.10, ellipse_scale: float = 0.92) -> np.ndarray:
    """把 ROI 转成"只属于目标"的 HSV 像素。

    ⚠️ 为什么不能直接用整个矩形 ROI：球是**圆的**，矩形的四角是背景。
    本项目实测踩到过——框住橙色篮球后，四角的深灰背景被算成第二个色相簇，
    推导出 [174, 0, 30, 6, ...] 这种 S≥0 的区间，几乎匹配一切，纯粹是误检源。
    所以默认取**内接椭圆**（球在矩形里正好内接），只采目标本身的像素。
    """
    h, w = roi_bgr.shape[:2]
    mask = np.zeros((h, w), np.uint8)
    if shape == "ellipse":
        cv2.ellipse(mask, (w // 2, h // 2),
                    (max(1, int(w / 2 * ellipse_scale)), max(1, int(h / 2 * ellipse_scale))),
                    0, 0, 360, 255, -1)
    else:
        m = max(1, int(min(w, h) * inset))
        cv2.rectangle(mask, (m, m), (w - m, h - m), 255, -1)
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    px = hsv[mask > 0]
    return px.reshape(-1, 1, 3)


def derive_ranges(roi_hsv: np.ndarray, coverage: float = 0.95,
                  max_ranges: int = 3) -> tuple[list[list[int]], str]:
    """从 ROI 的 HSV 像素推导 1~N 段 HSV 范围。返回 (ranges, 说明)。"""
    H = roi_hsv[..., 0].ravel().astype(np.int32)
    S = roi_hsv[..., 1].ravel().astype(np.int32)
    V = roi_hsv[..., 2].ravel().astype(np.int32)
    n = max(1, len(H))

    # 低饱和（白/灰球）：色相没有意义，改用"S 低 + V 高"的框
    med_s = float(np.median(S))
    if med_s < 40:
        s_hi = int(np.clip(np.percentile(S, 99) + 15, 20, 255))
        v_lo = int(np.clip(np.percentile(V, 2) - 25, 0, 255))
        return ([[0, 0, v_lo, 179, s_hi, 255]],
                f"低饱和目标（中位 S={med_s:.0f}）→ 用 S≤{s_hi}、V≥{v_lo} 的框，色相全范围")

    hist = _circular_smooth(np.bincount(H, minlength=180).astype(float), 3)
    thr = hist.max() * 0.12
    bins = np.arange(180)
    runs = _runs(hist >= thr)

    # 丢掉"质量远小于主簇"的杂簇：这是 ROI 里混入背景的主要防线。
    # 阈值取主簇的 25% —— 排球那种多色块（蓝/黄各占约一半）不会被误删。
    if runs:
        masses = [float(hist[_in_circular(bins, *r)].sum()) for r in runs]
        dom = max(masses)
        runs = [r for r, m in zip(runs, masses) if m >= 0.25 * dom and m / n > 0.02]
        dropped = len(masses) - len(runs)
    else:
        dropped = 0
    if not runs:
        runs = [(int(np.percentile(H, 2)), int(np.percentile(H, 98)))]

    # 按质量从大到小取，凑够 coverage
    runs.sort(key=lambda r: float(hist[_in_circular(bins, *r)].sum()), reverse=True)
    chosen, acc = [], 0.0
    for r in runs:
        chosen.append(r)
        acc += float(hist[_in_circular(bins, *r)].sum()) / n
        if acc >= coverage or len(chosen) >= max_ranges:
            break
    chosen.sort(key=lambda r: r[0])

    ranges: list[list[int]] = []
    for lo, hi in chosen:
        sel = _in_circular(H, lo, hi)
        if not sel.any():
            continue
        # S/V 下界留 margin 以容纳阴影侧；上界开到 255（高光/反光不该被排除）
        s_lo = int(np.clip(np.percentile(S[sel], 2) - 30, 0, 255))
        v_lo = int(np.clip(np.percentile(V[sel], 2) - 30, 0, 255))
        ranges.append([int((lo - 3) % 180), s_lo, v_lo, int((hi + 3) % 180), 255, 255])

    note = f"取 {len(ranges)} 段色相区间（目标覆盖率 {coverage:.0%}）"
    if dropped:
        note += f"，已丢弃 {dropped} 个疑似背景杂簇"
    if len(ranges) > 1:
        note += "；多段说明球上有多块颜色（如排球），HSV 方案天然吃力，建议上 YOLO"
    return ranges, note


def mask_from_ranges(frame_bgr: np.ndarray, ranges: list[list[int]]) -> np.ndarray:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for item in ranges:
        if len(item) != 6:
            continue
        lo = np.array(item[:3], dtype=np.uint8)
        hi = np.array(item[3:], dtype=np.uint8)
        mask |= cv2.inRange(hsv, lo, hi)
    return mask


# ---------------------------------------------------------------------------
# 写回 YAML（按行替换 hsv_ranges 列表，保留注释）
# ---------------------------------------------------------------------------
def _replace_hsv_ranges(text: str, label: str, ranges: list[list[int]]) -> str:
    lines = text.splitlines()
    label_re = re.compile(rf"^(\s*)-\s*label:\s*{re.escape(label)}\s*(#.*)?$")
    hr_re = re.compile(r"^(\s*)hsv_ranges:\s*(.*)$")
    next_label_re = re.compile(r"^\s*-\s*label:")

    start = next((i for i, l in enumerate(lines) if label_re.match(l)), None)
    if start is None:
        raise SystemExit(f"✗ 在 {PERCEPTION_YAML.name} 里找不到 label: {label}")

    j = None
    for i in range(start + 1, len(lines)):
        if next_label_re.match(lines[i]):
            break
        if hr_re.match(lines[i]):
            j = i
            break
    if j is None:
        raise SystemExit(f"✗ label {label} 下没有 hsv_ranges 键")

    m = hr_re.match(lines[j])
    indent, rest = m.group(1), m.group(2)
    item_indent = indent + "  "
    # 保留行尾注释
    comment = ""
    if "#" in rest:
        comment = "  " + rest[rest.index("#"):]
    elif rest.strip():
        comment = ""

    # 删掉原有的列表项
    k = j + 1
    while k < len(lines) and re.match(rf"^{re.escape(item_indent)}-\s*\[", lines[k]):
        k += 1

    new_items = [
        f"{item_indent}- [{r[0]}, {r[1]}, {r[2]}, {r[3]}, {r[4]}, {r[5]}]"
        for r in ranges
    ]
    out = lines[:j] + [f"{indent}hsv_ranges:{comment}"] + new_items + lines[k:]
    return "\n".join(out) + ("\n" if text.endswith("\n") else "")


def write_ranges(label: str, ranges: list[list[int]], dry_run: bool) -> None:
    text = PERCEPTION_YAML.read_text(encoding="utf-8")
    new_text = _replace_hsv_ranges(text, label, ranges)
    if dry_run:
        print(f"\n(--dry-run，未写回。下面是 {label} 段替换后的结果)")
        lines = new_text.splitlines()
        lab_re = re.compile(rf"^\s*-\s*label:\s*{re.escape(label)}\s*(#.*)?$")
        start = next((i for i, l in enumerate(lines) if lab_re.match(l)), None)
        if start is not None:
            print("  " + lines[start])
            for l in lines[start + 1:]:
                if re.match(r"^\s*-\s*label:", l):
                    break
                print("  " + l)
        return
    bak = PERCEPTION_YAML.with_suffix(".yaml.bak")
    if not bak.exists():
        shutil.copy2(PERCEPTION_YAML, bak)
        print(f"  ✓ 已备份原文件 → {bak.name}")
    PERCEPTION_YAML.write_text(new_text, encoding="utf-8")
    print(f"  ✓ 已写入 {PERCEPTION_YAML.relative_to(WS)} 的 {label}.hsv_ranges")


# ---------------------------------------------------------------------------
# 采样
# ---------------------------------------------------------------------------
def sample_from_live(device: str, width: int, height: int,
                     shape: str = "ellipse") -> tuple[np.ndarray | None, list[np.ndarray]]:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"✗ 打不开相机 {device}（rb_camera 还在跑？先停掉它）")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    rois: list[np.ndarray] = []
    win = "calibrate_color  SPACE=sample(框选球)  c=apply  q=quit"
    print("\n把球放进取景框：空格冻结画面 → 拖框框住球 → 回车。")
    print("建议采 3~5 次（不同角度、亮面/暗面、不同距离）。c=计算并写回，q=退出\n")
    last_frame = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            last_frame = frame
            vis = frame.copy()
            cv2.putText(vis, f"samples: {len(rois)}   SPACE=sample  c=apply  q=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(win, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                if rois:
                    break
                print("  还没采样，先按空格框选球")
            if key == ord(" "):
                r = cv2.selectROI("select ball (ENTER ok, c cancel)", frame, showCrosshair=False)
                cv2.destroyWindow("select ball (ENTER ok, c cancel)")
                x, y, w, h = (int(v) for v in r)
                if w > 4 and h > 4:
                    s = _roi_to_hsv_samples(frame[y:y + h, x:x + w], shape)
                    rois.append(s)
                    print(f"  已采样 {len(rois)} 个（框 {w}x{h}，取 {s.shape[0]} 像素，{shape}）")
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:  # noqa: BLE001
                pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return last_frame, rois


def sample_from_image(path: str, rect: str, shape: str = "ellipse") -> tuple[np.ndarray, list[np.ndarray]]:
    frame = cv2.imread(path)
    if frame is None:
        raise SystemExit(f"✗ 读不到图片 {path}")
    if rect:
        try:
            x, y, w, h = (int(v) for v in rect.split(","))
        except Exception:  # noqa: BLE001
            raise SystemExit("✗ --rect 格式应为 x,y,w,h")
    else:
        r = cv2.selectROI("select ball", frame, showCrosshair=False)
        cv2.destroyWindow("select ball")
        x, y, w, h = (int(v) for v in r)
    if w <= 4 or h <= 4:
        raise SystemExit("✗ 没框选到有效区域")
    print(f"  ROI: x={x} y={y} w={w} h={h}（采样形状={shape}）")
    return frame, [_roi_to_hsv_samples(frame[y:y + h, x:x + w], shape)]


# ---------------------------------------------------------------------------
def validate(frame: np.ndarray, ranges: list[list[int]], label: str) -> None:
    """用推导出的阈值：① 跑真检测器 ② 出对照图，肉眼确认误检。"""
    mask = mask_from_ranges(frame, ranges)
    h, w = frame.shape[:2]
    cover = float((mask > 0).sum()) / (h * w) * 100

    # 用工程里真正的检测器跑一遍，这才是端到端可信的验证
    try:
        from rb_perception.detectors import BlobDetector, CameraModel
        cfg = {"hsv_ranges": ranges, "min_area_ratio": 0.0006, "max_area_ratio": 0.2,
               "aspect_range": [0.5, 2.0], "circularity_min": 0.4, "real_diameter_m": 0.22}
        det = BlobDetector(label, cfg)
        cam = CameraModel.from_config({}, w, h)
        found = det.detect(frame, cam)
        print(f"  用真检测器({label}) 跑一遍: 检出 {len(found)} 个", end="")
        if found:
            best = found[0]
            print(f"，最高置信 {best.confidence:.2f}，方位 {np.degrees(best.bearing_rad):+.1f}°，"
                  f"距离 {best.distance_m:.2f}m")
        else:
            print("  ⚠️ 没检出——框选太小/太偏，或多采几个角度")
    except Exception as exc:  # noqa: BLE001
        print(f"  (真检测器验证跳过: {exc})")

    print(f"  掩码占全图 {cover:.2f}%（越低越干净；>8% 通常说明把地板/墙也框进来了）")

    vis_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    overlay = frame.copy()
    overlay[mask > 0] = (0, 255, 0)
    vis = np.hstack([frame, vis_mask, overlay])
    out = WS / "test_artifacts" / f"color_calib_{label}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), vis)
    print(f"  对照图(原图|掩码|叠加): {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description="HSV 颜色标定")
    ap.add_argument("--label", required=True,
                    help="要标定的类别，如 ball_basketball / ball_volleyball / hoop")
    ap.add_argument("--image", default="", help="用图片而不是实时相机")
    ap.add_argument("--rect", default="", help="配合 --image：x,y,w,h")
    ap.add_argument("--device", default="", help="默认从 camera.yaml 读")
    ap.add_argument("--width", type=int, default=0)
    ap.add_argument("--height", type=int, default=0)
    ap.add_argument("--coverage", type=float, default=0.95, help="色相簇覆盖率，默认 0.95")
    ap.add_argument("--max-ranges", type=int, default=3, help="最多输出几段，默认 3")
    ap.add_argument("--shape", default="ellipse", choices=["ellipse", "rect"],
                    help="ROI 内采样形状：ellipse=内接椭圆（球用这个，默认），rect=内缩矩形")
    ap.add_argument("--dry-run", action="store_true", help="只显示不写回")
    args = ap.parse_args()

    # 默认用生产分辨率（和 camera.yaml 一致）
    dev, w0, h0 = "/dev/video0", 1280, 720
    try:
        c = (yaml.safe_load((WS / "src/rb_camera/config/camera.yaml").read_text(encoding="utf-8")) or {}).get("camera", {})
        dev, w0, h0 = str(c.get("device", dev)), int(c.get("width", w0)), int(c.get("height", h0))
    except Exception:  # noqa: BLE001
        pass
    dev = args.device or dev
    w = args.width or w0
    h = args.height or h0

    if args.image:
        frame, rois = sample_from_image(args.image, args.rect, args.shape)
    else:
        print(f"实时采样: {dev} {w}x{h}")
        frame, rois = sample_from_live(dev, w, h, args.shape)

    if not rois:
        print("✗ 没有采样数据")
        return 1
    if frame is None:
        print("✗ 没有参考帧")
        return 1

    all_px = np.concatenate([r.reshape(-1, 3) for r in rois], axis=0).reshape(-1, 1, 3)
    ranges, note = derive_ranges(all_px, args.coverage, args.max_ranges)

    print(f"\n用 {len(rois)} 个样本推导 HSV 阈值（{note}）:")
    for r in ranges:
        print(f"  - [{r[0]}, {r[1]}, {r[2]}, {r[3]}, {r[4]}, {r[5]}]")
    print()
    validate(frame, ranges, args.label)
    print()
    write_ranges(args.label, ranges, args.dry_run)

    if not args.dry_run:
        print("\n生效: colcon build --symlink-install --packages-select rb_perception")
        print("提示: 想回退就 cp src/rb_perception/config/perception.yaml.bak \\")
        print("                        src/rb_perception/config/perception.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
