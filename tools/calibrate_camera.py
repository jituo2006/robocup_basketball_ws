#!/usr/bin/env python3
"""相机内参标定：把 fx/fy/cx/cy 标出来，单目测距才可信。

为什么要标定
------------
`Detection.distance_m` 用 `d = fx × 真实直径 / 像素宽` 算。fx 不准则距离不准，
进而影响：抓球距离判定、避障排斥半径、YOLO 距离估计。本工程未标定时按
assumed_hfov=60° 猜一个 fx，能跑但误差可观。

⚠️ 必须在**生产分辨率**下标定（默认从 camera.yaml 读，1280x720）。
   换了分辨率就得重标，内参不通用。

用法
----
    # 0) 先生成可打印棋盘格（贴平、别缩放打印，用"实际大小/100%"）
    python3 tools/calibrate_camera.py --print-board --out /tmp/chessboard.png

    # 1) 对着板子拍 15~25 张（覆盖四角、倾斜、远近）
    python3 tools/calibrate_camera.py                      # 实时采集
    #    窗口里：空格=拍一张（只在检出棋盘时有效）  c=开始计算  u=撤销  q=退出

    # 2) 也可以用已有的图片
    python3 tools/calibrate_camera.py --images ~/calib_shots

结果会**同时**写进两个配置（测距读的是 perception 那份）：
    src/rb_camera/config/camera.yaml      → intrinsics
    src/rb_perception/config/perception.yaml → camera

写回采用**按行替换**，不会破坏文件里的注释。
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

WS = Path(__file__).resolve().parent.parent
CAMERA_YAML = WS / "src" / "rb_camera" / "config" / "camera.yaml"
PERCEPTION_YAML = WS / "src" / "rb_perception" / "config" / "perception.yaml"


# ---------------------------------------------------------------------------
# 棋盘格生成
# ---------------------------------------------------------------------------
def make_board_image(cols: int, rows: int, square_mm: float, dpi: int, paper: str) -> np.ndarray:
    """生成可 1:1 打印的棋盘格。

    cols/rows 是**内角点**数（OpenCV 的 pattern_size），所以方格数是 (cols+1)×(rows+1)。
    """
    sizes = {"A4": (210.0, 297.0), "A3": (297.0, 420.0), "Letter": (215.9, 279.4)}
    pw, ph = sizes.get(paper, sizes["A4"])
    px_per_mm = dpi / 25.4
    canvas = np.full((int(ph * px_per_mm), int(pw * px_per_mm)), 255, np.uint8)

    sq = int(round(square_mm * px_per_mm))
    bw, bh = (cols + 1) * sq, (rows + 1) * sq
    if bw > canvas.shape[1] or bh > canvas.shape[0]:
        raise SystemExit(
            f"✗ {cols}x{rows} 内角点 × {square_mm}mm = {bw / px_per_mm:.0f}x{bh / px_per_mm:.0f}mm，"
            f"放不进 {paper}（{pw:.0f}x{ph:.0f}mm）。请调小 --square 或改用 --paper A3")

    x0, y0 = (canvas.shape[1] - bw) // 2, (canvas.shape[0] - bh) // 2
    # 交替黑白方格
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                canvas[y0 + r * sq:y0 + (r + 1) * sq, x0 + c * sq:x0 + (c + 1) * sq] = 0
    # 一圈细边框便于裁剪
    cv2.rectangle(canvas, (x0 - 3, y0 - 3), (x0 + bw + 3, y0 + bh + 3), 128, 2)
    return canvas


# ---------------------------------------------------------------------------
# 角点检测
# ---------------------------------------------------------------------------
def find_corners(gray: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    """返回 (N,1,2) 的亚像素角点；找不到返回 None。"""
    # 首选 SB 版本（更快更稳），OpenCV 老版本没有就退回经典版
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(
            gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return corners.reshape(-1, 1, 2)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


# ---------------------------------------------------------------------------
# 采集
# ---------------------------------------------------------------------------
def load_from_images(folder: str, pattern: tuple[int, int]) -> tuple[list, list, tuple[int, int]]:
    files = sorted(glob.glob(os.path.join(folder, "*")))
    files = [f for f in files if f.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"))]
    if not files:
        raise SystemExit(f"✗ {folder} 下没有图片")
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)

    objpoints, imgpoints, shape = [], [], None
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        c = find_corners(gray, pattern)
        if c is None:
            print(f"  ✗ 未检出棋盘: {os.path.basename(f)}")
            continue
        objpoints.append(objp)
        imgpoints.append(c)
        print(f"  ✓ {os.path.basename(f)}")
    return objpoints, imgpoints, shape or (0, 0)


def capture_live(device: str, width: int, height: int, pattern: tuple[int, int]):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"✗ 打不开相机 {device}（是不是 rb_camera 还在跑？先停掉它）")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)

    objpoints, imgpoints = [], []
    shape = (0, 0)
    win = "calibrate_camera  SPACE=capture  c=compute  u=undo  q=quit"
    print("\n把棋盘格放进取景框：远/近、四角、左右倾斜各拍几张，共 15~25 张。")
    print("空格=拍一张   c=计算   u=撤销上一张   q=退出\n")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            shape = frame.shape[1], frame.shape[0]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            c = find_corners(gray, pattern)
            vis = frame.copy()
            if c is not None:
                cv2.drawChessboardCorners(vis, pattern, c, True)
                cv2.putText(vis, "FOUND - press SPACE", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(vis, "searching board...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2, cv2.LINE_AA)
            cv2.putText(vis, f"captured: {len(objpoints)}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(win, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            if key == ord("u") and objpoints:
                objpoints.pop()
                imgpoints.pop()
                print(f"  撤销一张，剩 {len(objpoints)}")
            if key == ord(" ") and c is not None:
                objpoints.append(objp)
                imgpoints.append(c)
                print(f"  已拍 {len(objpoints)} 张")
            if key == ord("c"):
                if len(objpoints) >= 6:
                    break
                print(f"  还太少（{len(objpoints)} 张），至少 6 张，建议 15 张以上")
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:  # noqa: BLE001
                pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return objpoints, imgpoints, shape


# ---------------------------------------------------------------------------
# 写回 YAML（按行替换，保留注释）
# ---------------------------------------------------------------------------
def _replace_mapping_values(text: str, block: str, values: dict[str, float]) -> str:
    """替换顶层 `block:` 下第一层 `key: value` 的值，保留缩进与行尾注释。"""
    lines = text.splitlines()
    out: list[str] = []
    inside = False
    block_re = re.compile(rf"^{re.escape(block)}:\s*$")
    key_re = re.compile(r"^(\s+)([A-Za-z_][\w]*):(\s*)(.*)$")
    replaced: set[str] = set()
    for line in lines:
        if not inside:
            out.append(line)
            if block_re.match(line):
                inside = True
            continue
        # 退出块：遇到顶格的非空、非注释行
        if line and not line[0].isspace() and not line.lstrip().startswith("#"):
            inside = False
            out.append(line)
            continue
        m = key_re.match(line)
        if m and m.group(2) in values:
            indent, key = m.group(1), m.group(2)
            tail = m.group(4)
            # 保留行尾注释
            comment = ""
            if "#" in tail:
                comment = "  " + tail[tail.index("#"):]
            out.append(f"{indent}{key}: {values[key]}{comment}")
            replaced.add(key)
        else:
            out.append(line)
    missing = set(values) - replaced
    if missing:
        print(f"  ⚠️ 未在 {block}: 块里找到并替换: {', '.join(sorted(missing))}")
    return "\n".join(out) + ("\n" if text.endswith("\n") else "")


def write_intrinsics(fx: float, fy: float, cx: float, cy: float, dist: list[float]) -> None:
    intr = {"fx": round(fx, 2), "fy": round(fy, 2), "cx": round(cx, 2), "cy": round(cy, 2)}
    dist_str = "[" + ", ".join(f"{v:.6f}" for v in dist) + "]"
    if CAMERA_YAML.exists():
        text = CAMERA_YAML.read_text(encoding="utf-8")
        text = _replace_mapping_values(text, "intrinsics", {**intr, "distortion": dist_str})
        CAMERA_YAML.write_text(text, encoding="utf-8")
        print(f"  ✓ 已写入 {CAMERA_YAML.relative_to(WS)}")
    if PERCEPTION_YAML.exists():
        text = PERCEPTION_YAML.read_text(encoding="utf-8")
        # 畸变也要写进 perception —— 检测器去畸变读的是这一份
        text = _replace_mapping_values(text, "camera", {**intr, "distortion": dist_str})
        PERCEPTION_YAML.write_text(text, encoding="utf-8")
        print(f"  ✓ 已写入 {PERCEPTION_YAML.relative_to(WS)}（含 distortion，检测器会用它去畸变）")


# ---------------------------------------------------------------------------
def read_production_resolution() -> tuple[str, int, int]:
    """从 camera.yaml 读生产用的设备与分辨率——标定必须在同一分辨率下做。"""
    dev, w, h = "/dev/video0", 1280, 720
    try:
        d = yaml.safe_load(CAMERA_YAML.read_text(encoding="utf-8")) or {}
        c = d.get("camera", {}) or {}
        dev = str(c.get("device", dev))
        w = int(c.get("width", w))
        h = int(c.get("height", h))
    except Exception:  # noqa: BLE001
        pass
    return dev, w, h


def main() -> int:
    dev0, w0, h0 = read_production_resolution()
    ap = argparse.ArgumentParser(description="相机内参标定")
    ap.add_argument("--print-board", action="store_true", help="生成可 1:1 打印的棋盘格后退出")
    ap.add_argument("--cols", type=int, default=9, help="内角点数（列），默认 9")
    ap.add_argument("--rows", type=int, default=6, help="内角点数（行），默认 6")
    ap.add_argument("--square", type=float, default=20.0, help="方格边长 mm，默认 20（A4 能放下）")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--paper", default="A4", choices=["A4", "A3", "Letter"])
    ap.add_argument("--out", default="", help="--print-board 的输出路径")
    ap.add_argument("--images", default="", help="从文件夹读图（不实时采集）")
    ap.add_argument("--device", default=dev0)
    ap.add_argument("--width", type=int, default=w0)
    ap.add_argument("--height", type=int, default=h0)
    ap.add_argument("--no-write", action="store_true", help="只标定不写回配置")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)

    if args.print_board:
        img = make_board_image(args.cols, args.rows, args.square, args.dpi, args.paper)
        out = args.out or str(WS / "test_artifacts" / "chessboard.png")
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(out, img)
        mm_w = img.shape[1] / (args.dpi / 25.4)
        mm_h = img.shape[0] / (args.dpi / 25.4)
        print(f"✓ 棋盘格已生成: {out}")
        print(f"  {args.cols}x{args.rows} 内角点（{args.cols + 1}x{args.rows + 1} 方格），"
              f"方格 {args.square}mm，{args.paper} @ {args.dpi}dpi（{mm_w:.0f}x{mm_h:.0f}mm）")
        print("  打印时务必选【实际大小 / 100%】，不要'适应页面'缩小，否则尺寸对不上；")
        print("  打完用尺子量一下方格是不是 20mm。然后贴到硬纸板上保证平整。")
        return 0

    if args.images:
        print(f"从 {args.images} 读图…")
        objpoints, imgpoints, shape = load_from_images(args.images, pattern)
    else:
        print(f"实时采集: {args.device} {args.width}x{args.height}（与 camera.yaml 一致）")
        objpoints, imgpoints, shape = capture_live(args.device, args.width, args.height, pattern)

    if len(objpoints) < 6:
        print(f"\n✗ 只有 {len(objpoints)} 张有效图，至少 6 张（建议 15~25 张）")
        return 1
    if shape == (0, 0):
        print("✗ 没拿到图像尺寸")
        return 1

    print(f"\n用 {len(objpoints)} 张图计算内参…")
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    print(f"\n  fx = {fx:.2f}")
    print(f"  fy = {fy:.2f}")
    print(f"  cx = {cx:.2f}   (图像中心 {shape[0] / 2:.1f})")
    print(f"  cy = {cy:.2f}   (图像中心 {shape[1] / 2:.1f})")
    print(f"  畸变系数 D = {[round(float(v), 5) for v in D.ravel()]}")
    print(f"  重投影误差 RMS = {rms:.3f} px")

    if rms < 0.5:
        print("  ✅ 标定质量好（<0.5px）")
    elif rms < 1.0:
        print("  🟡 可接受（0.5~1.0px）。想更好：多拍几张、覆盖四角、板子别反光")
    else:
        print("  ❌ 偏差偏大（>1.0px）。常见原因：板子没贴平 / 打印被缩放 / 图像有模糊 / 角度太单一")

    # 反投影校验：用标定结果看每张图的误差分布，偏差大的那张多半有问题
    worst_i, worst_e = -1, 0.0
    for i, (op, ip) in enumerate(zip(objpoints, imgpoints)):
        proj, _ = cv2.projectPoints(op, rvecs[i], tvecs[i], K, D)
        e = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - ip.reshape(-1, 2), axis=1)))
        if e > worst_e:
            worst_i, worst_e = i, e
    if worst_i >= 0:
        print(f"  最大单张误差 {worst_e:.3f}px（第 {worst_i + 1} 张）")

    # 和"猜的"内参对比，让用户直观看到差多少
    guess = (shape[0] / 2.0) / np.tan(np.radians(60.0) / 2.0)
    print(f"\n  参考: 未标定时按 60° FOV 猜的 fx≈{guess:.1f}，实际 {fx:.1f}"
          f"（差 {abs(fx - guess) / fx * 100:.1f}%）")

    if args.no_write:
        print("\n(--no-write，未写回配置)")
        return 0
    print()
    write_intrinsics(fx, fy, cx, cy, [float(v) for v in D.ravel()])
    print("\n完成。重新 build 后生效:")
    print("  colcon build --symlink-install --packages-select rb_camera rb_perception")
    return 0


if __name__ == "__main__":
    sys.exit(main())
