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
import math
import os
import re
import sys
import time
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
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))  # tools/ 不是包，按目录导入
from gui_guard import WindowGuard  # noqa: E402


def make_board_pdf(path: str, cols: int, rows: int, square_mm: float, paper: str) -> None:
    """生成**带真实物理尺寸**的棋盘格 PDF（推荐用这个打印）。

    为什么必须提供 PDF：`cv2.imwrite` 写的 PNG **不带 DPI/物理尺寸元数据**，
    打印软件只能自己猜（常见猜 72 / 96 / 150 dpi）。猜错就会出现"选了实际大小
    却打印成放大/裁切"的结果 —— 本项目就踩过：打印出来的棋盘只有 5~6 格宽而不是
    10 格，导致 9x6 的角点检测完全失效。

    PDF 的页面尺寸是真实物理单位（mm），任何打印软件都按它走，100% 打印即准确。
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    sizes = {"A4": (210.0, 297.0), "A3": (297.0, 420.0), "Letter": (215.9, 279.4)}
    pw, ph = sizes.get(paper, sizes["A4"])

    fig = plt.figure(figsize=(pw / 25.4, ph / 25.4))          # 英寸
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, pw)
    ax.set_ylim(0, ph)
    ax.set_aspect("equal")
    ax.axis("off")

    bw, bh = (cols + 1) * square_mm, (rows + 1) * square_mm
    x0, y0 = (pw - bw) / 2.0, (ph - bh) / 2.0
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:      # 黑格
                ax.add_patch(Rectangle((x0 + c * square_mm, y0 + r * square_mm),
                                       square_mm, square_mm,
                                       facecolor="black", edgecolor="none"))
    fig.savefig(path, format="pdf")
    plt.close(fig)


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


def sharpness(gray: np.ndarray) -> float:
    """拉普拉斯方差：越大越清晰。实测 <40 基本是失焦/离太近，>120 算清晰。"""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def focus_hint(s: float) -> tuple[str, tuple[int, int, int]]:
    if s < 40:
        return "严重模糊：离太近/失焦", (0, 0, 255)
    if s < 120:
        return "偏软", (0, 165, 255)
    return "清晰", (0, 255, 0)


def detect_preview(gray: np.ndarray, pattern: tuple[int, int],
                   scale: float = 0.5) -> np.ndarray | None:
    """预览用的**快速**棋盘检测：降采样后再找，坐标放大回原分辨率。

    为什么必须这样：全分辨率 1280x720 跑一次 find_corners 要 **~155ms**（实测），
    预览若每帧都做，循环被拖到 6fps —— 表现就是"画面很卡"。
    降采样一半只要 ~50ms，配合调用方限频，预览就顺了。

    ⚠️ 这里精度有限，**只用于状态显示与姿态判断**；
       真正要存下来做标定的那一次必须用全分辨率重跑（见调用处）。
    """
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    c = find_corners(small, pattern)
    if c is None:
        return None
    return c / scale


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
    guard = WindowGuard(win)
    print("\n把棋盘格放进取景框：远/近、四角、左右倾斜各拍几张，共 15~25 张。")
    print("空格=拍一张   c=计算   u=撤销上一张   q=退出\n")
    detect_interval = 0.25      # 预览检测限频（全分辨率单次 ~155ms，不限频就会卡）
    last_detect = 0.0
    c_preview = None
    fps_t, fps_n, fps_val = time.time(), 0, 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            shape = frame.shape[1], frame.shape[0]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()
            now = time.time()
            if now - last_detect >= detect_interval:
                c_preview = detect_preview(gray, pattern)   # 降采样快速检测
                last_detect = now
            c = c_preview
            if c is not None:
                cv2.drawChessboardCorners(vis, pattern, c, True)
                cv2.putText(vis, "FOUND - press SPACE", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(vis, "searching board...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2, cv2.LINE_AA)
            cv2.putText(vis, f"captured: {len(objpoints)}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            fps_n += 1
            if now - fps_t >= 0.5:
                fps_val = fps_n / (now - fps_t)
                fps_t, fps_n = now, 0
            sh = sharpness(gray)
            ftxt, fcol = focus_hint(sh)
            cv2.putText(vis, f"focus {sh:5.0f}  {ftxt}   preview {fps_val:4.1f} fps",
                        (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, f"focus {sh:5.0f}  {ftxt}   preview {fps_val:4.1f} fps",
                        (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, fcol, 1, cv2.LINE_AA)
            cv2.imshow(win, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            if key == ord("u") and objpoints:
                objpoints.pop()
                imgpoints.pop()
                print(f"  撤销一张，剩 {len(objpoints)}")
            if key == ord(" ") and c is not None:
                full = find_corners(gray, pattern)   # 存档前用全分辨率重跑，保证精度
                if full is not None:
                    objpoints.append(objp)
                    imgpoints.append(full)
                    print(f"  已拍 {len(objpoints)} 张（focus={sh:.0f}）")
            if key == ord("c"):
                if len(objpoints) >= 6:
                    break
                print(f"  还太少（{len(objpoints)} 张），至少 6 张，建议 15 张以上")
            if guard.closed():
                print("  检测到窗口被关闭，退出")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return objpoints, imgpoints, shape


# -- 自动采集（推荐）--------------------------------------------------------
def _pose_signature(corners: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """把一次棋盘检出压成 4 维特征：[中心x, 中心y, 尺度, 旋转角]（前两个已归一化）。"""
    h, w = shape[:2]
    pts = corners.reshape(-1, 2)
    cx, cy = float(pts[:, 0].mean()) / w, float(pts[:, 1].mean()) / h
    x0, y0 = pts.min(axis=0)
    x1, y1 = pts.max(axis=0)
    scale = math.sqrt(max(float(x1 - x0) * float(y1 - y0), 1.0)) / min(w, h)
    v = pts[len(pts) // 2] - pts[0]
    ang = math.atan2(float(v[1]), float(v[0]))
    return np.array([cx, cy, scale, ang])


def _too_similar(sig: np.ndarray, existing: list[np.ndarray], thresh: float) -> bool:
    """与已拍的某一张太像就算重复（位置权重最高，其次是尺度和倾角）。"""
    for s in existing:
        d = math.hypot(sig[0] - s[0], sig[1] - s[1]) * 2.5
        d += abs(sig[2] - s[2]) * 3.0
        d += abs(math.atan2(math.sin(sig[3] - s[3]), math.cos(sig[3] - s[3]))) / math.pi
        if d < thresh:
            return True
    return False


def probe_pattern(device: str, width: int, height: int,
                  max_cols: int = 12, max_rows: int = 9) -> tuple[int, int] | None:
    """对着棋盘格**自动探测它实际有多少内角点**。

    用途：打印被缩放/裁切、或拿到一块不知道规格的板子时，不用猜 `--cols/--rows`。
    把板子放在相机前，它会从大到小试各种格数，屏幕上显示当前能检出的最大图案。
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"✗ 打不开相机 {device}（rb_camera 还在跑？先停掉它）")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    win = "probe chessboard  (q=quit)"
    guard = WindowGuard(win)
    print("\n把棋盘格对准相机（尽量正对、占画面一半以上）。")
    print("会从大到小试各种格数，屏幕左上角显示当前检出的最大图案。q 退出。\n")
    last = time.time()
    best: tuple[int, int] | None = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 穷举十几种格数，若都用全分辨率(155ms/次)会卡住 1.5 秒以上 →
            # 这里只在降采样图上探测（识别够用），确定后再用全分辨率画一次
            small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            now = time.time()
            if now - last > 0.5:
                last = now
                for cols in range(max_cols, 2, -1):
                    hit = None
                    for rows in range(min(cols, max_rows), 2, -1):
                        c = find_corners(small, (cols, rows))
                        if c is not None:
                            hit = (cols, rows, c)
                            break
                    if hit:
                        best = (hit[0], hit[1])
                        break
            vis = frame.copy()
            if best:
                c = find_corners(gray, best)
                if c is not None:
                    cv2.drawChessboardCorners(vis, best, c, True)
                txt = (f"FOUND {best[0]}x{best[1]} inner corners "
                       f"= {best[0]+1}x{best[1]+1} squares  -> use --cols {best[0]} --rows {best[1]}")
                col = (0, 255, 0)
            else:
                txt = "no chessboard found (larger? better lighting? flatter?)"
                col = (0, 165, 255)
            cv2.putText(vis, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1, cv2.LINE_AA)
            cv2.imshow(win, vis)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
            if guard.closed():
                print("  检测到窗口被关闭，退出")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if best:
        print(f"\n  ✅ 检出最大图案：{best[0]}x{best[1]} 内角点 = "
              f"{best[0]+1}x{best[1]+1} 方格")
        print(f"     标定请用：--cols {best[0]} --rows {best[1]}")
    else:
        print("\n  ❌ 没检出任何棋盘图案。检查：板子是否平、光照是否够、是否整块入镜")
    return best


def capture_auto(device: str, width: int, height: int, pattern: tuple[int, int],
                 target_count: int = 20, min_interval: float = 0.30,
                 diff_thresh: float = 0.10):
    """自动采集：你只管举着棋盘格**变换角度**，它自己挑"够不一样"的帧拍下来。

    为什么需要：标定精度取决于**姿态多样性**，尤其是倾斜——平面标定在正对相机时
    存在"焦距-尺度退化"（板子放大一倍、距离远一倍，图像一样），必须靠倾斜打破。
    手动按空格很容易连拍十几张几乎一样的正对姿态，标出来 RMS 看着还行但焦距是错的。
    这里用 4 维姿态特征做去重，自动保证多样性。

    操作：举着板子慢慢靠近/远离/左右移/上下俯仰/左右倾斜，拍够 target_count 张自动结束。
          q/Ctrl-C 随时退出，c 用当前已拍的立刻计算。
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"✗ 打不开相机 {device}（rb_camera 还在跑？先停掉它）")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)

    objpoints: list = []
    imgpoints: list = []
    sigs: list[np.ndarray] = []
    shape = (0, 0)
    last_t = 0.0
    win = "calibrate_camera AUTO   q=quit  c=compute now"

    guard = WindowGuard(win)
    print(f"\n自动采集：目标 {target_count} 张。请举着棋盘格：")
    print("  ① 慢慢靠近 / 远离   ② 移到画面四角   ③ 上下俯仰、左右倾斜（最重要）")
    print("  拍够会自动结束；q 退出，c 用已拍的立刻计算\n")

    detect_interval = 0.25      # 预览检测限频（全分辨率单次 ~155ms，不限频就卡）
    last_detect = 0.0
    c_preview = None
    fps_t, fps_n, fps_val = time.time(), 0, 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            shape = frame.shape[1], frame.shape[0]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()
            now = time.time()
            # 预览检测：降采样 + 限频，保证画面流畅（否则被 155ms/次拖到 6fps）
            if now - last_detect >= detect_interval:
                c_preview = detect_preview(gray, pattern)
                last_detect = now
            c = c_preview
            status = "searching board..."
            color = (0, 165, 255)

            if c is not None:
                cv2.drawChessboardCorners(vis, pattern, c, True)
                if now - last_t >= min_interval:
                    sig = _pose_signature(c, frame.shape)
                    if _too_similar(sig, sigs, diff_thresh):
                        status = "too similar - change angle/distance"
                    else:
                        # 真正要存下来 → 用全分辨率重跑一次，保证角点精度
                        full = find_corners(gray, pattern)
                        if full is not None:
                            objpoints.append(objp)
                            imgpoints.append(full)
                            sigs.append(_pose_signature(full, frame.shape))
                            last_t = now
                            status = f"CAPTURED #{len(objpoints)}"
                            color = (0, 255, 0)
                            print(f"  已拍 {len(objpoints):2d}/{target_count}  "
                                  f"中心({sig[0]:.2f},{sig[1]:.2f}) 尺度{sig[2]:.2f} "
                                  f"倾角{sig[3]:+.2f}  focus={sharpness(gray):.0f}")
                else:
                    status = "ok (waiting interval)"
                    color = (0, 255, 0)

            cv2.putText(vis, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
            cv2.putText(vis, f"captured {len(objpoints)}/{target_count}", (10, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
            fps_n += 1
            if now - fps_t >= 0.5:
                fps_val = fps_n / (now - fps_t)
                fps_t, fps_n = now, 0
            sh = sharpness(gray)
            ftxt, fcol = focus_hint(sh)
            cv2.putText(vis, f"focus {sh:5.0f}  {ftxt}   preview {fps_val:4.1f} fps",
                        (10, 101), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, f"focus {sh:5.0f}  {ftxt}   preview {fps_val:4.1f} fps",
                        (10, 101), cv2.FONT_HERSHEY_SIMPLEX, 0.6, fcol, 1, cv2.LINE_AA)
            # 进度条
            frac = min(1.0, len(objpoints) / max(target_count, 1))
            cv2.rectangle(vis, (10, 74), (10 + int(220 * frac), 86), (0, 255, 0), -1)
            cv2.rectangle(vis, (10, 74), (230, 86), (255, 255, 255), 1)
            cv2.imshow(win, vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c") and objpoints:
                break
            if len(objpoints) >= target_count:
                print(f"\n  已拍满 {target_count} 张，开始计算")
                break
            if guard.closed():
                print("  检测到窗口被关闭，退出")
                break
    except KeyboardInterrupt:
        print("\n  中断，用已拍的继续计算")
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
    ap.add_argument("--pdf", action="store_true",
                    help="生成 PDF（推荐！带真实物理尺寸，打印不会被缩放/裁切）")
    ap.add_argument("--cols", type=int, default=9, help="内角点数（列），默认 9")
    ap.add_argument("--rows", type=int, default=6, help="内角点数（行），默认 6")
    ap.add_argument("--square", type=float, default=20.0, help="方格边长 mm，默认 20（A4 能放下）")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--paper", default="A4", choices=["A4", "A3", "Letter"])
    ap.add_argument("--out", default="", help="--print-board 的输出路径")
    ap.add_argument("--images", default="", help="从文件夹读图（不实时采集）")
    ap.add_argument("--auto", action="store_true",
                    help="自动采集（推荐）：举着棋盘格变换角度，自动挑够不一样的帧")
    ap.add_argument("--auto-count", type=int, default=20, help="自动采集目标张数，默认 20")
    ap.add_argument("--diff-thresh", type=float, default=0.10,
                    help="自动采集的去重阈值，越小越严格（默认 0.10）")
    ap.add_argument("--device", default=dev0)
    ap.add_argument("--width", type=int, default=w0)
    ap.add_argument("--height", type=int, default=h0)
    ap.add_argument("--probe", action="store_true",
                    help="自动探测棋盘格实际有多少内角点（不知道板子规格时用）")
    ap.add_argument("--no-write", action="store_true", help="只标定不写回配置")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)

    if args.print_board:
        n_col = args.cols + 1
        n_row = args.rows + 1
        out = args.out or str(WS / "test_artifacts" /
                              ("chessboard.pdf" if args.pdf else "chessboard.png"))
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        if args.pdf or out.lower().endswith(".pdf"):
            make_board_pdf(out, args.cols, args.rows, args.square, args.paper)
            kind = "PDF（带真实物理尺寸）"
        else:
            img = make_board_image(args.cols, args.rows, args.square, args.dpi, args.paper)
            cv2.imwrite(out, img)
            kind = f"PNG @ {args.dpi}dpi（⚠️ 无 DPI 元数据，打印软件可能猜错）"
        print(f"✓ 棋盘格已生成: {out}")
        print(f"  {args.cols}x{args.rows} 内角点（{n_col}x{n_row} 方格），"
              f"方格 {args.square}mm，{args.paper}，{kind}")
        print()
        print(f"  ⚠️ 打印后必做两项核对（这是唯一可靠的验收方式，别跳过）：")
        print(f"     ① 用尺子量一个方格，必须是 {args.square:g}mm（不是就说明被缩放了）")
        print(f"     ② 数一下横向有 {n_col} 个方格（不是就说明被裁切了）")
        print(f"  满足这两条，标定时就用默认的 --cols {args.cols} --rows {args.rows}")
        print(f"  另外：贴到硬纸板/亚克力板上保证平整，皱了会显著拉高 RMS。")
        return 0

    if args.probe:
        probe_pattern(args.device, args.width, args.height)
        return 0

    if args.images:
        print(f"从 {args.images} 读图…")
        objpoints, imgpoints, shape = load_from_images(args.images, pattern)
    elif args.auto:
        print(f"自动采集: {args.device} {args.width}x{args.height}（与 camera.yaml 一致）")
        objpoints, imgpoints, shape = capture_auto(
            args.device, args.width, args.height, pattern,
            target_count=args.auto_count, diff_thresh=args.diff_thresh)
    else:
        print(f"手动采集: {args.device} {args.width}x{args.height}（与 camera.yaml 一致）")
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
