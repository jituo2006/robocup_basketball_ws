# 07 · 相机接入 与 篮球/干扰球 YOLO 训练

回答两个现场问题：**"是不是把工业相机插上主机就行"** 和 **"要不要训练一个 YOLO 识别篮球和干扰球，怎么做"**。

---

## 一、相机已接入（本次完成）

### 1.1 实际硬件（本机实测）

```
USB 2.0 Camera: HD USB Camera      ID 05a3:9230 (ARC International)
  /dev/video0                       ← 采集节点（用这个）
  /dev/video1                       ← metadata 节点（别用）

支持格式：
  MJPG  1280x720@60  1920x1080@30  640x480@120 ...
  YUYV  1280x720@9   1920x1080@6   640x480@30   ← 高分辨率下极慢
```

**这是 UVC 免驱 USB 摄像头，不是海康工业相机**，所以不用装 `/opt/MVS` SDK。
（队里以前那台海康工业相机才需要 MVS SDK + GigE 配网，见文末附录。）

### 1.2 新增的驱动包 `rb_camera`

不依赖 `usb_cam` / `v4l2_camera`（本机都没装），自己写在工作空间里：

```
src/rb_camera/
├── rb_camera/camera_node.py     # 驱动节点
├── config/camera.yaml           # 设备/分辨率/格式/QoS/压缩/内参
└── launch/camera.launch.py
```

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | 原始图（bgr8） |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | **JPEG 压缩图（实际用这个）** |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 内参（RViz/调试用） |
| `/rb_camera/ok` | `std_msgs/Bool` | 相机是否在出图 |

驱动做了三件 UVC 相机必须处理的事：

1. **MJPG 协商**：`FOURCC` 必须在设置分辨率**之前**下发，否则 V4L2 按 YUYV 协商，
   720p 直接掉到 9fps。
2. **独立采集线程**：持续抓帧、只保留最新一帧；发布定时器只取最新帧。
   这样处理慢时也不会积压缓冲、画面不会越来越延迟。
3. **掉线自愈**：连续读失败 N 次就 release 重开；拔了又插能自己恢复。

### 1.3 启动

```bash
source install/setup.bash

# 单独起相机
ros2 launch rb_camera camera.launch.py
ros2 launch rb_camera camera.launch.py width:=640 height:=480    # 换分辨率

# 整车（已把相机接进 bringup，默认走压缩）
ros2 launch rb_bringup bringup_full.launch.py
ros2 launch rb_bringup bringup_lidar_vision.launch.py
```

### 1.4 看画面（⚠️ 本机没装任何 ROS 看图工具）

**驱动只发 ROS 话题，不会自己弹窗**。而本机 `rqt_image_view` / `rqt` / `image_view`
**全都没装**，所以 `ros2 launch rb_camera camera.launch.py` 之后图是发出去了，
但没有任何东西显示它——这就是"看不到拍的图像"的原因。

自带的看图工具（只用已装的 OpenCV，无需新装包）：

```bash
python3 tools/view_camera.py                  # 实时画面 + 检测框（推荐）
python3 tools/view_camera.py --raw            # 看原始图（720p 下帧率低，属正常）
python3 tools/view_camera.py --no-detections  # 只看图
python3 tools/view_camera.py --duration 6 --save /tmp/shot.jpg   # 无人值守存图
```

快捷键：`q`/ESC 退出、`s` 存图、`d` 切换检测框。

其他看图的办法：

```bash
# rviz2 已装，可用 Image display 显示 /camera/image_raw/compressed
rviz2
# 想在别的电脑上看：装 rqt_image_view
sudo apt install ros-humble-rqt-image-view
rqt_image_view /camera/image_raw/compressed
```

> 用 `rqt_image_view` 看 **原始** `/camera/image_raw` 时若发现很卡，
> 那是因为 720p 原始图投递只有几 fps（见第二节），不是相机坏。

### 1.5 自检

```bash
python3 tools/camera_check.py                # 出图率 + 内参 + 存快照
python3 tools/vision_rate_check.py --sweep   # 相机 vs 感知吞吐，用来定分辨率
```

---

## 二、⚠️ 关键发现：大图像消息吞吐瓶颈 → 必须用 JPEG 压缩

这是本次调试最重要的发现，**不解决的话机器人视觉只有 5~7fps**。

### 2.1 现象

原始图（未压缩）投递给订阅端，帧率随分辨率崩塌：

| 分辨率 | 每帧大小 | 相机发布 | rb_perception 实收 |
| --- | --- | --- | --- |
| 320x240 | 230 KB | 30 fps | **30 fps** ✅ |
| 640x480 | 921 KB | 30 fps | **5~11 fps** ❌ |
| 1280x720 | 2.7 MB | 30 fps | **4~7 fps** ❌ |

### 2.2 定位过程（排除法）

1. **不是算法慢**：进程内直接跑检测器，480p 仅 11.2 ms/帧（≈89fps 上限），
   720p 30.3 ms/帧（≈33fps 上限）。
2. **不是 CPU 满**：相机进程占 0.17 核、感知占 0.25 核，都在等待。
3. **不是 QoS 方向反了**：把发布端从 reliable 改成 best_effort 只从 3.2 → 8.4 fps，仍不够。
4. **不是网卡选错**：`ROS_LOCALHOST_ONLY=1` 强制走本机，依旧 ~7fps。
5. **是"字节数"卡住**：320x240(230KB)→30fps、640x480(921KB)→7fps，
   两者**带宽都是约 7 MB/s**。即瓶颈是"大图像消息的投递吞吐 ≈ 7MB/s"，
   与分辨率无关，只看每帧字节数。

> 这个 7MB/s 对本工程还有别的含义：雷达点云也是大消息，
> 若发现 FAST-LIO 的点云上不去，可优先怀疑同一类吞吐问题。

### 2.3 解决：JPEG 压缩（已默认开启）

相机节点多发一路 `sensor_msgs/CompressedImage`（JPEG q=80），
把 640x480 从 921KB 压到 ~40KB、720p 从 2.7MB 压到 ~55KB，30fps 只需 1.2~1.7MB/s。

**效果（本机实测）：**

| 配置 | 感知输出 |
| --- | --- |
| 640x480 原始 | 5~11 fps |
| 640x480 + JPEG | **30.6 fps** |
| 1280x720 原始 | 4~7 fps |
| 1280x720 + JPEG | **25.6 fps** |

配置位置：`src/rb_camera/config/camera.yaml`

```yaml
publish_compressed: true    # 默认开，强烈建议保持
jpeg_quality: 80
```

对应地，`rb_perception` 默认订阅压缩话题：

```bash
ros2 run rb_perception perception_node --ros-args -p use_compressed:=true
# 关掉压缩（回到原始图，只在调试时用）：
ros2 run rb_perception perception_node --ros-args -p use_compressed:=false
```

> 想看原始图调试（rqt_image_view / RViz）时订阅 `/camera/image_raw` 即可，
> 但那一路帧率会低——那是"没压缩"，不是相机坏。

---

## 三、标定：相机内参 + 颜色阈值

### 3.1 相机内参（单目测距的前提）

`Detection.distance_m` 用 `d = fx × 真实直径 / 像素宽` 算，所以 `fx/fy/cx/cy` 必须标定。
未标定时按 `assumed_hfov=60°` 猜一个（本机 720p 下 fx≈1108），能跑但测距不准。
**它影响所有依赖距离的功能：抓球距离判定、避障排斥半径、YOLO 距离估计。**

⚠️ 必须在**生产分辨率**下标定（工具默认读 `camera.yaml` 的 1280x720）。换分辨率要重标。

```bash
# ① 生成可 1:1 打印的棋盘格（9x6 内角点、方格 20mm、A4）
python3 tools/calibrate_camera.py --print-board --out /tmp/chessboard.png
#    打印务必选【实际大小/100%】，别"适应页面"；打完量一下方格是不是 20mm

# ② 采集 + 标定（先停掉 rb_camera，直接用设备）
python3 tools/calibrate_camera.py
#    窗口里：空格=拍一张（只在检出棋盘时有效） c=计算 u=撤销 q=退出
#    目标 15~25 张：远/近、四角、左右倾斜都要有

# ③ 或用已有图片
python3 tools/calibrate_camera.py --images ~/calib_shots
```

工具会打印 `fx/fy/cx/cy`、畸变系数、**重投影误差 RMS**，并给质量判定：

| RMS | 含义 |
| --- | --- |
| < 0.5 px | 好 |
| 0.5~1.0 px | 可接受 |
| > 1.0 px | 偏差大 → 板子没贴平 / 打印被缩放 / 图模糊 / 角度太单一 |

结果**自动同时写入**两个配置（测距读的是 perception 那份），且**保留原注释**：

- `src/rb_camera/config/camera.yaml` → `intrinsics`
- `src/rb_perception/config/perception.yaml` → `camera`

写完重新 build 生效：

```bash
colcon build --symlink-install --packages-select rb_camera rb_perception
```

> 该工具的正确性已用**合成标定**自测过：造 18 张已知内参（fx=fy=1000, cx=640, cy=360）
> 的虚拟棋盘视图，反推得到 fx=999.9，误差 0.01%。顺带量化了"不标定的代价"：
> 60° FOV 猜出的 fx≈1109 会让距离偏约 11%。

**畸变会真正参与计算。** 检测器在算方位角/距离前会先用 `cv2.undistortPoints`
把像素坐标去畸变。标定脚本会把 `distortion` 同时写进 `camera.yaml` 和
`perception.yaml`——后者才是检测器读的那份。

为什么值得做（合成验证：fx=fy=1000、k1=−0.25）：

| 离光轴 | 不去畸变的方位角误差 | 去畸变后 |
| --- | --- | --- |
| 0° | 0.00 mrad | 0.00 mrad |
| 15° | 4.99 mrad | 0.00 mrad |
| 25° | 22.9 mrad | 0.00 mrad |
| 35° | **60.0 mrad（3.44°）** | **0.04 mrad（0.002°）** |

3.44° 在 3m 处是 **18cm** 横向偏差。抓球有闭环能修，但**瞄准投/传是按方位角
开环对准的，系统偏差不会自己消失**。

### 3.2 颜色（HSV）阈值标定

`perception.yaml` 里的 `hsv_ranges` 是通用占位值（"典型橙色篮球"）。
2026 规则**各队自带用球**，颜色图案不统一，必须按你们的真球重新采。

```bash
# 实时采样（推荐）：空格冻结画面 → 拖框框住球 → 回车；可采多次；c=计算并写回
python3 tools/calibrate_color.py --label ball_basketball

# 从图片 + 指定框
python3 tools/calibrate_color.py --image shot.jpg --rect 600,300,140,140 \
    --label ball_volleyball

# 只看结果不写回
python3 tools/calibrate_color.py --label ball_basketball --dry-run
```

工具做三件关键的事：

1. **按内接椭圆采样**，而不是整个矩形 ROI。球是圆的、矩形四角是背景——
   实测踩到过：框住橙色篮球后，四角的深灰背景被算成第二个色相簇，推导出
   `[174, 0, 30, 6, ...]` 这种 `S≥0` 的区间，**几乎匹配一切，是误检制造机**。
2. **色相环上聚类并丢弃背景杂簇**（质量 < 主簇 25% 的丢掉），因此对
   "排球那种蓝/黄多色块"会输出多段区间。
3. **用工程里真正的 `BlobDetector` 跑一遍验证**，打印检出数/置信度/方位/距离，
   并生成对照图 `test_artifacts/color_calib_<label>.png`（原图 | 掩码 | 叠加），
   你可以直接看有没有把地板也框进来（掩码占全图 > 8% 通常就是框太松）。

采样建议：3~5 次，覆盖亮面/暗面/不同距离。写回前会自动备份 `perception.yaml.bak`。

> 多段区间说明球上有多块颜色——HSV 方案对这种球天然吃力，这正是后面要上 YOLO 的原因。

### 3.3 ⚠️ 测试配置与生产配置是隔离的

`tools/make_test_image.py` 生成的合成图用的是**硬编码标准色**，而 3.2 会把生产
`perception.yaml` 改成**你们真球**的颜色。如果不隔离，标定完 `verify_offline.py`
的视觉项就会**误报失败**（因为真球可能根本不是橙色）。

所以离线自检固定读冻结的 **`src/rb_perception/config/perception_test.yaml`**：

```
perception.yaml       ← 生产：按真球标定，给车上的 rb_perception 用
perception_test.yaml  ← 冻结标准色：只给 verify_offline 的视觉自检用
```

**别改 `perception_test.yaml`**；它只回答"视觉管道通不通"。
真球检测效果用 `tools/view_camera.py` 看实况。

### 3.4 锁定自动曝光 / 白平衡（让颜色标定长期有效）

HSV 阈值对**光照**敏感，而相机的**自动曝光/自动白平衡一直在动**——同一颗球
在不同时刻会落到不同的 S/V，上午标好的阈值下午就不好使。所以要把它们锁死。

```bash
python3 tools/tune_camera.py     # 先停掉 rb_camera（同一设备不能被两个进程用）
#   拖滑块实时生效  s=写回 camera.yaml  a=自动/手动切换对比  r=复位  q=退出
```

驱动每次打开相机都会自动应用 `camera.yaml` 的 `controls:` 段，并**读回确认**
（这条很关键——必须确认"锁"真的生效，而不是以为生效）：

```
相机固定项已应用: power_line_frequency=1, 自动曝光→手动, exposure_time_absolute=30,
                自动白平衡→关, white_balance_temperature=4600
读回确认: 曝光已锁, 白平衡已锁, 曝光值=30, 色温=4600
```

调参目标：**画面均值亮度 80~160**、**R/G/B 三个均值尽量接近**。
本机屋里实测默认值（曝光 30、色温 4600）→ 亮度 127、R/G/B 133/131/118，合适。

⚠️ **这台相机的一个坑**：自动模式下 `exposure_time_absolute` 和
`white_balance_temperature` 都带 `flags=inactive`，**读不到相机自动算出的值**。
所以曝光/色温只能人工定（`tune_camera.py` 就是为此而生），不能"读出来再锁"。

> 换场地/换灯光后要重跑 `tune_camera.py` + `calibrate_color.py`。
> 想彻底摆脱这件事就上 YOLO（第四节）。

---

## 四、YOLO：识别篮球 vs 干扰球（排球）

### 4.1 为什么必须用 YOLO，而不是颜色阈值

- 2026 规则：**各队自带用球**，球的颜色/图案不统一，颜色阈值换队就失效。
- 比赛在真实光照、动态场景下，深度学习比 HSV 阈值稳一个量级。

> 规则里传球/投篮两环节的"目标球 vs 干扰球"**角色互换**。
> 本工程 `mission_labels`（`mission.yaml`）已做成配置，YOLO 只负责输出
> `ball_basketball` / `ball_volleyball` 两个类别，谁当目标谁当干扰由状态机定。

### 4.2 训练环境

- **建议**：在带 NVIDIA GPU 的机器（或云 GPU）上训，`yolov8n/yolo11n` 很小，
  普通显卡几小时就够。
- 本机（NUC）没装 torch/ultralytics，只负责**推理**（onnxruntime，CPU 即可跑 n 模型）。

```bash
pip install ultralytics        # 训练 + 导出
pip install onnxruntime        # 车上推理（本机还没装，要装）
```

### 4.3 数据采集与标注（最关键的一步）

1. **采集**：在**真实场地、真实光照**下拍自己队的篮球 + 排球 + 对方球
   （赛前拿到对方球信息后补拍），覆盖不同距离、角度、遮挡、地板上滚动。
   - 每种球 300~500 张；可用 `26_RC/datasets/` 和 `rc26_r2_algorithm_upload/`
     里的旧数据做预训练权重。
2. **标注**：labelImg（本地）或 Roboflow（网页，队里用过）。类别两个：
   `basketball`、`volleyball`。
3. **转 YOLO 格式**：

```yaml
# data.yaml
path: /home/user/datasets/basketball
train: images/train
val: images/val
names:
  0: basketball
  1: volleyball
```

### 4.4 训练与导出

```bash
yolo detect train model=yolov8n.pt data=data.yaml epochs=120 imgsz=640 batch=16 device=0
yolo export model=runs/detect/train/weights/best.pt format=onnx imgsz=640
# 得到 best.onnx，拷到机器人，如 ~/models/ball.onnx
```

### 4.5 接进本工程

`src/rb_perception/rb_perception/detectors.py` 里已留 `OnnxDetector` 接口，
但 `detect()` 目前是 `raise NotImplementedError`——**模型导出后我来补**
（YOLOv8/v11 ONNX 输出 `[1, 4+C, 8400]`，需 letterbox 预处理 + NMS 后处理 +
用 `CameraModel` 反算 `bearing_rad`/`distance_m`）。补完后在 `perception.yaml` 打开：

```yaml
    - label: ball_basketball
      type: onnx
      enabled: true
      model_path: "/home/user/models/ball.onnx"
```

> YOLO 还有个额外好处：它比 HSV+霍夫圆快得多，且不会像 circle 检测那样在
> 复杂背景下误检，是本工程"感知跟上 30fps"的长期解。

---

## 五、本工程已铺好的路

1. **视觉势场避障**：`rb_mission` 订阅 `/perception/detections`，把当前环节的
   **干扰球** / `obstacle` 换算成排斥速度叠加到前进速度上。
   参数在 `mission.yaml` 的 `avoidance:` 段。
2. **go_to_pose 服务 + 点地图**：`/rb_mission/goto_pose`（`GotoPose.srv`）；
   `tools/click_goto.py` 在电脑上点地图即发指令。

```bash
python3 tools/click_goto.py                       # 点地图操控
ros2 service call /rb_mission/goto_pose rb_msgs/srv/GotoPose \
    "{x: 5.0, y: 3.0, yaw: 0.0, align_yaw: false}"
```

---

## 附录：如果换回海康工业相机（GigE/USB3）

```
① 装海康 MVS SDK（官网 Linux 版，装到 /opt/MVS）
   确认：ls /opt/MVS/Samples/64/Python/MvImport
② GigE 网口相机：网卡配静态 IP + 巨帧
   sudo ip addr add 192.168.1.100/24 dev <网卡>
   sudo ip link set <网卡> mtu 9000
③ 参考 26_RC/src/yolo_test/hk_camera.py（MvCameraControl_class）发 sensor_msgs/Image
④ 其余流程同本文（MJPG/FOURCC 换成 SDK 的 PixelType，压缩逻辑照旧可用）
```

工业相机通常支持硬件 JPEG 或 Bayer 输出，带宽压力比 UVC 小，但**压缩/小图**的原则不变。
