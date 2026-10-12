# 0.60 m 球心距离与前后相机配置

软件流程：前相机识别 + 雷达测球心 → 停稳 → 相对旋转 180° →
后相机重新识别同一目标 + 雷达测距 → 校正并停稳。
此轮只修改感知、相机采集与上层脚本，未修改底盘驱动、CAN 或底盘配置。

## 测量与到位定义

唯一目标：**两条车尾导轨最外端中点到球心 0.60 m**。
计算为 `norm(球心_base - 导轨最外端中点_base)`，包含高度差，不减球半径。
配置键为 `target_center_distance_m`；旧 `target_gap_m` 被拒绝，避免旧配置静默改变含义。
球半径仍用于拟合球面求球心，不参与最终停车距离减法。
默认验收窗口 ±0.02 m，侧向偏差 ≤0.02 m、朝向偏差 ≤0.035 rad，
实际速度 ≤0.03 m/s、角速度 ≤0.05 rad/s，并持续稳定至少 0.5 s。
容差用于处理噪声，并非第二个目标距离；需要静止实测和移动验收后才能承诺精度。

雷达点位于球面，使用已知半径球面拟合，不能直接使用点云均值作球心。
篮球/排球默认半径 0.12/0.105 m，可按实际用球修改 `radii_m`。
点数、残差、几何条件数或估计不确定性不合格时，无有效距离。
检测框内存在多个球形簇或重复关联时，也不允许控制。
雷达看不到被底盘/机构挡住的球，或者距离太远点数不足时，摄像头仍可画框，
但不能单靠框大小继续盲走。必须在最后 0.60 m 位置验证雷达的视线。

## 通电前可完成的内容

1. 2026-10-12 已由现场确认并配置：前相机为 HD Camera，后相机为 LRCP V1080P，
   分别绑定 `/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0`
   和 `/dev/v4l/by-id/usb-LRCP_V1080P_LRCP_V1080P-video-index0`。
   同一摄像头可能有多个节点，使用能采集 MJPG 的 video-index0；不能仅按 video0/video1 猜。
2. 本轮默认 `calibrated: false`、`extrinsics_confirmed: false`、`intake_confirmed: false`。
   这些标志表示测量尚未完成，不能用猜测数值把它们改成 true。
3. 前相机显示话题保持 `/camera/...`、`/perception/...`；后相机使用
   `/camera_rear/...`、`/perception_rear/...`，无需本轮修改前端。

## 前后相机内参：照着做

先打印棋盘格，按实际大小/100% 打印，量一个格子的边长应为 20 mm。
板子贴平，避免反光。

```bash
python3 tools/calibrate_camera.py --print-board --pdf --out /tmp/chessboard.pdf
```

只暂停正在使用对应摄像头的采集进程，勿重启整套 bringup。
分别采集 20–25 个视角，覆盖画面四角和中央，包含倾斜、远近，锁定对焦，
使用与配置相同的 1280×720 分辨率。采集时只移动板子，不移动车辆。

```bash
python3 tools/calibrate_camera.py --camera-role front --auto
python3 tools/calibrate_camera.py --camera-role rear --auto
# 或已有照片文件夹：
python3 tools/calibrate_camera.py --camera-role rear --images /tmp/rear_calib
```

工具要求至少 15 个有效视角、总 RMS ≤0.7 px、每张 RMS ≤1 px，
检查位置覆盖、倾角和径向畸变单调性。混合分辨率直接拒绝。
通过后同时更新对应相机/感知 YAML 的内参、标定分辨率和 calibrated 标志，
每个文件写前备份并原子替换。不通过时打印原因，保留旧配置。
这只是内参验收，不能替代安装变换和独立距离验收。

## 安装外参和吸球口位置

### 2026-10-12 现场相机高度记录（待完成外参标定）

现场测量：前相机中心在底盘前横梁最底端正上方 0.30 m，
后相机中心在底盘后横梁最底端正上方 0.18 m。
两项测量各自以对应横梁的最底端为高度基准，不是相机离地高度，
也不是相机相对 `base_link` 原点的 z 坐标。
若横梁底面相对 `base_link` 的高度分别为 z_front_beam、z_rear_beam，
则相机中心候选高度分别为 z_front_beam + 0.30 m、z_rear_beam + 0.18 m。
前后横梁底面若不等高，不能直接推断前相机比后相机高 0.12 m。

现场补充确认：前后相机均在横梁左右正中；横梁最底端离地 0.03 m
（按本次前后横梁共同高度的回答记录）。据此，前相机中心离地 0.33 m，
后相机中心离地 0.21 m。这是离地高度，仍需减去 `base_link` 原点离地高度，
才能得到各相机的车体 z 坐标；左右正中是否对应 y=0 也需核对车体原点。

现场已确认前镜头朝车头、向下倾斜 20°；后镜头朝车尾、水平（俯角 0°）。
尚需对应所报“相机中心”与镜头光学中心，以及横梁纵向位置和 `base_link` 原点。
以上高度只作为实测约束，不据此填写完整 `base_from_camera` 或更改标定确认标志。

现场进一步确认：雷达中心在前相机水平后方 0.08 m 处；
此前 0.05 m 是到雷达外壳的距离，不能当作中心间距。
按车头为 +x 的约定，最新安装参照点约束为
`x_lidar_centre = x_front_camera_reference - 0.08 m`。
“水平后方”表述为同高度安装；仍需对应镜头光学中心和厂家定义的传感器原点，
并核对雷达安装轴向。雷达中心不能直接当作 FAST-LIO 点云的 IMU 原点。

2026-10-12 已直接读取 Linux 当前内参：前相机 fx=794.16、fy=793.9、
cx=577.4、cy=378.38，畸变为 [0.018392,-0.525673,0.015934,0.004543,0.808854]；
这组现有值在相机和感知配置中一致，但尚无本轮标定验收结果。
后相机 fx/fy/cx/cy 和畸变均为 0，不能作为有效标定值。
检查标准 ROS 相机标定目录及项目的标定目录未找到可直接复用的内参结果。
读取设备/CameraInfo 或配置的已有值，不等于完成棋盘格内参标定。

整车装配的 `COMPINSTANCETREE` 可读到铲球件和挡板的层级变换。
组合“整车→半边电机驱动→铲球件/挡板”变换后，沿两条导轨伸出方向，
以底盘板包围盒中心为参考，设计导轨外端中点约在 0.6175 m，
挡板包围盒中心约在 0.3390 m，两种基准相差约 0.2785 m。
现场已确认取球机构与图纸一致，测距参考点为两条导轨最外端中点，
目标是从该点到球心 0.60 m；已按下述车体原点约定换算并填写安装候选，尚未独立验收。
不再采用挡板中心或球表面作为停车基准。

所有测量统一以底盘使用的 `base_link` 原点为参考：x 车头、y 左侧、z 上。
相机采用 OpenCV 光学坐标：x 向右、y 向下、z 镜头前方。
FAST-LIO `/cloud_registered_body` 是 **IMU 坐标**，并非自动等于车体坐标。
`base_from_cloud` 必须是 IMU 到 base_link 的完整 4×4 变换。
不要把 Livox 到 IMU 的变换直接当成 IMU 到车体，也不能只填偏航角。

量两条取球导轨最外端中点相对车体原点的位置 `[x,y,z]`，单位 m；车尾 x 应为负。
`intake_outward_xy` 默认 `[-1,0]`，机构有偏角时应填写实测单位方向。

外参工具用同一棋盘格：固定棋盘并独立量出它在车体中的位置/方向，
拍摄一张完整棋盘图。JSON 输入如下；这里的 null 是必须替换的实测量，
不能用这些示例空值直接运行：

```json
{
  "square_m": 0.020,
  "base_from_board": null,
  "base_from_cloud": null,
  "intake_position_base_m": null
}
```

`base_from_board` 的平移列是第一个内角点在 base_link 中的位置，
旋转矩阵第 1/2 列分别为棋盘沿列/行方向的单位向量，第 3 列为两者叉积。
工具输出 `.corners.png`，核对编号 0 是否对应实测起始角点，
列端点和行端点方向是否一致，避免棋盘翻向导致镜像外参。

```bash
python3 tools/calibrate_ball_geometry.py --camera-role front \
  --image /tmp/front_board.jpg --geometry /tmp/front_geometry.json --write
python3 tools/calibrate_ball_geometry.py --camera-role rear \
  --image /tmp/rear_board.jpg --geometry /tmp/rear_geometry.json --write
```

工具用 solvePnP 计算相机到车体变换，检查重投影误差，保存候选与备份；
不会自动把外参或吸球口标记为已验收。前后相机必须共享同一个 IMU→车体变换。
可参考 [OpenCV 的相机标定与坐标变换说明](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)。

## 通电后的静止验收（先不动车）

2026-10-12 已完成 Linux Git 同步、三个上层包构建和 FAST-LIO 编译安装；
保留了外部仓库已有修改，仅新增当前扫描输出设置，没有重编译或启动底盘。
以后部署新主机时核对 `tools/fastlio_latest_input.patch`，仅构建雷达相关组件。
确认 `scan_bodyframe_pub_en: true`，输出 `/cloud_registered_body`，消息 frame_id 为 `body`。
使用当前扫描，不使用累积地图；雷达/相机使用同一 ROS 时钟。
Linux 上 236 项离线测试通过。运动与标定验收仍需现场完成。

仅构建这些 Python 包（消息 ABI 不变），按需重启对应感知/采集节点：

```bash
colcon build --symlink-install --packages-select rb_camera rb_perception rb_mission
source install/setup.bash
ros2 launch rb_perception rear_ball.launch.py
```

后相机启动文件只启动后相机和后感知，不启动底盘、任务或前摄像头。
标定候选已写入时，放一颗球，独立量出它的球心坐标，分别在前/后方，
0.60 m 目标附近及 1 m、2 m、多种侧向位置测量：

```bash
# X Y Z 替换为实测球心 base_link 坐标：
python3 tools/check_ball_ranging.py --camera-role rear \
  --expected-base X Y Z --out /tmp/rear_check.json
```

该工具只有订阅，不发布运动；用生产球心拟合链路检查候选外参，
不会持久修改确认标志。每次需 ≥20 对独立源时间戳数据，95% 球心误差 ≤0.02 m。
少点、遮挡或不通过时输出失败，不用单目距离替代。
实际雷达噪声/点密度达不到 2 cm 时需要调整安装与标定，或者增加近距深度传感器，
不能通过放宽到位阈值声称提高精度。

所有位置和两个相机均通过静止验收，确认吸球口位置后，再手工将相应
`extrinsics_confirmed` 与 `intake_confirmed` 设 true，保存验收报告。

## 控制入口与基础日志

```bash
python3 tools/precision_ball_approach.py         # 只检查配置，不创建运动发布者
# 已验收并具备现场测试条件后才使用 --execute：
python3 tools/precision_ball_approach.py --execute
```

执行入口拒绝其他 `/cmd_vel` 发布者；它不会杀进程、使能底盘或重启底盘。
默认每阶段 45 s 超时、平移 ≤0.08 m/s、旋转 ≤0.18 rad/s。
观测或位姿过期立即发零速；转身后的球须与转身前的目标位置相符，
多个同类候选时停车。最终只有后方新观测持续到位才返回成功。
`tools/test_perimeter_ball.py` 的找球阶段也已接入这套流程，
去掉无距离盲走 0.5 m 和 `stop_distance+0.30` 的宽松到位逻辑。
旧 mission 无有效/新鲜距离时也暂停前进。

感知仅保留启动信息和每 5 s 帧率/耗时/测距原因；
精确脚本记录阶段切换、每 5 s 距离/横向/方向误差以及超时或失败。
`/perception/ball_range` 和 `/perception_rear/ball_range` 为 JSON 状态话题，
记录相机/雷达源时间、球心 base 坐标、半径、点数、残差和不确定性。
已有 DetectionArray 距离仍表示相机光轴深度，不能把它当导轨末端到球心的距离。
运动验收需要现场逐步验证停距、侧向和转角；静止测试及离线测试不能替代该项。

## 2026-10-12 安装标注冲突核查

核查覆盖当前本地工作区、Mac 原始旧工作区及旧控制台目录、相关 Git 历史，
以及 Linux 项目的源码、文档、工具、相机/感知/bringup 安装目录和 FAST-LIO 配置。
Linux 本轮扫描 178 个相关文本文件，源码与安装目录的六份相机/感知 YAML 内容一致。

- 原 `docs/05_定位模块.md` §7.1 仍写前相机朝下 30°、相机高 0.5 m，
  与最新实测 20°、0.33 m 冲突，已修正，并移除基于旧假设 FOV 的视野表。
- 旧版 `camera_rear.yaml`（历史提交 `9be30626` 及 Mac 原始旧工作区）
  已注明后相机居中、水平无俯仰，与本次确认一致；这条已有标注不应视作缺失。
- `camera_yaw_offset_rad: 0.0` 是前相机水平偏航，不表示前相机俯角为 0°，
  不能在此填写 20° 或其弧度值。
- 现有定位配置保留雷达前置 0.30 m 的记录（`sensor_to_base_x: -0.30`）
  及 `sensor_to_base_yaw_deg: -85.0` 的历史实测补偿。源码和安装配置一致。
  前相机后方 0.08 m 是相对中心间距，不能单独替代雷达到车体原点的位置或偏航。
  后续生成完整 IMU→车体变换时，应核对这些旧定位约定，避免符号颠倒或重复补偿。
- `perception_test.yaml` 是冻结的离线自检配置，未配置精确雷达测距；
  生产前/后感知分别读取 `perception.yaml`、`perception_rear.yaml`。
- 整车 launch 中前相机的 `camera_device` 默认参数仍为 `/dev/video0`，会覆盖
  `camera.yaml` 的稳定设备路径；目前该节点确为 HD Camera，没有即时设备冲突，
  后续启动应显式传入前相机已确认的 `/dev/v4l/by-id/...` 路径。

前相机向下 20°、后相机 0° 已作为现场安装约束记录。
即使距离来自雷达，当前实现仍需相机完整外参把雷达点投影到 YOLO 框内；
新俯角不会自动替代内参标定、完整平移或独立测距验收。
本轮只修正标注与配置注释，不生成猜测变换、不更改标定确认标志，也不修改底盘。

## 无棋盘格条件下的静止实地检查

现场无法提供棋盘格时，先完成 CAD 与安装测量的候选配置，再读取真实相机/雷达检查。
本次采用车体原点为四轮中心的地面投影（与原二维车体中心同水平位置），
x 朝车头、y 朝左、z 朝上；CAD 的 Y 为竖直，Z 为取球导轨伸出方向。
CAD 横梁截面中心初步推算水平位置为 ±0.271421 m，雷达位置为 +0.191421 m。
现场随后明确更正：导轨末端在车体中心后方 0.65 m、雷达中心在前方 0.26 m。
这两项实测覆盖 CAD 水平位置；结合雷达中心在前相机后方 0.08 m，
前相机前置位置更新为 +0.34 m。旧定位的雷达前置 0.30 m 与最新实测相差 0.04 m，
旧二维定位参数保留，未将候选坐标自动写入旧定位链路。
后相机纵向位置仍为 CAD 候选，安装高度 0.21 m、朝后且水平由现场确认。

已写入前/后 `base_from_camera`、共同 `base_from_cloud` 和 `intake_position_base_m`：

| 参考点 | x (m) | y (m) | z (m) |
|---|---:|---:|---:|
| 前相机中心 | +0.340000 | 0 | 0.330000 |
| 后相机中心 | -0.271421 | 0 | 0.210000 |
| 雷达几何中心 | +0.260000 | 0 | 0.330000 |
| 导轨最外端中点 | -0.650000 | 约 0 | 0.016500 |

前相机光学 z 轴映射到车体为 `[cos(20°), 0, -sin(20°)]`，后相机为 `[-1,0,0]`。
IMU→车体旋转使用旧定位偏航补偿的逆旋转，不能将旧 -85° 数值直接复制到矩阵；
按 FAST-LIO 当前固定的 LiDAR→IMU 平移扣除内部原点偏移。
仍有参考点近似：相机中心按光学中心处理、雷达几何中心按 LiDAR 原点处理、
相机滚转和 IMU 倾斜暂按水平安装处理。原始来源与假设见 `ball_geometry_candidate.json`。
所有 calibrated/extrinsics_confirmed/intake_confirmed 仍为 false，不冒充已标定。
后相机只验证了可采图，内参仍使用明确的 60° HFOV 估算模型，没有声称测出了内参。

```bash
python3 tools/observe_ball_geometry.py --seconds 15 --out /tmp/ball_geometry_observation.json
```

该工具只订阅真实检测和当前雷达点云，以生产球心拟合算法检查投影和点簇。
无需标定板、独立球心坐标或尺子；输出源数据数量、时效、拟合原因和候选球心距离。
输出始终 `diagnostic_only=true`、`accuracy_verified=false`，不发布任何底盘指令，
不更改确认标志。能获得稳定候选距离也不等于厘米级准确度验收。

现场本次要求无需识别球：实地检查只核对传感器数据链、时间戳和坐标候选，
未摆球且没有独立测距参考，不能声称停车精度或球距实测已经通过。
