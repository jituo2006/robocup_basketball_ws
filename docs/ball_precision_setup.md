# 0.60 m 球面距离与前后相机配置

软件流程：前相机识别 + 雷达测球心 → 停稳 → 相对旋转 180° →
后相机重新识别同一目标 + 雷达测距 → 校正并停稳。
此轮只修改感知、相机采集与上层脚本，未修改底盘驱动、CAN 或底盘配置。

## 测量与到位定义

唯一目标：**车尾吸球口中心到球表面 0.60 m**。
计算为 `norm(球心_base - 吸球口中心_base) - 球半径`，包含高度差。
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

所有测量统一以底盘使用的 `base_link` 原点为参考：x 车头、y 左侧、z 上。
相机采用 OpenCV 光学坐标：x 向右、y 向下、z 镜头前方。
FAST-LIO `/cloud_registered_body` 是 **IMU 坐标**，并非自动等于车体坐标。
`base_from_cloud` 必须是 IMU 到 base_link 的完整 4×4 变换。
不要把 Livox 到 IMU 的变换直接当成 IMU 到车体，也不能只填偏航角。

量吸球口中心相对车体原点的位置 `[x,y,z]`，单位 m；车尾 x 应为负。
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
已有 DetectionArray 距离仍表示相机光轴深度，不能把它当吸球口表面距离。
运动验收需要现场逐步验证停距、侧向和转角；静止测试及离线测试不能替代该项。
