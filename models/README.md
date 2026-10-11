# 当前视觉模型

`ball_20261011_v5.onnx` 来自用户提供的 `best(1).onnx`，保持原文件内容。

- SHA-256：`4dea3c15d3830d05cc763ce49a07a6436f7c69ab7634ae06023633aad9d77c77`
- 导出元数据：YOLO26n，dataset_ball_v5，2026-10-11 21:35:59 +08:00。
- 输入：`images`，float32，`[1, 3, 640, 640]`。
- 输出：`output0`，`[1, 7, 8400]`，4 个框参数和 3 个类别分数。
- 类别：0 篮球 → `ball_basketball`，1 排球 → `ball_volleyball`，2 定位柱 → `pillar`。

部署配置位于 `src/rb_perception/config/perception.yaml`。模型路径按当前 Linux
工作区 `/home/user/robocup_basketball_ws/models/ball_20261011_v5.onnx` 配置；
更换工作区时同步修改 `model_path`。旧模型保留在 `/home/user/models/ball.onnx`。

每张处理图像都推理，不复用上帧框。推理线程保持 2/1，OpenCV 线程设为 2，
避免传统圆环检测默认开启 16 线程抢占 FAST-LIO 计算资源。
重复的定位柱颜色检测关闭。柱宽沿用旧配置的 0.20m，按框宽估距；
球径沿用篮球 0.24m、排球 0.21m。球的控制距离现在由标定后的相机/雷达
球面拟合提供；未标定或无可靠雷达观测时只画框，球距离为 NaN。
前后相机标定与 0.60m 车尾吸球口到球表面控制见
[`docs/ball_precision_setup.md`](../docs/ball_precision_setup.md)。

视觉基础日志保留配置/模型加载、每 5 秒帧率/耗时/最新目标摘要、限频异常和无图像告警。
本次运行日志为 `/tmp/rb-perception-20261012.log`；FAST-LIO 初始化日志为
`/tmp/rb-fastlio-20261012.log`。这些临时日志会在重启后消失。

FAST-LIO 独立初始化时，Linux 的
`/home/user/lidar_360/fast_prop_ws/src/FAST_LIO_WITH_PROPAGATE/config/mid360.yaml`
中 `publish.map_en` 设为 `false`。该版本的 `publish_map()` 每秒向
`pcl_wait_pub` 追加整帧并重发全部历史点，运行越久复制/序列化越慢，影响里程计及时性。
关闭的是 `/Laser_map` 累积全图输出；`/Odometry` 和 `/cloud_registered` 继续发布。
测球所需 `/cloud_registered_body` 通过补丁开启当前 IMU 体系扫描输出；
该新增输出仍待 Linux 通电后应用配置、部署和确认，不能视为已经运行。
原配置备份于 `/tmp/rb-fastlio-config-before-20261012.yaml`。

同一 FAST-LIO 版本的 Livox 输入队列从固定 20 帧改为可配置的小型可靠队列，
`common.lidar_queue_depth` 默认 5，允许 1–20；修改后重启 FAST-LIO 生效。
避免排队处理约 2 秒以前的点云；IMU 队列和几何标定参数保持原值。
独立依赖仓库的精确改动保存在 `tools/fastlio_latest_input.patch`，可在该依赖根目录
用 `git apply --check` 校验后应用。本次只编译 `fastlio_mapping` 目标。
加载 ROS 环境、结束旧 FAST-LIO 节点后，`python3 tools/fastlio_basic_logger.py`
可以单独运行定位并过滤逐帧调试输出；支持 `--executable`、`--config` 调整路径。

2026-10-12 最后一轮改动状态：SSH 在可配置队列版本的编译期间中断，
尚未确认该版本编译完成或替换运行文件。断连前运行的是可靠单帧队列版本：
30 秒收到 41 条里程计，消息年龄中位数 0.0237 秒，但定位有效标志仅 231/601，
因此还不能认为定位健康检查通过。需重新连接主机，完成小型可靠队列的部署及复验。

初始化成功和新模型完成推理不代表找球方向/距离已经校准；需用已知球距与角度复验。
