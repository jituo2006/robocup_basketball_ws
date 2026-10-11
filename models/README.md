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
球径沿用篮球 0.24m、排球 0.21m。以上单目距离仍需实测标定。

视觉基础日志保留配置/模型加载、每 5 秒帧率/耗时/最新目标摘要、限频异常和无图像告警。
本次运行日志为 `/tmp/rb-perception-20261012.log`；FAST-LIO 初始化日志为
`/tmp/rb-fastlio-20261012.log`。这些临时日志会在重启后消失。

FAST-LIO 独立初始化时，Linux 的
`/home/user/lidar_360/fast_prop_ws/src/FAST_LIO_WITH_PROPAGATE/config/mid360.yaml`
中 `publish.map_en` 设为 `false`。该版本的 `publish_map()` 每秒向
`pcl_wait_pub` 追加整帧并重发全部历史点，运行越久复制/序列化越慢，影响里程计及时性。
关闭的是 `/Laser_map` 累积全图输出；`/Odometry` 和 `/cloud_registered` 继续发布。
原配置备份于 `/tmp/rb-fastlio-config-before-20261012.yaml`。

初始化成功和新模型完成推理不代表找球方向/距离已经校准；需用已知球距与角度复验。
