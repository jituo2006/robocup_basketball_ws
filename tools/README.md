# tools/ —— 只放「人工工具」

这里**不放测试**。测试已经全部搬到 `src/` 下，由 `colcon test` 管理（见下）。

## 目录约定

```
src/<各包>/test/       单元测试（纯逻辑、毫秒级、无进程）
src/rb_tests/          集成/系统测试 + 测试夹具
tools/                 人工工具（要人操作、看画面、按卷尺量）
docs/                  文档
test_artifacts/        测试与工具的输出（可随时删）
```

**为什么分开**：测试要能在 CI 里一键跑、失败要能定位；人工工具要人拿着卷尺/棋盘格
去操作，不该被 `colcon test` 当成测试跑。混在一起时 `tools/` 既不好找、也不受管理。

## 这里的工具

| 工具 | 用途 | 何时用 |
| --- | --- | --- |
| `tune_camera.py` | 现场调曝光/白平衡并写回配置 | 换场地/换灯光后 |
| `calibrate_camera.py` | 相机内参标定（含打印棋盘格） | 装好相机后一次 |
| `calibrate_color.py` | 用真球标定 HSV 阈值 | 赛前、拿到用球后 |
| `view_camera.py` | 实时看画面 + 检测框 | 调视觉时 |
| `click_goto.py` | 点地图让车走到指定点 | 调试/摆位 |
| `measure_chassis.py` | 实测轮径/轮距 | 上车标定时 |
| `preflight.sh` | 上车前逐项红绿灯预检 | **每次上车前** |
| `check_all.sh` | 一键跑离线+预检 | 提交前 |
| `check_lidar_net.py` | 雷达网络配置核对 | 雷达连不上时 |

## ⚠️ 带「兼容入口」的 9 个文件

下面这些文件的**实现已经搬到 `rb_tests` 包**（测试资产与主代码分离），
`tools/` 里只剩一个 5 行的转发壳，用于兼容文档和旧习惯：

```
tools/fake_board.py          →  ros2 run rb_tests fake_board
tools/fake_detections.py     →  ros2 run rb_tests fake_detections
tools/make_test_image.py     →  ros2 run rb_tests make_test_image
tools/publish_test_image.py  →  ros2 run rb_tests publish_test_image
tools/verify_offline.py      →  ros2 run rb_tests verify_offline
tools/verify_goto_avoid.py   →  ros2 run rb_tests verify_goto_avoid
tools/verify_lidar_replay.py →  ros2 run rb_tests verify_lidar_replay
tools/camera_check.py        →  ros2 run rb_tests camera_check
tools/vision_rate_check.py   →  ros2 run rb_tests vision_rate_check
```

新代码请直接用 `ros2 run rb_tests ...`（不依赖 cwd 与相对路径）。

## 跑测试

```bash
colcon build --symlink-install
colcon test                       # 全部（单元 + 集成）
colcon test-result --verbose      # 看失败详情

# 只跑单元测试（快，秒级）
colcon test --packages-select rb_mission rb_localization rb_perception rb_camera

# 只跑集成测试
colcon test --packages-select rb_tests
```

完整离线链路验证（会真的拉起 6 个节点，约 2 分钟）：

```bash
ros2 run rb_tests verify_offline
```

> ⚠️ 环境备注：本机用户装的 `dash` 包注册了一个 pytest 插件，其依赖的
> `typing_extensions` 版本过旧，导入即失败会让 pytest 整个起不来
> （症状是 `colcon test` 报 `pytest.missing_result`）。各包的 `pytest.ini`
> 里用 `-p no:dash` 显式禁掉了它，与本工程无关。
