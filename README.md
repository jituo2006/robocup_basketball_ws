# robocup_basketball_ws

RoboCup 中国赛 **篮球机器人 · 自主机器人项目** 的上位机工作空间。

> 本工程是**新建**的，不是改造 ROBOCON 的代码。
> 可移植的部分（底盘运动学、CAN 驱动、弹射机构串口协议）从
> `/home/user/Code/rc_ws/2025RC_R1_ws` 抽出来，并按 RoboCup 规则重写了上层。

---

## 1. 包结构

| 包 | 语言 | 来源 | 作用 |
|---|---|---|---|
| `rb_msgs` | C++ | 新建 | 接口：`Detection` / `DetectionArray` / `RobotState` / `MissionStatus` / `Launch.srv` / `SetMission.srv` / `GotoPose.srv` |
| `rb_chassis` | C++ | **移植** | 全向底盘运动学 + CAN（DJI 板）。移植时修了 3 个问题（见 §3） |
| `rb_launcher` | C++ | **移植** | 弹射/运球/夹爪串口帧协议（未改协议本身），加了优雅降级与自动重连 |
| `rb_camera` | Python | 新建 | UVC 相机驱动：MJPG 协商、采集线程、掉线自愈、JPEG 压缩、锁定曝光/白平衡 |
| `rb_perception` | Python | 新建 | 视觉：球种 / 篮筐 / 传球架圆环 / 定位柱（含去畸变） |
| `rb_localization` | Python | 新建 | 2D EKF：里程计 + 绝对位姿 + 视觉定位柱 |
| `rb_mission` | Python | 新建 | 自主任务状态机（传球/投篮）+ 视觉避障 + `go_to_pose` |
| `rb_bringup` | - | 新建 | 启动入口与参数 |
| `rb_tests` | Python | 新建 | **集成/系统测试 + 测试夹具**（见 §7） |

## 2. 数据流

```
                     ┌──────────── rb_perception（视觉）────────────┐
   相机 ──image──▶   │ 球种 / 篮筐 / 传球架圆环 / 定位柱            │
                     └───────────────┬──────────────────────────────┘
                                     │ DetectionArray
   底盘/定位板/雷达 ──/odom──────────▶ rb_localization（2D EKF）
                                     │ PoseWithCovarianceStamped
                                     ▼
                              rb_mission（状态机）
                                     │ /cmd_vel            │ Launch.srv
                                     ▼                     ▼
                              rb_chassis（CAN）      rb_launcher（串口）
```

## 3. 移植时修掉/改进的东西（都是原代码里的真实问题）

| # | 位置 | 原问题 | 现在 |
|---|---|---|---|
| 1 | `omni_chassis::execute` | `clamp(...)` 的**返回值被丢弃**，所以限速从来没生效 | 正确赋值 + 新增斜率（加速度）限制（原来 `limit_acc` 设了但没用） |
| 2 | 同文件 | **没有 /cmd_vel 超时保护**，收不到新指令会一直沿用最后一条速度（这是 2026-07-08「延迟窜车、紧急断电」事故的成因） | 补上 200 ms 超时 → 下发零轮速，退出时也发零 |
| 3 | 同文件 | 每周期都 `RCLCPP_INFO`，日志洪泛 | 改为限频 DEBUG + 发布 `/chassis/wheel_speed` |
| 4 | `ballrobot/R1_server` | 打不开串口时**抛未捕获异常直接 abort** | 降级为 `ok=false` + 周期性自动重连，节点不崩 |
| 5 | 全部参数 | 硬编码在 `main()` 里 | 全部改成 ROS 参数 / YAML |
| 6 | — | 无离线验证手段 | 加 `dry_run` 模式与 4 个离线工具 |
| 7 | `ratio`（减速比） | 被 `setParameter` 设置但**从未参与计算** | 变成显式乘子，**默认必须填 1.0** 才与原行为一致 |

## 4. 快速开始

```bash
cd /home/user/robocup_basketball_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 不需要任何硬件，验证整条软件链路
ros2 launch rb_bringup bringup_fsm_test.launch.py
# 另一个终端
ros2 service call /rb_mission/set_mission rb_msgs/srv/SetMission "{mission: PASS}"
ros2 topic echo /mission/status
```

**一键检查（离线验证 + 上车预检）**：

```bash
./tools/check_all.sh
```

详细步骤见 `docs/`：

| 文档 | 内容 |
|---|---|
| `docs/00_上车就绪清单.md` | 到车边照着执行的清单 |
| `docs/01_离线验证.md` | 无硬件跑通全链路 |
| `docs/02_上车调通.md` | **逐步上车清单**（按硬件到位顺序） |
| `docs/03_标定与调参.md` | 相机/颜色/底盘参数/场地几何怎么标 |
| `docs/05_雷达与视觉定位启动流程.md` | **雷达+视觉定位的启动与标定流程**（当前采用的方案） |
| `docs/06_FAST-LIO是什么.md` | **FAST-LIO 是什么、怎么用、和本工程代码的关系** |
| `docs/04_已知限制.md` | 还没做的事，以及按 2026 规则要补什么 |

## 6. 目录

```
robocup_basketball_ws/
├── src/
│   ├── rb_msgs/ rb_chassis/ rb_launcher/ rb_camera/
│   ├── rb_perception/ rb_localization/ rb_mission/ rb_bringup/
│   └── rb_tests/              集成测试 + 测试夹具（含自己的 test/）
│       ├── rb_tests/fakes/    假底盘 / 假检测 / 合成图 / 图发布
│       ├── rb_tests/checks/   离线验证 / 服务验证 / 相机自检
│       └── test/              集成测试（colcon test 会跑）
├── tools/         人工工具（标定、调参、看图、预检）+ 兼容入口
├── docs/          文档
└── test_artifacts/  测试产物（图、日志，可随时删）
```

## 7. 测试与工程约定

**测试分两层，都在 `colcon test` 之下：**

| 层 | 位置 | 特点 | 跑法 |
| --- | --- | --- | --- |
| 单元测试 | `src/<各包>/test/` | 纯逻辑、无进程、毫秒级 | `colcon test --packages-select rb_mission ...` |
| 集成测试 | `src/rb_tests/test/` | 真拉节点、验数据流、秒级 | `colcon test --packages-select rb_tests` |

```bash
colcon build --symlink-install
colcon test                       # 全部
colcon test-result --verbose      # 失败详情
ros2 run rb_tests verify_offline  # 完整离线链路（约 2 分钟，含 6 节点启动）
```

**为什么测试夹具（假底盘等）放在 `rb_tests` 而不是 `tools/`：**
它们同时被集成测试和 `bringup_*` launch 文件使用，属于测试资产；
放 `tools/` 会与人工工具混在一起，既不受 `colcon test` 管理也不好找。
`tools/` 里保留的是 5 行转发壳，只为兼容旧命令。

> ⚠️ 环境备注：本机用户装的 `dash` 包注册了 pytest 插件，其 `typing_extensions`
> 依赖过旧导致导入失败、pytest 整个起不来（症状 `pytest.missing_result`）。
> 各包的 `pytest.ini` 用 `-p no:dash` 显式禁掉，与本工程无关。

## 8. 规则依据

2026 规则已提取为可检索文本：`/home/user/篮球规则/2026规则.txt`。
代码里凡是规则相关的地方都标了 §章节号，例如：

- `§2.3-5` 禁止遥控 → `rb_mission` 是唯一的 `/cmd_vel` 来源
- `§2.3-7` 启动延迟 5~15s → `START_DELAY` 阶段
- `§2.5` 传球/投篮环节**目标球与干扰球角色互换** → `mission.yaml` 的 `mission_labels`
- `§2.5` 得分与位置强相关 → 区域合规判定与三分线判定
