# 找球全流程命令行手册

> 场景：从冷启动开始，一路到「原地转圈找到球 → 走到球前 0.60m → 转身背对球」。
> 每一步都有对应命令，照抄即可。**不需要绕场时一律加 `--skip-perimeter`。**

---

## 0. 上电（物理动作，不是命令）

```
□ 工控机 / NUC 上电
□ 电机板那一路上电        ← 和工控机往往是【两路独立电源】，别只开一个
□ 雷达（Livox MID360）上电 ← 由机器人供电
```

上电后立刻自检：

```bash
# 雷达：应能 ping 通、网卡应 UP
ping -c 2 192.168.1.138
ip -br addr show enp114s0          # 期望 UP 192.168.1.110/24

# CAN：适配器在、接口 UP
lsusb | grep 0c72
ip -br link show can0
```

---

## 1. 一键自检（什么都不启动，只看）

```bash
rbws
bash tools/check_ready.sh
```

一条命令告诉你「现在能不能开车 / 看地图」。**缺什么它会直接列出来。**

---

## 2. 启动整套

```bash
rbws
bash tools/boot.sh
```

它会依次拉起：Livox 驱动 → FAST-LIO → 相机（前+后）→ 感知（前+后）→ 定位 → 底盘 → 机构 → 任务。

⚠️ **`boot.sh` 会先清理残留进程**，所以重复执行是安全的。

等待约 40~50 秒，用这条确认 7 个关键进程都在：

```bash
for n in rb_chassis_node mission_node localization_node camera_node \
         perception_node livox_ros_driver2_node fastlio_mapping; do
  printf "  %-24s %s\n" "$n" \
    "$(pgrep -x "${n:0:15}" >/dev/null && echo '✓' || echo '✗')"
done
```

> 💡 `"${n:0:15}"` 不是多余的：Linux 的 `/proc/<pid>/comm` **最多 15 字符**，
> `localization_node`、`rb_launcher_node`、`livox_ros_driver2_node` 都会被截断，
> 拿全名去 `pgrep -x` 永远匹配不上。

---

## 3. 启动控制台（**必须在 boot.sh 之后**）

```bash
bash tools/console.sh
```

打开 http://127.0.0.1:8090

> ⚠️ **顺序很重要**：控制台若先于底盘启动，FastRTPS 的 writer/reader 有时匹配不上，
> 现象是「话题上有数据但底盘收不到」。先起 boot.sh、再起控制台可避免。

---

## 4. 找球（核心）

### 4.1 先做「不动车」检查（强烈建议）

```bash
python3 tools/test_perimeter_ball.py --skip-perimeter --dry --yes
```

期望输出：

```
[1/5] 检查节点      ✓ 底盘  ✓ 定位  ✓ 感知  ✓ FAST-LIO
[2/5] 接管 /cmd_vel  ✓ mission 没在跑，可安全接管
[3/5] 读位姿        ✓ 出发点 (+1.50, +8.00, +0.0°)
                    ✓ 场地 14.0 × 7.5 m，绕场 inset=0.70m
--dry：前置条件都过了，没动车 ✓
```

**只要这里报「缺: …」，就先别往下走**，回到第 2 步。

### 4.2 正式跑（车会真的动）

```bash
# ⚠️ 车周围清空、手在电源开关上、另开终端准备好急停
python3 tools/test_perimeter_ball.py --skip-perimeter --takeover --yes
```

- `--skip-perimeter` → **不绕场**，直接找球
- `--takeover` → 自动 SIGTERM 掉 mission 节点（它在 IDLE 时也发零速，会抢 `/cmd_vel`）
- `--yes` → 跳过「确认要动车」的提问

### 4.3 从 UI 跑（等价）

```
http://127.0.0.1:8090  →  「🔍 寻球 → 走到球前 0.60m → 转身背对球」
```

它执行的就是 4.2 那条命令（`--skip-perimeter --takeover --yes`）。
日志看右下「实时日志」面板。

**跑完记得点「🔄 重启任务节点」** —— 因为 `--takeover` 把 mission 杀了，
之后「回出发点 / 去投篮点 / 起任务」都会报「服务不可用」。

---

## 5. 常用参数

```bash
python3 tools/test_perimeter_ball.py --skip-perimeter --takeover --yes \
  --ball-label ball_basketball     # 目标球标签（YOLO 有时把篮球认成 ball_volleyball）
  --search-wz 0.4                  # 找球自转角速度 rad/s（慢了找不到、快了相机糊）
  --speed 0.20                     # 平移速度上限 m/s（默认偏慢，安全）
  --max-ang 0.5                    # 角速度上限 rad/s
  --yaw-tol 0.06                   # 朝向容差 rad
  --tol 0.10                       # 到点容差 m
  --stop-distance 0.60            # 【固定 0.60m，不要改】容差在 ball_approach.yaml
```

绕场相关（只在不加 `--skip-perimeter` 时有意义）：

```bash
  --inset 0.70                     # 绕场时离边界距离 m
  --approach-radius 0.45           # 绕场航点减速半径
  --approach-speed 0.12            # 绕场航点接近速度
```

查所有参数：

```bash
python3 tools/test_perimeter_ball.py --help
```

---

## 6. 只想验证下层「精确找球」（更安全，默认不动车）

```bash
# 只校验配置，不发任何运动指令 ✓
python3 tools/precision_ball_approach.py

# 真下发（会动！）
python3 tools/precision_ball_approach.py --execute
```

---

## 7. 急停

```bash
# 另开一个终端备好
bash tools/panic.sh
```

或者直接按电源开关 / UI 上的「🛑 急停」。

---

## 8. 排错速查

### 现象：脚本在 `[1/5]` 就退出，一个动作都不做

```bash
bash tools/check_ready.sh
```

脚本的判据是：**底盘 / 定位 / 感知 / FAST-LIO 四个进程都得在**，
缺任何一个就 `return 1`。

### 现象：卡在 `waiting for fresh radar odometry`

雷达消息太旧（判据 ≤ 0.3 秒）。用这条精确测：

```bash
ros2 topic echo /Odometry --once --field header
python3 -c "import time; print('now =', time.time())"
```

两者差超过 0.3 秒就是延迟问题。**已知原因**：FAST-LIO 的 `map_en: true`
会不停发布全局累积地图，把整条管线拖慢到 **1.38 秒**。
改法（在 `~/lidar_360`，需谨慎）：

```bash
grep -n "map_en" ~/lidar_360/fastlio_ws/src/FAST_LIO/config/mid360.yaml
# 期望：map_en: false      ← true 就会拖慢 ✗
# 改完必须重启 FAST-LIO 才生效
```

### 现象：卡在 `waiting for one valid front target (no_unambiguous_sphere)`

**相机前面没有球**：

```bash
ros2 topic echo /perception/detections --once | grep label
```

如果只有 `rack_ring` 而没有 `ball_basketball` / `ball_volleyball`，
就是视野里没有球 → **把球放到车前方 0.6~2m 处。**

### 现象：UI 点「寻球」完全没反应

```bash
# 1) 控制台活着吗
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/

# 2) /cmd_vel 上有没有【别的发布者】—— 找球要求恰好只有它自己
ros2 topic info /cmd_vel --verbose | grep -E "Publisher count|Node name"

# 3) 控制台日志面板有没有脚本输出
```

控制台**不能常驻占用 `/cmd_vel`**（`precision_ball_approach.py` 的独占检查要求
`count_publishers("/cmd_vel") == 1`）。正常应是「点动时才有、点完就释放」。

### 现象：底盘一直报 `Error writing to CAN bus` / `No buffer space available`

CAN 发送队列满了：

```bash
ip -details link show can0 | grep -oP 'qlen \K[0-9]+'
sudo ip link set can0 txqueuelen 1000     # 临时；boot.sh 每次启动会设
bash tools/can_recover.sh                 # 或者用这个
```

⚠️ 队列加大只是**让报文能发出去**；如果电机还是不回，是**总线/供电**问题：

```bash
ip -statistics link show can0 | grep -A1 RX:     # RX 涨不涨
candump can0                                     # 总线上有什么帧
```

### 现象：进程都在，但一个话题都收不到

`ROS_DOMAIN_ID` 不一致。全工程用 **2**：

```bash
echo $ROS_DOMAIN_ID      # 期望 2
```

### 现象：「一键启动」报「缺少节点」

那是**误报**的话，看名字是不是长于 15 字符的进程（见第 2 步的说明）。

---

## 9. 附录：两个易踩的坑

```bash
# ❌ pgrep -f 会匹配到【你自己】——命令行里就带着那个名字
pgrep -f "console_app.py"      # 危险：可能把自己的 shell 也列进去并 kill 掉

# ✅ 两个安全写法
pgrep -f "[c]onsole_app.py"    # 正则技巧：自己的命令行里是字面量 [c]，不匹配
pgrep -x console_app.py        # 精确匹配进程名（注意 15 字符上限）

# ❌ nohup ... & 起的进程可能被回收
# ✅ 用 setsid 彻底脱离会话
setsid bash tools/console.sh > /tmp/console.log 2>&1 < /dev/null &
```

**控制台的日志存在内存里**，重启就清空 —— 排障时如果重启过控制台，
之前的证据就没了。看日志请**当场看**，或改脚本让它同时写文件。
