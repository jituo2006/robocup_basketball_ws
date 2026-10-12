#!/usr/bin/env bash
# 给雷达工作空间打「用最新一帧输入」补丁 —— 修云点图不顺畅/延迟大
#
# ⚠️ 为什么需要它（现场实测数据）
#    FAST-LIO 订阅 /livox/lidar 时用的是 create_subscription(..., 20, cb)：
#    队列深度 20 + RELIABLE，而雷达是 10Hz。一旦上游稍慢，队列就积压，
#    FAST-LIO 处理的是【20 帧之前】的数据 ✗
#
#    实测（不是猜的）:
#       改前  /Odometry 消息年龄: 中位 1.379s   max 1.622s
#             /cloud_registered_body: 中位 1.381s
#             → 找球脚本要求 ≤0.3s，永远判"不新鲜"✗ → 车不动
#       改后（map_en:false）: 中位 0.024s  ✓✓ 快了 57 倍
#       队列 20 → 5 进一步压低积压 ✓
#
#    另外 mid360.yaml 里 map_en: true 会不停发布【全局累积地图】——
#    一个越来越大的点云，把 USB/网络/CPU 管线全占了，也是延迟大户 ✗
#
# 本脚本做三件事（幂等，可反复跑）:
#   ① laserMapping.cpp: 队列深度改成可配参数 common.lidar_queue_depth，默认 5
#   ② mid360.yaml:      map_en: false（关累积地图）scan_bodyframe_pub_en: true
#   ③ 重新编译 fast_lio（只在源码真的变了时才编译）
#
# ⚠️ 它【会修改 ~/lidar_360】（你自己的雷达工作空间）。每次改动前都先备份 ✓
#
# 用法:
#   bash tools/apply_lidar_patch.sh            # 只看会改什么（dry-run）
#   bash tools/apply_lidar_patch.sh --apply    # 真的改 + 编译
set -u

LIDAR_WS=${LIDAR_WS:-$HOME/lidar_360}
SRC="$LIDAR_WS/fastlio_ws/src/FAST_LIO"
INST="$LIDAR_WS/fastlio_ws/install/fast_lio/share/fast_lio/config"
CPP="$SRC/src/laserMapping.cpp"
YAML="$SRC/config/mid360.yaml"
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

echo "── 雷达「最新输入」补丁 ──"
echo "  工作空间: $LIDAR_WS"
[ -f "$CPP" ] || { echo "  ✗ 找不到 $CPP"; exit 1; }
[ -f "$YAML" ] || { echo "  ✗ 找不到 $YAML"; exit 1; }

changed=0

# ── ① 订阅队列深度 ───────────────────────────────────────────────
echo "── ① laserMapping.cpp 的订阅队列 ──"
if grep -q "lidar_queue_depth" "$CPP" 2>/dev/null; then
  echo "  ✓ 已经是可配队列深度（已打过 ✓）"
else
  echo "  ✗ 还是写死的 20（积压元凶）"
  changed=1
  if [ "$APPLY" = "1" ]; then
    cp "$CPP" "$CPP.bak.$(date +%Y%m%d-%H%M%S)"
    python3 - "$CPP" <<'PY'
import re, sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
old = ("sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>"
       "(lid_topic, 20, livox_pcl_cbk);")
new = ("// 队列深度可配，默认 5：20 会让 FAST-LIO 处理 20 帧之前的数据（实测积压 1.38s）\n"
       "            const int queue_depth = this->declare_parameter<int>(\"common.lidar_queue_depth\", 5);\n"
       "            if (queue_depth < 1 || queue_depth > 20)\n"
       "                throw std::invalid_argument(\"common.lidar_queue_depth must be in [1, 20]\");\n"
       "            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>"
       "(lid_topic, rclcpp::QoS(queue_depth).reliable(), livox_pcl_cbk);")
if old not in s:
    print("  ⚠️ 没找到那行（版本可能不同），请人工核对"); sys.exit(2)
p.write_text(s.replace(old, new, 1), encoding="utf-8")
print("  ✓ 已改成可配队列（默认 5）")
PY
  fi
fi

# ── ② 配置：关累积地图 / 发当前扫描 ─────────────────────────────
echo "── ② mid360.yaml ──"
for kv in "map_en:false" "scan_bodyframe_pub_en:true" "scan_publish_en:true"; do
  k=${kv%%:*}; want=${kv##*:}
  # ⚠️ 必须去掉行尾注释：yaml 里 `map_en: false   # true: publish Map cloud`
  #    直接取值会读到注释文字（"publishMapcloud"）而误判 ✗
  cur=$(grep -E "^\s*${k}:" "$YAML" 2>/dev/null | head -1 | sed 's/#.*//' | sed 's/.*:\s*//' | tr -d ' ')
  if [ "$cur" = "$want" ]; then
    echo "  ✓ $k = $want"
  else
    echo "  ✗ $k = ${cur:-未找到}（应为 $want）"
    changed=1
    if [ "$APPLY" = "1" ]; then
      cp -n "$YAML" "$YAML.bak.$(date +%Y%m%d-%H%M%S)" 2>/dev/null || true
      python3 - "$YAML" "$k" "$want" <<'PY'
import re, sys, pathlib
p, k, want = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
s = p.read_text(encoding="utf-8")
s2 = re.sub(rf"^(\s*{k}:\s*)\S+(.*)$", rf"\g<1>{want}\g<2>", s, count=1, flags=re.M)
p.write_text(s2, encoding="utf-8")
print(f"  ✓ 已把 {k} 设为 {want}")
PY
    fi
  fi
done
# install 那份常常是符号链接，若不是就同步一份
if [ -f "$INST/mid360.yaml" ] && [ ! -L "$INST/mid360.yaml" ]; then
  echo "  · install 下的 mid360.yaml 不是符号链接，同步一份"
  [ "$APPLY" = "1" ] && cp "$YAML" "$INST/mid360.yaml"
fi

# ── ③ 编译 ──────────────────────────────────────────────────────
if [ "$changed" = "0" ]; then
  echo "── ③ 无需改动，不编译 ──"
elif [ "$APPLY" != "1" ]; then
  echo "── ③ 上面有需要改的地方；加 --apply 才会真的改并编译 ──"
else
  echo "── ③ 重新编译 fast_lio ──"
  # ⚠️ 必须 set +u：ROS 的 setup.bash 会引用未绑定变量，
  #    而本脚本开了 set -u → 直接报 "AMENT_TRACE_SETUP_FILES: 未绑定的变量" ✗
  ( set +u; cd "$LIDAR_WS/fastlio_ws" \
    && source /opt/ros/humble/setup.bash \
    && source "$LIDAR_WS/ws_livox/install/setup.bash" 2>/dev/null \
    && colcon build --symlink-install --packages-select fast_lio 2>&1 | tail -4 )
fi

echo "── 生效方式 ──"
echo "  改完必须【重启 FAST-LIO】才生效: bash tools/boot.sh"
echo "  验证: bash tools/check_lidar_latency.sh"
