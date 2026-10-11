// RoboCup 篮球机器人 · 底盘节点
//
// 移植自 ROBOCON 2025 篮球工程的 `chassis/omni_driver`，主要变化：
//   1. 所有物理参数改为 ROS 参数（原来硬编码在 main 里）
//   2. 新增 dry_run 模式：无 CAN 也能验证运动学解算
//   3. 新发布 /chassis/wheel_speed，便于离线核对（不用接电机也能看解算结果）
//   4. 修掉原实现的限速失效 bug、补上指令超时零速保护
//
// 用法：
//   ros2 run rb_chassis rb_chassis_node                        # 真实运行
//   ros2 run rb_chassis rb_chassis_node --ros-args -p dry_run:=true   # 离线自测

#include <csignal>
#include <chrono>
#include <memory>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include "bupt_can/bupt_can.h"
#include "chassis_lib/omni_chassis.h"

namespace {
// 关终端窗口时，进程收到的是 **SIGHUP**（不是 SIGINT）。rclcpp 默认只处理
// SIGINT，收到 SIGHUP/SIGTERM 会直接死掉 —— 主循环末尾的 sendZeroVelocity()
// 根本来不及执行，电机板失去通信后会按"最后一条指令"短暂窜一下。
// 这里把 SIGHUP/SIGTERM 也接到 rclcpp::shutdown()：它会唤醒主循环、让
// rclcpp::ok() 变 false，从而走完"退出循环 → 发零速"这段优雅退出。
// （rclcpp 自己处理 SIGINT 时也是调用 rclcpp::shutdown()，同一个套路。）
void signal_shutdown(int sig) {
  (void)sig;
  rclcpp::shutdown();
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 先建一个临时节点读参数，再交给 OmniChassis（它自己也是 Node）
  auto param_node = std::make_shared<rclcpp::Node>("rb_chassis_param_probe");
  const auto can_interface = param_node->declare_parameter<std::string>("can_interface", "can0");
  const auto odom_topic = param_node->declare_parameter<std::string>("odom_topic", "/odom");
  const auto cmd_vel_topic = param_node->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const auto board_id = param_node->declare_parameter<int>("board_id", 1);
  const auto limit_vel = param_node->declare_parameter<double>("limit_vel", 1.0);
  const auto limit_acc = param_node->declare_parameter<double>("limit_acc", 1.5);
  // 停车（cmd_vel 超时/急停）专用减速度：要快，但不能是阶跃（会过冲反转）
  const auto stop_accel = param_node->declare_parameter<double>("stop_accel", 3.0);
  // CAD 设计初值，与 chassis.yaml 一致；实车标定后可用 ROS 参数覆盖。
  const auto width = param_node->declare_parameter<double>("width", 0.477059538);
  const auto length = param_node->declare_parameter<double>("length", 0.477059538);
  const auto wheel_radius = param_node->declare_parameter<double>("wheel_radius", 0.0765);
  const auto ratio = param_node->declare_parameter<double>("ratio", 1.0);
  const auto timeout_ms = param_node->declare_parameter<int>("cmd_vel_timeout_ms", 200);
  const auto period_ms = param_node->declare_parameter<int>("control_period_ms", 10);
  const auto dry_run = param_node->declare_parameter<bool>("dry_run", false);
  const auto angular_z_sign = param_node->declare_parameter<double>("angular_z_sign", 1.0);
  const auto config_file = param_node->declare_parameter<std::string>("config_file", "");
  if (!config_file.empty()) {
    RCLCPP_ERROR(param_node->get_logger(),
      "底盘不读取 config_file；请用 --params-file %s 或 chassis.launch.py", config_file.c_str());
    rclcpp::shutdown();
    return 2;
  }
  if (period_ms <= 0 || timeout_ms <= 0 || can_interface.empty() ||
      can_interface.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-.") != std::string::npos) {
    RCLCPP_ERROR(param_node->get_logger(), "CAN 接口名或控制周期/超时参数无效");
    rclcpp::shutdown();
    return 2;
  }
  // 每个真实 CAN 接口只允许一个底盘进程；干跑不占用硬件锁。
  // 进程退出后内核释放 flock，锁文件保留以免删除/重建导致两个 inode。
  int owner_fd = -1;
  if (!dry_run) {
    const auto lock_path = "/tmp/rb_chassis-" + std::to_string(getuid()) + "-" + can_interface + ".lock";
    owner_fd = open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (owner_fd < 0 || flock(owner_fd, LOCK_EX | LOCK_NB) != 0) {
      RCLCPP_ERROR(param_node->get_logger(), "无法独占 %s；已有底盘实例或锁文件不可用", can_interface.c_str());
      if (owner_fd >= 0) close(owner_fd);
      rclcpp::shutdown();
      return 2;
    }
  }
  // 全局 __node 重映射也作用于参数探测节点，读完即释放，避免同名 ROS 节点。
  param_node.reset();

  // DJIBoard 必须拿到非空 Can；干跑只构造，不打开 SocketCAN。
  // 修复版 Can 可在未启动或接口掉线时正常析构，无需泄漏对象。
  std::shared_ptr<Can> can_handle;
  can_handle = std::make_shared<Can>(can_interface);

  auto node = std::make_shared<OmniChassis>("rb_chassis", odom_topic, cmd_vel_topic,
                                            static_cast<uint32_t>(board_id), can_handle);

  node->setDryRun(dry_run);
  node->setCommandTimeout(std::chrono::milliseconds(timeout_ms));
  node->setAngularZSign(angular_z_sign);
  node->setParameter(limit_vel, limit_acc, width, length, wheel_radius, ratio);
  node->setStopAccel(stop_accel);

  auto wheel_pub = node->create_publisher<std_msgs::msg::Int16MultiArray>("/chassis/wheel_speed", 10);

  if (!dry_run) {
    try {
      can_handle->can_start();
    } catch (const std::exception &e) {
      node->setDryRun(true);
      RCLCPP_ERROR(node->get_logger(), "CAN 启动失败: %s", e.what());
      rclcpp::shutdown();
      return 2;
    }
    RCLCPP_INFO(node->get_logger(), "CAN 已启动: %s", can_interface.c_str());
  }
  node->initialize();

  // 关终端(SIGHUP)/被 kill(SIGTERM) 时也走优雅退出（发零速），见上面的 signal_shutdown
  std::signal(SIGHUP, signal_shutdown);
  std::signal(SIGTERM, signal_shutdown);

  rclcpp::Rate rate(1000.0 / static_cast<double>(period_ms));
  bool can_failed = false;
  while (rclcpp::ok()) {
    if (!dry_run && !can_handle->isStarted()) {
      can_failed = true;
      RCLCPP_ERROR(node->get_logger(), "CAN 发送失败，底盘停止接收运动指令；通信恢复后需重新启动，不能确认停车报文已送达");
      break;
    }
    node->execute();

    std_msgs::msg::Int16MultiArray msg;
    const auto &w = node->wheelSpeed();
    msg.data.assign(w.begin(), w.end());
    wheel_pub->publish(msg);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  node->sendZeroVelocity();
  rclcpp::shutdown();
  node.reset();
  can_handle.reset();
  if (owner_fd >= 0) close(owner_fd);
  return can_failed ? 2 : 0;
}
