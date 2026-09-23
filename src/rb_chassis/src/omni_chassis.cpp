#include "chassis_lib/omni_chassis.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>

#include "utils/utils.h"

OmniChassis::OmniChassis(const std::string &node_name, const std::string &odom_topic,
                         const std::string &vel_topic, uint32_t board_id,
                         const std::shared_ptr<Can> &can_handle)
    : Chassis(node_name, odom_topic, vel_topic) {
  dji_board = std::make_shared<DJIBoard>(board_id, can_handle);

  // 退出时确保零速（原实现只在析构里 MotorOff，节点被 kill -9 时来不及执行）
  rclcpp::on_shutdown([this]() {
    for (int i = 1; i <= 4; ++i) {
      dji_board->VelCtrl(i, 0);
    }
  });
}

OmniChassis::~OmniChassis() {
  if (dry_run_) return;
  sendZeroVelocity();
  for (int i = 1; i <= 4; i++) {
    dji_board->MotorOff(i);
  }
}

void OmniChassis::initialize() {
  if (dry_run_) {
    RCLCPP_WARN(get_logger(), "dry_run 模式：跳过电机使能，只做运动学解算（无 CAN 输出）");
    return;
  }
  for (int i = 1; i <= 4; i++) {
    dji_board->VelCfg(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    dji_board->MotorOn(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  RCLCPP_INFO(get_logger(), "底盘已使能 4 个电机（board_id=%u）", 0u);
}

void OmniChassis::sendZeroVelocity() {
  if (dry_run_) {
    wheel_speed.fill(0);
    return;
  }
  for (int i = 1; i <= 4; ++i) {
    dji_board->VelCtrl(i, 0);
  }
  wheel_speed.fill(0);
}

void OmniChassis::execute() {
  // ---------- ① 指令超时保护 ----------
  //
  // 原 2025 篮球实现**没有**这一层：一旦收不到新 /cmd_vel，会一直沿用最后一条速度。
  // 这正是 2026-07-08 那次"发了指令当时不动、随后突然窜车、紧急断电"的成因。
  // 这里补上：超时后只发一次零速并回到安全状态。
  if (!hasFreshVelocityCommand()) {
    if (!timeout_zero_sent_) {
      sendZeroVelocity();
      timeout_zero_sent_ = true;
      RCLCPP_WARN(get_logger(), "cmd_vel 超时 %ldms，已下发零轮速",
                  static_cast<long>(commandTimeout().count()));
    }
    return;
  }
  timeout_zero_sent_ = false;

  // ---------- ② 限幅 + 加速度限制 ----------
  const geometry_msgs::msg::Twist cmd = applyLimits();

  // ---------- ③ 全向运动学 ----------
  //
  // yaw 取自里程计的 position.z（本队约定：yaw 塞在 z 里）。
  // 没有里程计时 yaw=0，此时指令按车体系直接下发 —— 方向仍然对，只是不做世界系旋转。
  if (!has_odom_ && !warned_no_odom_) {
    RCLCPP_WARN(get_logger(),
                "尚未收到里程计，yaw 按 0 处理（指令将作为车体系速度下发）。"
                "若定位未起来，这是预期行为。");
    warned_no_odom_ = true;
  }
  const double yaw = has_odom_ ? current_odom_.pose.pose.position.z : 0.0;

  const double vx = cmd.linear.x * std::cos(yaw) + cmd.linear.y * std::sin(yaw);
  const double vy = cmd.linear.y * std::cos(yaw) - cmd.linear.x * std::sin(yaw);
  // 旋转方向符号：
  //   原 2025 代码这里是 `-cmd.angular.z`。实车测试发现顺逆反了（正 angular.z 却顺时针转），
  //   说明这辆车的轮子安装朝向与公式默认相反，故把默认符号从 -1 改为 +1，
  //   并做成可配置（angular_z_sign），下次装轮方向变了不用改代码。
  const double wz = cmd.angular.z * angular_z_sign_;

  // ⚠️ ratio（减速比）在原实现里被 setParameter 设置但**从未参与计算**。
  //    这里保留为显式乘子，默认必须填 1.0 才能与原行为一致。
  //    只有在确认电机侧还有一级减速时才改为真实比值。
  const double factor = 60.0 / (2.0 * M_PI * wheel_radius) * ratio;

  const double c = std::cos(M_PI_2 / 2.0);  // cos(45°)
  const double arm = std::hypot(width / 2.0, length / 2.0);

  wheel_speed[0] = static_cast<int16_t>((+vx * c - vy * c - wz * arm) * factor);
  wheel_speed[1] = static_cast<int16_t>((+vx * c + vy * c - wz * arm) * factor);
  wheel_speed[2] = static_cast<int16_t>((-vx * c + vy * c - wz * arm) * factor);
  wheel_speed[3] = static_cast<int16_t>((-vx * c - vy * c - wz * arm) * factor);

  if (!dry_run_) {
    for (int i = 1; i <= 4; i++) {
      dji_board->VelCtrl(i, wheel_speed[i - 1]);
    }
  }

  // 原实现每周期都 RCLCPP_INFO，导致日志洪泛。改为限频调试输出。
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                        "vx=%.3f vy=%.3f wz=%.3f wheels=[%d %d %d %d]",
                        vx, vy, wz, wheel_speed[0], wheel_speed[1], wheel_speed[2], wheel_speed[3]);
}

void OmniChassis::setParameter(double limit_vel, double limit_acc, double width, double length,
                               double wheel_radius, double ratio) {
  this->limit_vel = limit_vel;
  this->limit_acc = limit_acc;
  this->width = width;
  this->length = length;
  this->wheel_radius = wheel_radius;
  this->ratio = ratio;
  RCLCPP_INFO(get_logger(),
              "底盘参数: 限速=%.2f 限加速=%.2f 轮距=%.3fx%.3f 轮径=%.3f 减速比=%.2f",
              limit_vel, limit_acc, width, length, wheel_radius, ratio);
}
