#include "chassis_lib/base_chassis.h"

#include <algorithm>
#include <cmath>

namespace {
double slew(double target, double previous, double max_delta) {
  const double delta = target - previous;
  if (delta > max_delta) return previous + max_delta;
  if (delta < -max_delta) return previous - max_delta;
  return target;
}
}  // namespace

Chassis::Chassis(const std::string &node_name, const std::string &odom_topic, const std::string &vel_topic)
    : Node(node_name) {
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&Chassis::updateOdom, this, std::placeholders::_1));
  vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      vel_topic, 10, std::bind(&Chassis::setVelocity, this, std::placeholders::_1));

  last_execute_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "Chassis 已就绪: odom=%s cmd_vel=%s timeout=%ldms",
              odom_topic.c_str(), vel_topic.c_str(),
              static_cast<long>(cmd_vel_timeout_.count()));
}

void Chassis::setVelocity(const geometry_msgs::msg::Twist &cmd_vel) {
  target_vel = cmd_vel;
  last_velocity_command_time_ = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(get_logger(), "收到 /cmd_vel: x=%.3f y=%.3f wz=%.3f",
               target_vel.linear.x, target_vel.linear.y, target_vel.angular.z);
}

void Chassis::updateOdom(const nav_msgs::msg::Odometry &odom) {
  current_odom_ = odom;
  has_odom_ = true;
}

bool Chassis::hasFreshVelocityCommand() const {
  return (std::chrono::steady_clock::now() - last_velocity_command_time_) <= cmd_vel_timeout_;
}

geometry_msgs::msg::Twist Chassis::applyLimits() {
  geometry_msgs::msg::Twist out = target_vel;

  // ① 限幅
  //
  // ⚠️ 移植时修复的 bug：原实现写的是 `clamp(target_vel.linear.x, ...)`，
  //    返回值被丢弃，所以限速**从来没有生效**。这里必须赋值。
  out.linear.x = std::clamp(out.linear.x, -limit_vel, limit_vel);
  out.linear.y = std::clamp(out.linear.y, -limit_vel, limit_vel);
  out.angular.z = std::clamp(out.angular.z, -limit_vel, limit_vel);

  // ② 斜率（加速度）限制 —— 原实现的 limit_acc 被设置但从未使用
  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_execute_time_).count();
  last_execute_time_ = now;
  if (dt <= 0.0 || dt > 0.5) dt = 0.01;  // 首帧或长间隔，按 10ms 处理

  const double max_delta = std::max(limit_acc, 1e-3) * dt;
  out.linear.x = slew(out.linear.x, last_sent_vel_.linear.x, max_delta);
  out.linear.y = slew(out.linear.y, last_sent_vel_.linear.y, max_delta);
  out.angular.z = slew(out.angular.z, last_sent_vel_.angular.z, max_delta);

  last_sent_vel_ = out;
  return out;
}
