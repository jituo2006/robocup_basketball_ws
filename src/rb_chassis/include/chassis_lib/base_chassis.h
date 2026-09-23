#ifndef RB_BASE_CHASSIS_H
#define RB_BASE_CHASSIS_H

#include <chrono>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * 底盘基类。
 *
 * 移植自 ROBOCON 2025 篮球工程（2025chassislib），并做了以下修改：
 *   1. 新增 /cmd_vel 指令超时检测（原来没有，底盘会一直沿用最后一条速度）
 *      来源：2026_R1 在 2026-07-08 一次"延迟窜车、紧急断电"事故后的修复
 *   2. 新增限速/限加速的实际生效逻辑
 *   3. 记录是否收到过里程计，便于排查"车按世界系还是车体系动"
 */
class Chassis : public rclcpp::Node {
protected:
  geometry_msgs::msg::Twist target_vel;
  nav_msgs::msg::Odometry current_odom_;

  double limit_vel = 1.0;      // 速度上限 (m/s 或 rad/s)
  double limit_acc = 2.0;      // 加速度上限 (m/s^2 或 rad/s^2)
  double width = 0.5;          // 轮距（左右轮中心距，m）
  double length = 0.5;         // 轴距（前后轮中心距，m）
  double wheel_radius = 0.1;   // 驱动轮半径 (m)
  double ratio = 1.0;          // 减速比

  bool dry_run_ = false;                        // true 时只解算不下发 CAN（无需硬件自测）
  bool has_odom_ = false;                       // 是否收到过里程计
  bool warned_no_odom_ = false;

  std::chrono::steady_clock::time_point last_velocity_command_time_{};
  std::chrono::milliseconds cmd_vel_timeout_{200};

  // 加速度限制用的"上一条已下发速度"
  geometry_msgs::msg::Twist last_sent_vel_;
  std::chrono::steady_clock::time_point last_execute_time_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

public:
  Chassis(const std::string &node_name, const std::string &odom_topic, const std::string &vel_topic);

  void setVelocity(const geometry_msgs::msg::Twist &cmd_vel);
  void updateOdom(const nav_msgs::msg::Odometry &odom);

  bool hasFreshVelocityCommand() const;
  void setDryRun(bool dry_run) { dry_run_ = dry_run; }
  bool dryRun() const { return dry_run_; }
  void setCommandTimeout(std::chrono::milliseconds timeout) { cmd_vel_timeout_ = timeout; }
  std::chrono::milliseconds commandTimeout() const { return cmd_vel_timeout_; }

  /// 按 limit_vel / limit_acc 对目标速度做限幅与斜率限制，返回处理后的速度。
  geometry_msgs::msg::Twist applyLimits();

  virtual void initialize() = 0;
  virtual void execute() = 0;
};

#endif  // RB_BASE_CHASSIS_H
