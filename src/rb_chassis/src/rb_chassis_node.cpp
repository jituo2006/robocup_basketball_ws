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

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include "bupt_can/bupt_can.h"
#include "chassis_lib/omni_chassis.h"

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
  const auto width = param_node->declare_parameter<double>("width", 0.40);
  const auto length = param_node->declare_parameter<double>("length", 0.40);
  const auto wheel_radius = param_node->declare_parameter<double>("wheel_radius", 0.10);
  const auto ratio = param_node->declare_parameter<double>("ratio", 1.0);
  const auto timeout_ms = param_node->declare_parameter<int>("cmd_vel_timeout_ms", 200);
  const auto period_ms = param_node->declare_parameter<int>("control_period_ms", 10);
  const auto dry_run = param_node->declare_parameter<bool>("dry_run", false);
  const auto angular_z_sign = param_node->declare_parameter<double>("angular_z_sign", 1.0);

  // ⚠️ 注意：这里**必须**始终传一个真实的 Can 对象，即使是 dry_run。
  //
  // 原因（MotorLib-For-Linux 的一个 bug）：
  //   DJIBoard::DJIBoard(boardId, can_handle) 里写着
  //       if (can_handle == nullptr) {
  //           this->can_handle = std::make_shared<Can>("can0");
  //           can_handle->can_start();     // ← 用的是参数（nullptr），不是成员！
  //       }
  //   所以传 nullptr 会直接解引用空指针段错误。
  //   这里传真实对象即可绕开该分支；dry_run 下不调用 can_start()，
  //   且 execute() 里对 VelCtrl 有 dry_run 守卫，所以不会真的发 CAN。
  //
  // TODO 上游修复：把 `can_handle->can_start()` 改成 `this->can_handle->can_start()`。
  //
  // ⚠️ 另一个上游 bug：Can::~Can() 无条件执行
  //        send_thread_->join();  recv_thread_->join();
  //    而这两个线程只在 can_start() 里才创建。dry_run 下从不调用 can_start()，
  //    于是析构时空指针解引用 → 进程退出时段错误（exit code 139）。
  //    这里在 dry_run 下**故意不释放**该对象（进程退出前一直持有），
  //    以绕开该 bug；正式运行时 Can 会正常析构。
  //
  // TODO 上游修复：析构里加判空
  //        if (send_thread_ && send_thread_->joinable()) send_thread_->join();
  //        if (recv_thread_ && recv_thread_->joinable()) recv_thread_->join();
  std::shared_ptr<Can> can_handle;
  if (dry_run) {
    // 用"空删除器"让这个对象**永不析构**（注意：static 局部变量在进程退出时
    // 仍会析构，所以必须用空删除器，不能靠 static 延长寿命）。
    static Can *leaked = new Can(can_interface);
    can_handle = std::shared_ptr<Can>(leaked, [](Can *) { /* 故意不释放 */ });
  } else {
    can_handle = std::make_shared<Can>(can_interface);
  }

  auto node = std::make_shared<OmniChassis>("rb_chassis", odom_topic, cmd_vel_topic,
                                            static_cast<uint32_t>(board_id), can_handle);

  node->setDryRun(dry_run);
  node->setCommandTimeout(std::chrono::milliseconds(timeout_ms));
  node->setAngularZSign(angular_z_sign);
  node->setParameter(limit_vel, limit_acc, width, length, wheel_radius, ratio);

  auto wheel_pub = node->create_publisher<std_msgs::msg::Int16MultiArray>("/chassis/wheel_speed", 10);

  if (!dry_run) {
    can_handle->can_start();
    RCLCPP_INFO(node->get_logger(), "CAN 已启动: %s", can_interface.c_str());
  }
  node->initialize();

  rclcpp::Rate rate(1000.0 / static_cast<double>(period_ms));
  while (rclcpp::ok()) {
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
  return 0;
}
