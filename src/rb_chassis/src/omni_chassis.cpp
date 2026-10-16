#include "chassis_lib/omni_chassis.h"
#include "chassis_lib/omni_kinematics.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <random>
#include <stdexcept>

#include "utils/utils.h"

OmniChassis::OmniChassis(const std::string &node_name, const std::string &odom_topic,
                         const std::string &vel_topic, uint32_t board_id,
                         const std::shared_ptr<Can> &can_handle)
    : Chassis(node_name, odom_topic, vel_topic), board_id_(board_id) {
  dji_board = std::make_shared<DJIBoard>(board_id, can_handle);
  if (board_id < 1 || board_id > 3) throw std::invalid_argument("supported chassis board_id: 1..3");
  board_can_ = can_handle;
  board_link_ = std::make_unique<rb_chassis::BoardLink>(board_id, std::random_device{}());
  auto mailbox = board_mailbox_;
  can_handle->register_msg(0x280 + board_id, Can::CAN_ID_STD,
      [mailbox](const std::shared_ptr<can_frame> &frame) {
        std::lock_guard<std::mutex> lock(mailbox->mutex);
        mailbox->frame = *frame;
      });


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
  RCLCPP_WARN(get_logger(),
      "等待板端握手和四路电调反馈；配套新固件确认前不发送运动速度（board_id=%u）",
      board_id_);

}

void OmniChassis::sendZeroVelocity() {
  // 零轮速也要同步斜率限制状态，避免后续沿用停车前速度。
  last_sent_vel_ = geometry_msgs::msg::Twist{};
  last_execute_time_ = std::chrono::steady_clock::now();
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
  if (!dry_run_) {
    const auto now = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    std::optional<can_frame> reply;
    {
      std::lock_guard<std::mutex> lock(board_mailbox_->mutex);
      reply.swap(board_mailbox_->frame);
    }
    if (reply) board_link_->accept(*reply, now);
    if (auto request = board_link_->next(now)) {
      if (!board_can_->send_can_with_respond(*request)) board_link_->failed();
    }
    const bool ready = board_link_->ready(now);
    if (motion_gate_.setReady(ready)) {
      sendZeroVelocity();
      target_vel = geometry_msgs::msg::Twist{};
      last_velocity_command_time_ = {};
      RCLCPP_WARN(get_logger(), "%s", ready
          ? "板端已确认四路反馈、速度模式及使能；等待新的零速指令后允许运动"
          : "板端握手失效，已清除运动目标，重新等待就绪");
    }
    board_was_ready_ = ready;
    if (!ready) {
      target_vel = geometry_msgs::msg::Twist{};
      last_sent_vel_ = geometry_msgs::msg::Twist{};
      last_velocity_command_time_ = {};
      wheel_speed.fill(0);
      return;
    }
    const bool zero = std::abs(target_vel.linear.x) < 1e-6 &&
        std::abs(target_vel.linear.y) < 1e-6 && std::abs(target_vel.angular.z) < 1e-6;
    if (!motion_gate_.allow(hasVelocityCommand(), zero)) {
      sendZeroVelocity();
      return;
    }
  }
  // User-selected policy: retain the latest command until another command,
  // explicit stop, a reported board reset, or a concrete I/O fault replaces it.
  // A deliberate zero command still uses the configured stopping deceleration.
  const bool stopping = std::abs(target_vel.linear.x) < 1e-6 &&
      std::abs(target_vel.linear.y) < 1e-6 && std::abs(target_vel.angular.z) < 1e-6;
  const double saved_acc = limit_acc;
  if (stopping) limit_acc = stop_accel_;
  const geometry_msgs::msg::Twist cmd = applyLimits();
  limit_acc = saved_acc;

  // ---------- ③ 全向运动学 ----------
  //
  // /cmd_vel 统一为车体系：+x 前进、+y 左移、+wz 逆时针。
  // 导航已经完成 field→body 转换；遥控/视觉避障也是车体系。
  // 这里不能再次按里程计 yaw 旋转，否则非零朝向时会重复转换。
  // （两人独立发现同一 BUG：任务侧 goto_command 已做 R(−yaw)，底盘再转一次
  //   会双重旋转 —— 车朝左时想往前走却倒退；yaw=0 时恰好抵消。）
  const double vx = cmd.linear.x;
  const double vy = cmd.linear.y;
  // 旋转方向符号：
  //   原 2025 代码这里是 `-cmd.angular.z`。实车测试发现顺逆反了（正 angular.z 却顺时针转），
  //   说明这辆车的轮子安装朝向与公式默认相反，故把默认符号从 -1 改为 +1，
  //   并做成可配置（angular_z_sign），下次装轮方向变了不用改代码。
  const double wz = cmd.angular.z * angular_z_sign_;

  // ⚠️ ratio（减速比）在原实现里被 setParameter 设置但**从未参与计算**。
  //    这里保留为显式乘子，默认必须填 1.0 才能与原行为一致。
  //    只有在确认电机侧还有一级减速时才改为真实比值。
  const auto rpm = omniWheelRpm(vx, vy, wz, width, length, wheel_radius, ratio);
  for (unsigned i = 0; i < wheel_speed.size(); ++i) {
    wheel_speed[i] = static_cast<int16_t>(rpm[i]);
  }

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
