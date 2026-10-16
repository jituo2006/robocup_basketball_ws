#ifndef RB_OMNI_CHASSIS_H
#define RB_OMNI_CHASSIS_H

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include "board_link.h"

#include "base_chassis.h"
#include "bupt_can/bupt_can.h"
#include "motor_lib/dji_board.h"

/**
 * 全向轮底盘（4 轮 X 型布置，驱动：DJI RoboMaster M3508 经大疆驱动板）。
 *
 * 运动学公式沿用 ROBOCON 2025 篮球工程的实现，未改动物理模型，
 * 保留限速、加速度限制和日志控制（见 base_chassis.h 与 execute()）。
 */
class OmniChassis : public Chassis {
private:
  std::shared_ptr<DJIBoard> dji_board;
  // 存下来只为日志/排查用。之前 initialize() 里把 board_id 写死成 0u 打印，
  // 害得排查时误以为"配置是 1 但实际用了 0"，白白去改了能用的配置。
  uint32_t board_id_ = 0;
  double angular_z_sign_ = 1.0;   // 旋转方向符号（物理装轮方式决定，默认 +1）

  // 停车（显式零速 / 急停）时的减速度上限 m/s²。
  // ⚠️ 必须有这一项：停车若走平时的 limit_acc 会太慢（不安全），
  //    若直接把速度阶跃到零则会让电机速度环过冲甚至反转 ——
  //    现场表现就是"急停时猛往后窜一下"。取"够快但不阶跃"的中间值。
  double stop_accel_ = 3.0;
  std::array<int16_t, 4> wheel_speed{};
  struct StatusMailbox {
    std::mutex mutex;
    std::optional<can_frame> frame;
  };
  std::shared_ptr<StatusMailbox> board_mailbox_ = std::make_shared<StatusMailbox>();
  std::shared_ptr<Can> board_can_;
  std::unique_ptr<rb_chassis::BoardLink> board_link_;
  bool board_was_ready_ = false;
  uint64_t observed_transport_generation_ = 0;
  rb_chassis::MotionGate motion_gate_;


public:
  OmniChassis(const std::string &node_name, const std::string &odom_topic, const std::string &vel_topic,
              uint32_t board_id, const std::shared_ptr<Can> &can_handle);
  ~OmniChassis() override;

  void initialize() override;
  void execute() override;
  bool boardReady() const {
    return dry_run_ || (board_was_ready_ &&
        observed_transport_generation_ == board_can_->transport_generation());
  }
  bool boardReplyRecent(uint64_t now) const { return board_link_->responseRecent(now); }
  const char *boardStatusReason(uint64_t now) const {
    if (dry_run_) return "底盘处于模拟模式，不能执行实车回合";
    if (observed_transport_generation_ != board_can_->transport_generation())
      return "CAN通信异常，等待重新建立底盘握手";
    return board_link_->diagnosticReason(now);
  }

  void setParameter(double limit_vel, double limit_acc, double width, double length,
                    double wheel_radius, double ratio);

  /// 旋转方向符号。若实车自转方向与指令相反，把它设成 -1（不用改代码重编译）。
  void setAngularZSign(double sign) { angular_z_sign_ = (sign < 0 ? -1.0 : 1.0); }
  void setStopAccel(double a) { if (a > 0.0) stop_accel_ = a; }
  double stopAccel() const { return stop_accel_; }
  double angularZSign() const { return angular_z_sign_; }


  /// 立即下发零轮速
  void sendZeroVelocity();

  /// 最近一次解算出的 4 个轮速（rpm），调试用
  const std::array<int16_t, 4> &wheelSpeed() const { return wheel_speed; }
};

#endif  // RB_OMNI_CHASSIS_H
