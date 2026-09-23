#ifndef RB_OMNI_CHASSIS_H
#define RB_OMNI_CHASSIS_H

#include <array>
#include <cstdint>
#include <memory>

#include "base_chassis.h"
#include "bupt_can/bupt_can.h"
#include "motor_lib/dji_board.h"

/**
 * 全向轮底盘（4 轮 X 型布置，驱动：DJI RoboMaster M3508 经大疆驱动板）。
 *
 * 运动学公式沿用 ROBOCON 2025 篮球工程的实现，未改动物理模型，
 * 只修复了限速/超时/日志三类工程缺陷（见 base_chassis.h 与 execute()）。
 */
class OmniChassis : public Chassis {
private:
  std::shared_ptr<DJIBoard> dji_board;
  double angular_z_sign_ = 1.0;   // 旋转方向符号（物理装轮方式决定，默认 +1）
  std::array<int16_t, 4> wheel_speed{};
  bool timeout_zero_sent_ = false;

public:
  OmniChassis(const std::string &node_name, const std::string &odom_topic, const std::string &vel_topic,
              uint32_t board_id, const std::shared_ptr<Can> &can_handle);
  ~OmniChassis() override;

  void initialize() override;
  void execute() override;

  void setParameter(double limit_vel, double limit_acc, double width, double length,
                    double wheel_radius, double ratio);

  /// 旋转方向符号。若实车自转方向与指令相反，把它设成 -1（不用改代码重编译）。
  void setAngularZSign(double sign) { angular_z_sign_ = (sign < 0 ? -1.0 : 1.0); }
  double angularZSign() const { return angular_z_sign_; }

  /// 立即下发零轮速
  void sendZeroVelocity();

  /// 最近一次解算出的 4 个轮速（rpm），调试用
  const std::array<int16_t, 4> &wheelSpeed() const { return wheel_speed; }
};

#endif  // RB_OMNI_CHASSIS_H
