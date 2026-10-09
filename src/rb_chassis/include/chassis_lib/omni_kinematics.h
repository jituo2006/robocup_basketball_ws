#ifndef RB_OMNI_KINEMATICS_H
#define RB_OMNI_KINEMATICS_H

#include <array>
#include <cmath>

// All input velocities are in the body frame. No odometry/yaw conversion belongs here.
inline std::array<double, 4> omniWheelRpm(double vx, double vy, double wz,
                                        double width, double length,
                                        double wheel_radius, double ratio) {
  const double pi = std::acos(-1.0);
  const double factor = 60.0 / (2.0 * pi * wheel_radius) * ratio;
  const double c = std::sqrt(0.5);
  const double arm = std::hypot(width / 2.0, length / 2.0);
  return {(+vx * c - vy * c - wz * arm) * factor,
          (+vx * c + vy * c - wz * arm) * factor,
          (-vx * c + vy * c - wz * arm) * factor,
          (-vx * c - vy * c - wz * arm) * factor};
}

#endif  // RB_OMNI_KINEMATICS_H
