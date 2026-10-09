#include "chassis_lib/omni_kinematics.h"
#include <cmath>
#include <cstdlib>
#include <iostream>

void check(bool condition, const char *description) {
  if (!condition) {
    std::cerr << description << '\n';
    std::exit(1);
  }
}

int main() {
  const double r = 0.0765, span = 0.477059538;
  const auto forward = omniWheelRpm(0.4, 0, 0, span, span, r, 1);
  check(forward[0] > 0 && forward[1] > 0 && forward[2] < 0 && forward[3] < 0,
        "forward wheel directions");
  const auto right = omniWheelRpm(0, -0.4, 0, span, span, r, 1);
  // Navigation at yaw=90 deg maps world +X to body -Y; chassis must preserve it.
  check(right[0] > 0 && right[1] < 0 && right[2] < 0 && right[3] > 0,
        "body -Y must remain a right translation");
  const auto turn = omniWheelRpm(0, 0, 0.5, span, span, r, 1);
  for (double rpm : turn) check(rpm < 0 && std::abs(rpm-turn[0]) < 1e-12, "pure rotation");
  const double distance_per_rev = 2 * std::acos(-1.0) * r;
  check(std::abs(-turn[0] * distance_per_rev / 60.0 / 0.5 - 0.337332034356) < 1e-9,
        "CAD rotation arm");
  const auto combined = omniWheelRpm(0.4, -0.4, 0.5, span, span, r, 1);
  for (unsigned i = 0; i < 4; ++i)
    check(std::abs(combined[i]-forward[i]-right[i]-turn[i]) < 1e-12,
          "translation and rotation superposition");
  const auto stop = omniWheelRpm(0, 0, 0, span, span, r, 1);
  for (double rpm : stop) check(rpm == 0, "zero command");
  std::cout << "PASS: body frame, wheel signs, CAD arm, superposition, stop\n";
}
