#include "chassis_lib/dji_protocol.h"
#include <cstdlib>
#include <iostream>
#include <limits>

void check(const std::array<uint8_t, 8> &actual,
           const std::array<uint8_t, 8> &expected) {
  if (actual != expected) {
    std::cerr << "DJI v2 wire packet mismatch\n";
    std::exit(1);
  }
}

int main() {
  for (int board : {-1, 0, 4, std::numeric_limits<int>::max()}) {
    if (rb_dji::validBoardId(board)) return 1;
  }
  for (int board : {1, 2, 3}) {
    if (!rb_dji::validBoardId(board)) return 1;
  }
  for (uint16_t motor = 1; motor <= 4; ++motor) {
    for (uint16_t op : {rb_dji::kMotorOn, rb_dji::kMotorOff,
                       rb_dji::kVelocityConfig, rb_dji::kReadInfo})
      check(rb_dji::command(motor, op),
            {static_cast<uint8_t>(motor), 0, static_cast<uint8_t>(op), 0, 0, 0, 0, 0});
  }
  check(rb_dji::command(4, rb_dji::kVelocityControl, -16),
        {4, 0, 7, 0, 0xf0, 0xff, 0xff, 0xff});
  check(rb_dji::command(1, rb_dji::kVelocityControl, 500),
        {1, 0, 7, 0, 0xf4, 1, 0, 0});
  check(rb_dji::command(2, rb_dji::kVelocityControl), {2, 0, 7, 0, 0, 0, 0, 0});
  check(rb_dji::command(1, rb_dji::kVelocityControl, std::numeric_limits<int32_t>::min()),
        {1, 0, 7, 0, 0, 0, 0, 0x80});
  std::cout << "PASS: DJI v2 commands, zero reserved bytes, signed little endian\n";
}
