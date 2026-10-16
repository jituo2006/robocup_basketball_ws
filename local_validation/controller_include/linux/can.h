#pragma once
// Native Windows protocol-test fixture only. Production Linux uses SocketCAN.
#include <cstdint>
using canid_t = std::uint32_t;
struct can_frame {
  canid_t can_id{};
  std::uint8_t can_dlc{}, pad{}, reserved0{}, reserved1{};
  alignas(8) std::uint8_t data[8]{};
};
