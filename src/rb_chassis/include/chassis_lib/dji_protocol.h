#ifndef RB_DJI_PROTOCOL_H
#define RB_DJI_PROTOCOL_H

#include <array>
#include <cstdint>

namespace rb_dji {
// DJI v2 commands from the field project's dji_boardv2_can.h.
constexpr uint16_t kMotorOn = 1;
constexpr uint16_t kMotorOff = 2;
constexpr uint16_t kVelocityConfig = 3;
constexpr uint16_t kVelocityControl = 7;
constexpr uint16_t kReadInfo = 11;

inline std::array<uint8_t, 8> command(uint16_t motor, uint16_t opcode,
                                      int32_t value = 0) {
  const uint32_t bits = static_cast<uint32_t>(value);
  return {static_cast<uint8_t>(motor), static_cast<uint8_t>(motor >> 8),
          static_cast<uint8_t>(opcode), static_cast<uint8_t>(opcode >> 8),
          static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8),
          static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 24)};
}
}  // namespace rb_dji

#endif
