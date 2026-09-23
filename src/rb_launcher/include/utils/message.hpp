#ifndef MESSAGE_H
#define MESSAGE_H

#include <cstdint>
#include <string>

enum class JoystickSwitch : int { BIT_JOY_HANDLE, BIT_AIM_MODE };

/*
R2 车串口报文
head: `+`
cmd: 从高到低为 B_7~B_0，目前暂定四个动作，依次为：运球、发射、夹爪开关、校准，
     占用高四位，1 使能，采用优先级仲裁策略，优先级次序同位次
speed: 转速
crc32: 循环冗余校验码
tail: `*`
*/
namespace Message {
void w_bytes32(uint8_t *b, uint32_t v);
void w_bytes16(uint8_t *b, uint16_t v);

enum class FrameControl {
  DRIBBLE = 0x80,
  DOUBLE_DRIBBLE = 0x81,
  SHOOT = 0x40,
  FAST_SHOOT = 0x41,
  KEEP_LOOP = 0x42,
  PAWL = 0x20,
  SLIDEWAY = 0x10,
  RESPONSE = 0x60
};

struct MessagePacket {
  char head;
  uint8_t cmd;
  uint16_t speed;
  uint16_t angle;
  uint32_t crc32;
  char tail;

  MessagePacket() {
    head = '+';
    cmd = 0;
    speed = 0;
    angle = 0;
    crc32 = 0;
    tail = '*';
  }
};

static constexpr char CMD_FAST_SHOOT[] = "launchallfast";
static constexpr char CMD_CATCH[] = "catch";
static constexpr char CMD_SLIDEWAY[] = "slideway";
static constexpr char CMD_DOUBLE_DRIBBLE[] = "dribbledouble";
}  // namespace Message

#endif /* MESSAGE_H */
