#ifndef SERIAL_PORT_DRIBBLE_H
#define SERIAL_PORT_DRIBBLE_H

#include <array>
#include <cstdint>
#include <string>

#include <boost/crc.hpp>

#include "serial_port.hpp"
#include "utils/message.hpp"

using namespace Message;

/**
 * R1 发射机构串口（移植自 ROBOCON 2025 ballrobot，帧协议未改动）。
 *
 * 帧格式（11 字节）：
 *   `+` | cmd(1) | speed(2, 小端) | angle(2, 小端) | crc32(4, 覆盖前 6 字节) | `*`
 *
 * 唯一的改动：波特率可配置（原来硬编码 115200），默认值不变。
 */
class SerialPortR1 : public SerialPort {
private:
  MessagePacket packet_;
  boost::crc_32_type crc32_;
  std::array<uint8_t, 11> buff_{};

public:
  SerialPortR1(io_context& ioc, const std::string& port, unsigned int baud = 115200)
      : SerialPort(ioc, port, baud) {}

  void put_packet(Message::FrameControl cmd, uint16_t speed, uint16_t angle);
  bool send_command() override;
  void send_string(const std::string& data);
};

#endif /* SERIAL_PORT_DRIBBLE_H */
