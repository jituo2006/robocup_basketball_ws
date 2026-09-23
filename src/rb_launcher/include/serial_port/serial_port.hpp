#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <boost/asio.hpp>
#include <iostream>
#include <string>

using boost::asio::io_context;
using boost::asio::serial_port;

/**
 * 串口基类（移植自 ROBOCON 2025 ballrobot）。
 *
 * 唯一改动：波特率改为构造参数（原实现硬编码 115200），默认值仍是 115200，
 * 因此不改变原有行为。
 */
class SerialPort {
protected:
  io_context &ioc_;
  serial_port serial_;

public:
  SerialPort(io_context &ioc, const std::string &port, unsigned int baud = 115200);
  virtual ~SerialPort() = default;

  /* 约定：返回 true 表示这一帧已经发出、可以换下一条命令；
     返回 false 表示发送失败，调用方应保持原命令并重发。 */
  virtual bool send_command() = 0;
};

#endif /* SERIAL_PORT_H */
