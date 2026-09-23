#include "serial_port/serial_port.hpp"

SerialPort::SerialPort(boost::asio::io_context &ioc, const std::string &port, unsigned int baud)
    : ioc_(ioc), serial_(ioc, port) {
  serial_.set_option(serial_port::baud_rate(baud));
  serial_.set_option(serial_port::flow_control(serial_port::flow_control::none));
  serial_.set_option(serial_port::parity(serial_port::parity::none));
  serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  serial_.set_option(serial_port::character_size(8));
}
