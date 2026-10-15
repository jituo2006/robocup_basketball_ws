#include "serial_port/serial_port_R1.hpp"

#include <cstring>
#include <cerrno>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdexcept>
#include <algorithm>

void SerialPortR1::put_packet(Message::FrameControl cmd, uint16_t speed, uint16_t angle) {
  packet_.cmd = static_cast<uint8_t>(cmd);

  buff_[0] = packet_.head;
  buff_[1] = packet_.cmd;
  std::memcpy(buff_.data() + 2, &speed, sizeof(packet_.speed));
  std::memcpy(buff_.data() + 4, &angle, sizeof(packet_.angle));
  crc32_.process_bytes(buff_.data(), 6);
  packet_.crc32 = crc32_.checksum();
  std::memcpy(buff_.data() + 6, &packet_.crc32, sizeof(packet_.crc32));
  buff_[10] = packet_.tail;
  crc32_.reset();
}

bool SerialPortR1::send_command() {
  try {
    boost::system::error_code write_ec;
    size_t bytes_written = boost::asio::write(serial_, boost::asio::buffer(buff_), write_ec);

    if (write_ec) {
      std::cerr << "Failed to write data: " << write_ec.message() << std::endl;
      return false;
    }
    if (bytes_written != buff_.size()) {
      std::cerr << "Warning: Only wrote " << bytes_written << " of " << buff_.size() << " bytes" << std::endl;
      return false;
    }

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Serial communication error: " << e.what() << std::endl;
    return false;
  } catch (...) {
    std::cerr << "Unknown serial communication error occurred" << std::endl;
    return false;
  }
}

bool SerialPortR1::connected() {
  if (!serial_.is_open()) return false;
  // USB 拔出时文件描述符还在，但内核会报告挂断；is_open() 无法识别这一点。
  pollfd descriptor{serial_.native_handle(), 0, 0};
  const int ready = ::poll(&descriptor, 1, 0);
  if (ready < 0) return errno == EINTR;
  return !(descriptor.revents & (POLLERR | POLLHUP | POLLNVAL));
}

bool SerialPortR1::send_string(const std::string& data) {
  try {
    boost::system::error_code ec;
    size_t bytes_written = boost::asio::write(serial_, boost::asio::buffer(data), ec);

    if (ec) {
      std::cerr << "Write error: " << ec.message() << std::endl;
      return false;
    }
    if (bytes_written != data.size()) {
      std::cerr << "Incomplete text write: " << bytes_written << "/" << data.size() << std::endl;
      return false;
    }
    std::cout << "Sent " << bytes_written << " bytes: " << data << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return false;
  } catch (...) {
    std::cerr << "Unknown text serial communication error" << std::endl;
    return false;
  }
}

// Only this node reads the serial FD. FIONREAD prevents blocking the ROS executor.
std::string SerialPortR1::read_available() {
  int count = 0;
  if (::ioctl(serial_.native_handle(), FIONREAD, &count) < 0)
    throw std::runtime_error("serial FIONREAD failed");
  if (count <= 0) return {};
  std::string bytes(static_cast<size_t>(std::min(count, 16384)), '\0');
  const ssize_t n = ::read(serial_.native_handle(), bytes.data(), bytes.size());
  if (n < 0) {
    if (errno == EINTR || errno == EAGAIN) return {};
    throw std::runtime_error("serial read failed");
  }
  if (n == 0) throw std::runtime_error("serial EOF");
  bytes.resize(static_cast<size_t>(n));
  return bytes;
}
