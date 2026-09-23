#include "serial_port/serial_port_R1.hpp"

#include <cstring>

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

void SerialPortR1::send_string(const std::string& data) {
  try {
    boost::system::error_code ec;
    size_t bytes_written = boost::asio::write(serial_, boost::asio::buffer(data), ec);

    if (ec) {
      std::cerr << "Write error: " << ec.message() << std::endl;
      return;
    }

    std::cout << "Sent " << bytes_written << " bytes: " << data << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }
}