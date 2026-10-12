#pragma once
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

// PTP PHC follows TAI; ROS headers follow UTC. One configuration drives both.
namespace rb_ptp {
struct Config {
  bool enabled;
  uint64_t offset_ns;
};

inline Config ReadConfig(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    struct stat info;
    if (stat(path.c_str(), &info) == 0)
      throw std::runtime_error("Cannot read RoboCup PTP UTC offset: " + path);
    return {false, 0};  // No installation: retain upstream behaviour.
  }
  int seconds = -1;
  std::string extra;
  if (!(file >> seconds) || seconds < 0 || seconds > 100 || (file >> extra))
    throw std::runtime_error("Invalid RoboCup PTP UTC offset: " + path);
  return {true, static_cast<uint64_t>(seconds) * 1000000000ULL};
}

inline const Config& ClockConfig() {
  static const Config config = ReadConfig("/etc/robocup/livox_ptp_utc_offset");
  return config;
}

inline uint64_t ToUtc(uint8_t type, uint64_t stamp, const Config& config) {
  // Never substitute packet arrival time for an unsynchronised measurement.
  if (type != 1 || stamp <= config.offset_ns) return 0;
  return stamp - config.offset_ns;
}
}  // namespace rb_ptp
