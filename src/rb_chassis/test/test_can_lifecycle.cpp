// Mock system calls: no physical CAN interface or motor frames are used.
#include "bupt_can/bupt_can.h"
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdlib>
#include <iostream>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <thread>

namespace {
std::atomic<int> opened{0}, closed{0}, sent{0};
bool missing_interface = false, failed_send = false;
void check(bool ok, const char *message) {
  if (!ok) { std::cerr << message << std::endl; std::exit(1); }
}
}
extern "C" {
int __wrap_socket(int, int, int) { return 100 + opened++; }
int __wrap_close(int) { ++closed; return 0; }
int __wrap_ioctl(int, unsigned long request, ...) {
  if (missing_interface) { errno = ENODEV; return -1; }
  va_list args; va_start(args, request);
  auto *interface = va_arg(args, ifreq*); va_end(args);
  if (request == SIOCGIFINDEX) interface->ifr_ifindex = 3;
  if (request == SIOCGIFFLAGS) interface->ifr_flags = IFF_UP;
  return 0;
}
int __wrap_bind(int, const sockaddr*, socklen_t) { return 0; }
int __wrap_setsockopt(int, int, int, const void*, socklen_t) { return 0; }
ssize_t __wrap_send(int, const void*, size_t size, int flags) {
  check(flags & MSG_DONTWAIT, "CAN sends must not block");
  ++sent;
  if (failed_send) { errno = ENOBUFS; return -1; }
  return size;
}
int __wrap_poll(pollfd*, nfds_t, int timeout) {
  check(timeout > 0 && timeout <= 100, "receive must have bounded wait");
  std::this_thread::sleep_for(std::chrono::milliseconds(5)); return 0;
}
}
int main() {
  { Can dry("can0"); dry.send_can(0x201, Can::CAN_ID_STD, 8, {}); }
  check(opened == 0 && sent == 0, "dry lifecycle must never access CAN");
  missing_interface = true;
  bool threw = false;
  { Can unavailable("missing"); try { unavailable.can_start(); } catch (const std::exception&) { threw = true; } }
  check(threw && closed == opened, "missing interface must fail and close sockets");
  missing_interface = false;
  for (bool fail : {false, true}) {
    failed_send = fail;
    const int before = sent;
    const auto begin = std::chrono::steady_clock::now();
    { Can bus("can0"); bus.can_start();
      bus.send_can(0x201, Can::CAN_ID_STD, 8, {});
      if (fail) {
        for (int i = 0; i < 100 && bus.isStarted(); ++i)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(!bus.isStarted(), "write failure must invalidate transport");
        bus.send_can(0x201, Can::CAN_ID_STD, 8, {});
      }
    }
    check(sent == before + 1, "failed frame must not be retried or shutdown frame added");
    check(std::chrono::steady_clock::now() - begin < std::chrono::seconds(1), "shutdown must not await bus echo");
    check(closed == opened, "all CAN sockets must close");
  }
}
