#pragma once
#include <linux/can.h>
#include <functional>
#include <memory>
#include <vector>
class Can {
 public:
  enum CAN_ID_TYPE {CAN_ID_STD,CAN_ID_EXT,CAN_ID_ERR,CAN_ID_RTR};
  std::vector<can_frame> sent;
  uint64_t generation=0;
  bool fail_send=false;
  uint64_t transport_generation() const { return generation; }
  std::function<void(const std::shared_ptr<can_frame>&)> callback;
  void register_msg(uint32_t,CAN_ID_TYPE,decltype(callback) cb) { callback=cb; }
  bool send_can_with_respond(const can_frame &f) { if(fail_send){++generation;return false;}sent.push_back(f);return true; }
};
