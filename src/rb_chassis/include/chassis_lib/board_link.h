#pragma once
#include <linux/can.h>
#include <array>
#include <cstdint>
#include <optional>

namespace rb_chassis {
// Paired firmware protocol: status query 0x70, arm 0x71. Neither commands motion.
class MotionGate {
 public:
  bool setReady(bool value) {
    const bool changed=value!=ready_;
    if (changed) neutral_=false;
    ready_=value;
    return changed;
  }
  bool allow(bool fresh, bool zero) {
    if (!ready_) return false;
    if (fresh && zero) neutral_=true;
    return neutral_;
  }
 private:
  bool ready_=false, neutral_=false;
};
class BoardLink {
 public:
  explicit BoardLink(uint32_t board, uint32_t seed) : board_(board), nonce_(seed) {}
  bool ready(uint64_t now) const {
    (void)now;
    return armed_;  // User policy: silence alone never clears readiness.
  }
  void failed() { armed_=false; action_.reset(); awaiting_=false; }
  bool accept(const can_frame &f, uint64_t now) {
    if (f.can_id != 0x280+board_ || f.can_dlc != 8 ||
        (f.data[0]!=0xB1 && f.data[0]!=0xB2) || !awaiting_ ||
        unpack(f.data+4)!=expected_) return false;
    awaiting_=false;
    (void)now;
    const bool feedback=f.data[3]==15, mode=f.data[1]==15, on=f.data[2]==15;
    armed_=feedback && mode && on && f.data[0]==0xB2;
    if (!feedback) return true;  // Never enable motors before ESC feedback has been observed.
    if (!mode || !on) {
      for (unsigned i=0;i<4;++i) {
        if (!(f.data[1] & (1u<<i))) { action_=command(i+1,3,0); return true; }
      }
      for (unsigned i=0;i<4;++i) {
        if (!(f.data[2] & (1u<<i))) { action_=command(i+1,1,0); return true; }
      }
    } else if (!armed_) {
      action_=command(0,0x71,expected_);
    }
    return true;
  }
  std::optional<can_frame> next(uint64_t now) {
    if (!ready(now)) armed_=false;
    if (action_) {
      can_frame f=*action_; action_.reset();
      next_poll_=now+100;
      if (f.data[2]==0x71) {
        expected_=unpack(f.data+4); awaiting_=true;
      }
      return f;
    }
    if (now < next_poll_) return {};
    // Retry the same challenge until its reply arrives; no response deadline.
    if (!awaiting_) {
      if (++nonce_==0) ++nonce_;
      expected_=nonce_;
    }
    awaiting_=true; next_poll_=now+100;
    return command(0,0x70,expected_);
  }
 private:
  static uint32_t unpack(const uint8_t *p) {
    return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
  }
  can_frame command(uint16_t motor, uint8_t op, uint32_t value) const {
    can_frame f{}; f.can_id=0x200+board_; f.can_dlc=8;
    f.data[0]=motor&255; f.data[1]=motor>>8; f.data[2]=op;
    for (unsigned i=0;i<4;++i) f.data[4+i]=(value>>(i*8))&255;
    return f;
  }
  uint32_t board_, nonce_, expected_=0;
  uint64_t next_poll_=0;
  bool awaiting_=false, armed_=false;
  std::optional<can_frame> action_;
};
} // namespace rb_chassis
