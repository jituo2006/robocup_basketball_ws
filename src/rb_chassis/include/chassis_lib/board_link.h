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
  void failed() {
    armed_=false; action_.reset(); awaiting_=false; next_poll_=0;
    last_response_.reset();
  }
  // Admission diagnostics only: a stale reply never expires motor commands
  // or changes ready(), per the user's motion policy.
  bool responseRecent(uint64_t now) const {
    return last_response_ && now >= *last_response_ && now-*last_response_ <= 1000;
  }
  const char *diagnosticReason(uint64_t now) const {
    if (!last_response_) return "未收到大疆板应答，请检查控制板、电调供电及CAN连接";
    if (!responseRecent(now)) return "大疆板状态应答中断，暂不能开始新回合";
    if (feedback_mask_ != 15) return "四路电调反馈不齐，请检查电调供电和电机CAN连接";
    if (mode_mask_ != 15) return "四路电机的速度模式尚未全部确认";
    if (on_mask_ != 15) return "四路电机的使能状态尚未全部确认";
    if (!armed_) return "等待大疆板完成使能握手确认";
    return "大疆板及四路电调已确认就绪";
  }
  bool accept(const can_frame &f, uint64_t now) {
    if (f.can_id != 0x280+board_ || f.can_dlc != 8 ||
        (f.data[0]!=0xB1 && f.data[0]!=0xB2) || !awaiting_ ||
        unpack(f.data+4)!=expected_) return false;
    awaiting_=false;
    last_response_=now;
    feedback_mask_=f.data[3]; mode_mask_=f.data[1]; on_mask_=f.data[2];
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
  std::optional<uint64_t> last_response_;
  uint8_t feedback_mask_=0, mode_mask_=0, on_mask_=0;
  std::optional<can_frame> action_;
};
} // namespace rb_chassis
