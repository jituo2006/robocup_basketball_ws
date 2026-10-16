#pragma once
#include <bupt_can/bupt_can.h>
class DJIBoard {
 public:
  DJIBoard(uint32_t board, const std::shared_ptr<Can>& can):board_(board),can_(can) {}
  void VelCtrl(unsigned motor,int32_t value) { send(motor,7,value); }
  void MotorOff(unsigned motor) { send(motor,2,0); }
 private:
  uint32_t board_;
  std::shared_ptr<Can> can_;
  void send(unsigned motor,uint8_t op,int32_t value) {
    can_frame f{};f.can_id=0x200+board_;f.can_dlc=8;f.data[0]=motor;f.data[2]=op;
    for(unsigned i=0;i<4;++i)f.data[4+i]=(uint32_t(value)>>(8*i))&255;
    can_->send_can_with_respond(f);
  }
};
