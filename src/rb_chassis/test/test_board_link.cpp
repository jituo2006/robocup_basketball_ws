#include "chassis_lib/board_link.h"
#include <cstdlib>
#include <iostream>
#include <string>
using rb_chassis::BoardLink;
void check(bool b) { if(!b) std::abort(); }
can_frame reply(const can_frame &request, uint8_t magic=0xB1,
                uint8_t mode=15,uint8_t on=15,uint8_t feedback=15) {
  can_frame f{}; f.can_id=0x281; f.can_dlc=8;
  f.data[0]=magic;f.data[1]=mode;f.data[2]=on;f.data[3]=feedback;
  for(int i=4;i<8;++i)f.data[i]=request.data[i];
  return f;
}
int main() {
  rb_chassis::MotionGate gate;
  check(!gate.allow(true,true));
  check(gate.setReady(true));
  check(!gate.allow(true,false)); // held task command must not restart after power-up
  check(!gate.allow(false,true)); // old zero is not an acknowledgement
  check(gate.allow(true,true));
  check(gate.allow(true,false));
  check(gate.setReady(false));check(!gate.allow(true,false));
  check(gate.setReady(true));check(!gate.allow(true,false));
  check(gate.allow(true,true));

  BoardLink l(1,123);
  check(!l.ready(0));
  check(!l.responseRecent(0));
  check(std::string(l.diagnosticReason(0)).find("未收到") != std::string::npos);
  auto q=l.next(0); check(q && q->data[2]==0x70);
  check(!l.next(50));
  auto bad=reply(*q);bad.data[4]^=1;check(!l.accept(bad,1));check(!l.ready(1));
  bad=reply(*q);bad.can_id=0x282;check(!l.accept(bad,1));
  bad=reply(*q);bad.can_dlc=7;check(!l.accept(bad,1));
  check(l.accept(reply(*q),2));check(!l.ready(2));
  auto arm=l.next(3);check(arm && arm->data[2]==0x71);
  check(l.accept(reply(*arm,0xB2),4));check(l.ready(4));
  check(l.responseRecent(4));
  check(!l.responseRecent(1005));
  check(l.ready(1005)); // Admission freshness does NOT expire motion/handshake readiness.
  check(std::string(l.diagnosticReason(1005)).find("应答中断") != std::string::npos);
  check(!l.accept(reply(*arm,0xB2),5)); // duplicate replies are ignored
  check(l.ready(355)); // silence alone no longer stops the chassis
  q=l.next(355);check(q && q->data[2]==0x70);
  check(!l.accept(reply(*arm,0xB2),356)); // previous session challenge
  check(l.accept(reply(*q,0xB1),356));check(!l.ready(356)); // board reset
  arm=l.next(357);check(arm && arm->data[2]==0x71);
  check(l.accept(reply(*arm,0xB2),358));check(l.ready(358));
  l.failed();check(!l.ready(359));
  check(!l.responseRecent(359));
  BoardLink missing(1,8);
  q=missing.next(0);check(missing.accept(reply(*q,0xB1,15,15,7),1));
  check(std::string(missing.diagnosticReason(1)).find("反馈不齐") != std::string::npos);
  check(!missing.next(2));check(!missing.ready(2)); // no configuration without four ESCs
  q=missing.next(100);check(missing.accept(reply(*q,0xB1,14,15,15),101));
  check(std::string(missing.diagnosticReason(101)).find("速度模式") != std::string::npos);
  auto setup=missing.next(102);check(setup && setup->data[0]==1 && setup->data[2]==3);
  q=missing.next(202);check(missing.accept(reply(*q,0xB1,15,13,15),203));
  check(std::string(missing.diagnosticReason(203)).find("使能状态") != std::string::npos);
  setup=missing.next(204);check(setup && setup->data[0]==2 && setup->data[2]==1);
  q=missing.next(304);check(missing.accept(reply(*q),555)); // delayed matching replies do not expire
  BoardLink cycling(1,800);
  for(uint64_t cycle=0;cycle<500;++cycle) {
    const auto now=cycle*1000;
    auto query=cycling.next(now);check(query && query->data[2]==0x70);
    auto status=reply(*query);
    // Extended or remote frames can never masquerade as board status.
    status.can_id|=CAN_EFF_FLAG;check(!cycling.accept(status,now));
    status=reply(*query);status.can_id|=CAN_RTR_FLAG;check(!cycling.accept(status,now));
    check(cycling.accept(reply(*query),now));
    auto enable=cycling.next(now+1);check(enable && enable->data[2]==0x71);
    auto previous_ack=reply(*enable,0xB2);
    check(cycling.accept(previous_ack,now+2));check(cycling.ready(now+2));
    cycling.failed();check(!cycling.ready(now+3));
    query=cycling.next(now+3);check(query && query->data[2]==0x70);
    check(!cycling.accept(previous_ack,now+4)); // Old reply cannot re-arm after an I/O fault.
    cycling.failed();
  }
  std::cout << "PASS: delayed power, nonce, duplicate, no age expiry, reset, 500 power cycles, feedback/mode/enable gates\n";
}
