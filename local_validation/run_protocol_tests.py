"""Compile production protocol functions with fake hardware, entirely locally."""
from pathlib import Path
import subprocess
import sys

tests = Path(__file__).resolve().parent
firmware = Path(sys.argv[2]) if len(sys.argv)>2 else Path('E:/Dji/dji_driver_board-feature-cxx')
host = Path(sys.argv[1]) if len(sys.argv)>1 else tests.parent
source = (firmware / 'motor_cxx/message.cpp').read_text(encoding='utf-8')
protocol = source[source.index('static uint8_t feedback_seen'):source.index('void  Receive_Choose(')]
prefix = r'''
#include "chassis_lib/board_link.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
union CAN_Message { uint8_t ui8[8]; };
enum { VEL_Mode=1, CAN_ID_STD=0 };
int hcan2=0, BOARDID=1;
int modes[4]={1,1,1,1}, enabled[4]={1,1,1,1};
float targets[4]={10,10,10,10};
can_frame captured{};
int get_MotorCtrlMode(unsigned i) { return modes[i]; }
int If_used(unsigned i) { return enabled[i]; }
void write_MotorTarget(unsigned i, float value) { targets[i]=value; }
int OSLIB_CAN_TrySendMessage_Length(int*, int, int id, CAN_Message *m, int length) {
  captured={};captured.can_id=id;captured.can_dlc=length;
  std::memcpy(captured.data,m->ui8,8);return 1;
}
void check(bool b) { if(!b) std::abort(); }
uint32_t nonce(const can_frame &f) {
  uint32_t n=0;for(unsigned i=0;i<4;++i)n|=uint32_t(f.data[4+i])<<(i*8);return n;
}
'''
cases = r'''
int main() {
  rb_chassis::BoardLink link(1,7);
  auto q=link.next(0);check(q && q->data[2]==0x70);
  HostControlQuery(nonce(*q));check(captured.data[3]==0);
  check(link.accept(captured,1));check(!link.ready(1));
  check(!link.next(2));
  feedback_seen=15; // Four motor feedback messages have been observed.
  q=link.next(100);HostControlQuery(nonce(*q));
  check(link.accept(captured,101));
  auto arm=link.next(102);check(arm && arm->data[2]==0x71);
  HostControlArm(nonce(*arm));check(HostControlPermitted());
  check(link.accept(captured,103));check(link.ready(103));
  for(float target:targets)check(target==0); // Arming discards previous targets.
  check(link.ready(10000000));check(HostControlPermitted()); // No silence timeout.
  targets[0]=25;
  HostControlArm(nonce(*arm));check(targets[0]==25); // No repeated PID reset/zero on replay.
  host_armed=false;probe_seen=false; // Simulated control-board reset.
  HostControlArm(nonce(*arm));check(!HostControlPermitted());
  q=link.next(10000000);HostControlQuery(nonce(*q));
  check(link.accept(captured,10000001));check(!link.ready(10000001));
  arm=link.next(10000002);HostControlArm(nonce(*arm));
  check(link.accept(captured,10000003));check(link.ready(10000003));
  check(HostControlPermitted());check(targets[0]==0);
  host_armed=false;HostControlQuery(12345);HostControlArm(54321);
  check(!HostControlPermitted()); // A mismatched challenge never arms.
  std::cout << "PASS: actual firmware/host handshake, late power, no age expiry, reset, nonce and arm replay\n";
}
'''
generated = tests / 'generated_protocol_test.cpp'
generated.write_text(prefix + protocol + cases, encoding='utf-8')
exe = tests / 'test_protocol.exe'
subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-static',
                '-I', str(tests/'include'), '-I', str(host/'src/rb_chassis/include'),
                str(generated), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
