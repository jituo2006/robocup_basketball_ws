#include "chassis_lib/omni_chassis.h"
#include <cstdlib>
#include <iostream>
class Controller: public OmniChassis {
 public:
  using OmniChassis::OmniChassis;
  void tick() { last_execute_time_=std::chrono::steady_clock::now()-std::chrono::milliseconds(10);execute(); }
  void age() { last_velocity_command_time_-=std::chrono::hours(24); }
};
void check(bool value) { if(!value)std::abort(); }
void reply(const std::shared_ptr<Can>& can, const can_frame &request,uint8_t kind) {
  auto f=std::make_shared<can_frame>();f->can_id=0x281;f->can_dlc=8;
  f->data[0]=kind;f->data[1]=f->data[2]=f->data[3]=15;
  for(unsigned i=4;i<8;++i)f->data[i]=request.data[i];
  can->callback(f);
}
bool moving(Controller &c) { for(auto wheel:c.wheelSpeed())if(wheel!=0)return true;return false; }
int main() {
  auto can=std::make_shared<Can>();
  Controller c("test","/odom","/cmd_vel",1,can);
  c.initialize();check(can->sent.empty());c.tick();check(can->sent.size()==1);
  auto q=can->sent.back();check(q.data[2]==0x70);check(!moving(c));
  reply(can,q,0xB1);c.tick();auto arm=can->sent.back();check(arm.data[2]==0x71);
  reply(can,arm,0xB2);c.tick();check(!moving(c));
  geometry_msgs::msg::Twist velocity;velocity.linear.x=.4;
  c.setVelocity(velocity);c.tick();check(!moving(c)); // require a new explicit zero after handshake
  c.setVelocity({});c.tick();c.setVelocity(velocity);c.tick();check(moving(c));
  c.age();for(unsigned i=0;i<20;++i)c.tick();check(moving(c)); // 24h-old commands remain active
  c.setVelocity({});for(unsigned i=0;i<20;++i)c.tick();check(!moving(c)); // explicit stop still works
  // Concrete transport failures clear both readiness and the previous target.
  c.setVelocity(velocity);c.tick();check(moving(c));
  can->fail_send=true;++can->generation;c.tick();check(!moving(c));check(!c.boardReady());
  for(unsigned i=0;i<1000;++i) c.tick();
  check(!moving(c));
  can->fail_send=false;c.tick();q=can->sent.back();check(q.data[2]==0x70);
  reply(can,q,0xB1);c.tick();arm=can->sent.back();check(arm.data[2]==0x71);
  reply(can,arm,0xB2);c.tick();check(c.boardReady());check(!moving(c));
  c.setVelocity(velocity);c.tick();check(!moving(c)); // Old route cannot reopen the neutral gate.
  c.setVelocity({});c.tick();c.setVelocity(velocity);c.tick();check(moving(c));
  // A reply queued just before a transport fault cannot arm the next session.
  ++can->generation;c.tick();q=can->sent.back();reply(can,q,0xB2);
  ++can->generation;c.tick();check(!c.boardReady());check(!moving(c));
  {
    auto dry_can=std::make_shared<Can>();
    Controller dry("dry","/odom","/cmd_vel",1,dry_can);
    dry.setDryRun(true);dry.initialize();dry.tick();
    rclcpp::shutdown_hook();check(dry_can->sent.empty());
  }
  std::cout << "PASS: production Chassis/OmniChassis, startup handshake, neutral gate, no command TTL, power-cycle recovery, stale-reply rejection, dry-run shutdown, explicit stop\n";
}
