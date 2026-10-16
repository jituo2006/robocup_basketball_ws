// Exercise the production Can implementation; only OS calls are wrapped.
#include "bupt_can/bupt_can.h"
#include "chassis_lib/board_link.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/error.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <iostream>
#include <vector>

void check(bool value) { if (!value) std::abort(); }
namespace {
std::atomic<int> next_fd{100}, sends{0}, closes{0}, tx_error{0}, writes{0};
std::atomic<bool> short_send{false};
std::mutex input_mutex;
std::condition_variable input_cv;
std::deque<can_frame> incoming;
std::vector<can_frame> accepted;
std::atomic<int> callbacks{0};
std::atomic<bool> saw_extended{false};
void inject(uint32_t id) {
    can_frame frame{}; frame.can_id=id;frame.can_dlc=8;
    { std::lock_guard<std::mutex> lock(input_mutex);incoming.push_back(frame); }
    input_cv.notify_all();
}
template<class F> void until(F predicate) {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (!predicate()) {
        check(std::chrono::steady_clock::now()<deadline);
        std::this_thread::yield();
    }
}
}
extern "C" {
int __wrap_socket(int family,int type,int protocol) {
    check(family==AF_CAN && type==SOCK_RAW && protocol==CAN_RAW);return next_fd++;
}
int __wrap_bind(int,const sockaddr*,socklen_t) { return 0; }
int __wrap_ioctl(int,unsigned long request,...) {
    check(request==SIOCGIFINDEX);va_list args;va_start(args,request);
    auto p=va_arg(args,ifreq*);va_end(args);p->ifr_ifindex=3;return 0;
}
int __wrap_setsockopt(int,int level,int option,const void *value,socklen_t size) {
    if(level==SOL_SOCKET && option==SO_SNDBUF)check(*static_cast<const int*>(value)<=4096);
    if(level==SOL_CAN_RAW && option==CAN_RAW_ERR_FILTER)
        check((*static_cast<const can_err_mask_t*>(value)&(CAN_ERR_BUSOFF|CAN_ERR_RESTARTED))!=0);
    if(level==SOL_CAN_RAW && option==CAN_RAW_FILTER)check(size<=512*sizeof(can_filter));
    return 0;
}
ssize_t __wrap_send(int,const void *data,size_t size,int flags) {
    check(size==sizeof(can_frame));check(flags&MSG_DONTWAIT);check(flags&MSG_NOSIGNAL);++sends;
    if (tx_error) { errno=tx_error;return -1; }
    if (short_send)return size-1;
    accepted.push_back(*static_cast<const can_frame*>(data));return size;
}
ssize_t __wrap_write(int,const void*,size_t) { ++writes;std::abort(); }
int __wrap_close(int) { ++closes;return 0; }
int __wrap_poll(pollfd *p,nfds_t n,int timeout) {
    check(n==1 && timeout<=100);
    std::unique_lock<std::mutex> lock(input_mutex);
    input_cv.wait_for(lock,std::chrono::milliseconds(2),[]{return !incoming.empty();});
    if(incoming.empty())return 0;
    p->revents=POLLIN;return 1;
}
ssize_t __wrap_recv(int,void *data,size_t size,int flags) {
    check(flags&MSG_DONTWAIT);check(size==sizeof(can_frame));
    std::lock_guard<std::mutex> lock(input_mutex);
    if(incoming.empty()){errno=EAGAIN;return -1;}
    std::memcpy(data,&incoming.front(),size);incoming.pop_front();return size;
}
}
int main() {
    {
        Can bus("can0");can_frame f{};f.can_id=0x201;f.can_dlc=8;
        check(bus.send_can_with_respond(f));check(bus.transport_generation()==0);
        for(int error:{EAGAIN,ENOBUFS,ENETDOWN,ENODEV,EIO,EINTR}) {
            auto before=bus.transport_generation();tx_error=error;
            check(!bus.send_can_with_respond(f));check(errno==error);
            check(bus.transport_generation()==before+1);
        }
        tx_error=ENOBUFS;
        for(int i=0;i<20000;++i)bus.send_can(f);
        check(accepted.size()==1 && sends==20007); // No queue, retries or replay.
        tx_error=0;short_send=true;
        check(!bus.send_can_with_respond(f));check(errno==EIO);short_send=false;
        f.data[0]=99;check(bus.send_can_with_respond(f));
        check(accepted.size()==2 && accepted.back().data[0]==99 && writes==0);
        check(bus.send_can_with_respond(0x321,Can::CAN_ID_EXT,8,{}));
        check(accepted.back().can_id==(0x321|CAN_EFF_FLAG));
        auto count=sends.load();check(!bus.send_can_with_respond(1,Can::CAN_ID_STD,-1,{}));
        check(!bus.send_can_with_respond(1,Can::CAN_ID_STD,9,{}));check(sends==count);
    }
    check(closes==2); // Destruction without can_start has no on-bus sentinel.
    {
        Can bus("can0");
        bus.register_msg(0x281,Can::CAN_ID_STD,[](const auto &f){
            if(f->can_id&CAN_EFF_FLAG) saw_extended=true;
            ++callbacks;
        });
        bus.can_start();
        inject(0x281|CAN_RTR_FLAG);inject(0x281|CAN_EFF_FLAG);inject(0x281);
        until([]{return callbacks==2;});check(saw_extended);
        auto before=bus.transport_generation();inject(CAN_ERR_FLAG|CAN_ERR_BUSOFF);
        until([&]{return bus.transport_generation()>before;});
        before=bus.transport_generation();inject(CAN_ERR_FLAG|CAN_ERR_RESTARTED);
        until([&]{return bus.transport_generation()>before;});
        inject(0x281);until([]{return callbacks==3;}); // RX still works after bus-off/restart.
    }
    check(closes==4 && writes==0);
    std::cout<<"PASS: production Can, 20000 failed submissions, no replay, error generations, RX recovery, bounded shutdown\n";
}
