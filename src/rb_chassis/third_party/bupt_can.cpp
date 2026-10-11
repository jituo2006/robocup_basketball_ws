#include "bupt_can/bupt_can.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <system_error>
#include <stdexcept>
#include <chrono>
#include <poll.h>

namespace {
void can_failure(const std::string &operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}
}

Can::Can(const std::string &name)
{
    can_name = name;
    can_fd_read = can_fd_write = -1;
    ifr = {};
    addr = {};
    isDestroyed = false;
    Started = false;
    filter_size = 1;
    filters[0].can_id = 0x7FC;
    filters[0].can_mask = CAN_SFF_MASK;
}

Can::~Can()
{
    // 不靠发送 0x7FC 唤醒读线程：总线故障时该帧可能永远发送不出去。
    // 保留类成员布局，与系统静态 motor_lib 的 Can ABI 一致。
    {
        std::lock_guard<std::mutex> lock(send_que_mutex);
        isDestroyed = true;
    }
    send_que_cv.notify_all();
    if (send_thread_ && send_thread_->joinable()) send_thread_->join();
    if (recv_thread_ && recv_thread_->joinable()) recv_thread_->join();
    if (can_fd_read >= 0) close(can_fd_read);
    if (can_fd_write >= 0) close(can_fd_write);
}

void Can::can_start()
{
    if (Started) return;
    if (send_thread_ || recv_thread_ || isDestroyed)
        throw std::runtime_error("CAN transport failed; restart the chassis process");
    if (can_name.empty() || can_name.size() >= IFNAMSIZ)
        throw std::invalid_argument("invalid CAN interface name");
    try {
        can_fd_read = socket(AF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
        if (can_fd_read < 0) can_failure("CAN socket(read)");
        can_fd_write = socket(AF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
        if (can_fd_write < 0) can_failure("CAN socket(write)");
        std::strcpy(ifr.ifr_name, can_name.c_str());
        if (ioctl(can_fd_read, SIOCGIFINDEX, &ifr) < 0)
            can_failure("CAN interface " + can_name);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (ioctl(can_fd_read, SIOCGIFFLAGS, &ifr) < 0)
            can_failure("CAN flags " + can_name);
        if (!(ifr.ifr_flags & IFF_UP))
            throw std::runtime_error("CAN interface is DOWN: " + can_name);
        if (bind(can_fd_read, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            can_failure("CAN bind(read) " + can_name);
        if (bind(can_fd_write, (struct sockaddr*)&addr, sizeof(addr)) < 0)
            can_failure("CAN bind(write) " + can_name);
        if (set_recv_filter() < 0) can_failure("CAN receive filter");
        std::cerr << "CAN 初始化: " << can_name << " ifindex=" << addr.can_ifindex << std::endl;
        recv_thread_ = std::make_unique<std::thread>(std::bind(&Can::receive_thread,this));
        send_thread_ = std::make_unique<std::thread>(std::bind(&Can::send_thread,this));
        Started = true;
    } catch (...) {
        {
            std::lock_guard<std::mutex> lock(send_que_mutex);
            isDestroyed = true;
        }
        send_que_cv.notify_all();
        if (recv_thread_ && recv_thread_->joinable()) recv_thread_->join();
        if (can_fd_read >= 0) close(can_fd_read);
        if (can_fd_write >= 0) close(can_fd_write);
        can_fd_read = can_fd_write = -1;
        throw;
    }
}

uint32_t Can::set_id_type(const CAN_ID_TYPE &id_type, const uint32_t &id)
{
    uint32_t ret = id;
    if (id_type == CAN_ID_TYPE::CAN_ID_EXT)
    {
        ret |= CAN_EFF_FLAG;
    }
    else if (id_type == CAN_ID_TYPE::CAN_ID_RTR)
    {
        ret |= CAN_RTR_FLAG;
    }
    else if (id_type == CAN_ID_TYPE::CAN_ID_ERR)
    {
        ret |= CAN_ERR_FLAG;
    }
    return ret;
}

void Can::set_callback_map(const uint32_t &id, const std::function<void(const std::shared_ptr<can_frame>&)> callback)
{
    std::lock_guard<std::mutex> lock(recv_callback_map_mutex);
    recv_callback_map[id] = callback;
}

void Can::register_msg(const uint32_t &id, const CAN_ID_TYPE &id_type, const std::function<void(const std::shared_ptr<can_frame>&)> callback)
{
    set_callback_map(id, callback);
    filters[filter_size].can_id = id ;
    filters[filter_size].can_id = set_id_type(id_type,filters[filter_size].can_id);
    if (id_type == CAN_ID_STD)
        filters[filter_size].can_mask = CAN_SFF_MASK;
    else if (id_type == CAN_ID_EXT)
        filters[filter_size].can_mask = CAN_EFF_MASK,printf("1\n");
    else if (id_type == CAN_ID_ERR)
        filters[filter_size].can_mask = CAN_ERR_MASK;
    filter_size++;
}

void Can::send_can(const uint32_t &id, const CAN_ID_TYPE &id_type, const int &dlc,const std::array<uint8_t,8> &data)
{
    struct can_frame frame{};
    
    frame.can_id = set_id_type(id_type,id);
    frame.can_dlc = dlc;
    for (int i = 0;i < dlc;i++)
    {
        frame.data[i] = data[i];
    }
    send_can(frame);
}

void Can::send_can(const can_frame &frame)
{
    std::lock_guard<std::mutex> lock(send_que_mutex);
    constexpr std::size_t kMaxQueuedFrames = 64;
    if (send_que.size() >= kMaxQueuedFrames) {
        std::cerr << "CAN send queue overflow, dropping stale frames" << std::endl;
        std::queue<can_frame> empty;
        send_que.swap(empty);
    }
    if (isDestroyed || !Started) return;
    send_que.push(frame);
    if (send_que.size()==1)
    {
        send_que_cv.notify_one();
    }
}

void Can::receive_thread()
{
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(send_que_mutex);
            if (isDestroyed) break;
        }
        pollfd descriptor{can_fd_read, POLLIN, 0};
        const int ready = poll(&descriptor, 1, 100);
        if (ready < 0) { if (errno == EINTR) continue; break; }
        if (ready == 0) continue;
        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) break;
        auto frame_ptr = std::make_shared<can_frame>();
        int nbytes = read(can_fd_read,frame_ptr.get(),sizeof(struct can_frame));
        if (nbytes == sizeof(struct can_frame))
        {
            if (frame_ptr->can_id & CAN_ERR_FLAG)
            {
                frame_ptr->can_id &= CAN_ERR_MASK;
            }
            else if (frame_ptr->can_id & CAN_EFF_FLAG)
            {
                frame_ptr->can_id &= CAN_EFF_MASK;
            }
            else if (frame_ptr->can_id & CAN_RTR_FLAG)
            {
                frame_ptr->can_id &= CAN_SFF_MASK;
            }
            else
            {
                frame_ptr->can_id &= CAN_SFF_MASK;
            }
            std::lock_guard<std::mutex> lock(recv_callback_map_mutex);
            if (recv_callback_map.find(frame_ptr->can_id) != recv_callback_map.end())
            {
                recv_callback_map[frame_ptr->can_id](frame_ptr);
            }

        }
    }
}

void Can::send_thread()
{
    auto last_error = std::chrono::steady_clock::time_point{};
    unsigned errors = 0;
    while (true)
    {
        std::unique_lock<std::mutex> lock1(send_que_mutex);
        send_que_cv.wait(lock1, [this](){return isDestroyed || !send_que.empty();});
        if (send_que.empty()) break;
        struct can_frame frame = send_que.front();
        send_que.pop();
        lock1.unlock();
        std::lock_guard<std::mutex> lock2(can_fd_write_mutex);
        // 失败即丢弃，禁止恢复后重放旧速度；不阻塞退出和停车报文。
        const ssize_t ret = send(can_fd_write, &frame, sizeof(frame), MSG_DONTWAIT);
        if (ret != sizeof(frame)) {
            const int error = errno;
            {
                std::lock_guard<std::mutex> lock(send_que_mutex);
                Started = false;
                std::queue<can_frame> empty;
                send_que.swap(empty);
            }
            ++errors;
            const auto now = std::chrono::steady_clock::now();
            if (last_error.time_since_epoch().count() == 0 || now - last_error >= std::chrono::seconds(1)) {
                std::cerr << "CAN TX failed: " << can_name << " errno=" << error
                          << " (" << std::strerror(error) << ") can_id=0x"
                          << std::hex << frame.can_id << std::dec << " errors=" << errors
                          << "; frame dropped, not retried" << std::endl;
                last_error = now;
            }
        }
    }
}

bool Can::send_can_with_respond(const uint32_t &id, const CAN_ID_TYPE &id_type, const int &dlc, const std::array<uint8_t, 8> &data)
{
    struct can_frame frame{};
    frame.can_id = set_id_type(id_type, id);
    frame.can_dlc = dlc;
    for (int i = 0;i < dlc;i++)
    {
        frame.data[i] = data[i];
    }
    return send_can_with_respond(frame);
}

bool Can::send_can_with_respond(const can_frame &frame)
{
    std::lock_guard<std::mutex> lock(can_fd_write_mutex);
    ssize_t ret=send(can_fd_write,&frame,sizeof(frame),MSG_DONTWAIT);
    return ret==sizeof(frame);
}

int Can::set_recv_filter()
{
    return setsockopt(can_fd_read,SOL_CAN_RAW,CAN_RAW_FILTER,
        filters.data(),sizeof(struct can_filter)*filter_size);
}

bool Can::isStarted() const
{
    std::lock_guard<std::mutex> lock(send_que_mutex);
    return Started;
}
