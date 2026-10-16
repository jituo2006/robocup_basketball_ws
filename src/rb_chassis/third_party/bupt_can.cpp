#include "bupt_can/bupt_can.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <unistd.h>
#include <poll.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <system_error>

namespace {
// Keep Can's installed motor_lib ABI unchanged: no added/reordered data members.
struct LinkState {
    std::atomic<uint64_t> generation{0};
    std::atomic<uint64_t> last_log_ms{0};
};
std::mutex links_mutex;
std::map<const Can*, std::shared_ptr<LinkState>> links;
std::shared_ptr<LinkState> link_state(const Can *can) {
    std::lock_guard<std::mutex> lock(links_mutex);
    auto it = links.find(can);
    return it == links.end() ? nullptr : it->second;
}
void fault(const std::shared_ptr<LinkState> &state, const std::string &name, int error) {
    if (!state) return;
    state->generation.fetch_add(1, std::memory_order_relaxed);
    const auto now = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    auto previous = state->last_log_ms.load();
    if (now - previous >= 1000 && state->last_log_ms.compare_exchange_strong(previous, now))
        std::cerr << name << ": CAN I/O fault (" << error
                  << "), dropped frame; board handshake must restart\n";
}
}

Can::Can(const std::string &name) {
    can_name = name;
    can_fd_read = can_fd_write = -1;
    isDestroyed = false;
    Started = false;
    filter_size = 0;
    ifr = {};
    addr = {};
    if (name.empty() || name.size() >= IFNAMSIZ)
        throw std::invalid_argument("invalid CAN interface name");
    try {
        can_fd_read = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd_read < 0) throw std::system_error(errno, std::generic_category(), "CAN RX socket");
        can_fd_write = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd_write < 0) throw std::system_error(errno, std::generic_category(), "CAN TX socket");
        std::memcpy(ifr.ifr_name, name.c_str(), name.size()+1);
        if (ioctl(can_fd_read, SIOCGIFINDEX, &ifr) < 0)
            throw std::system_error(errno, std::generic_category(), "CAN interface index");
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_fd_read, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
            bind(can_fd_write, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            throw std::system_error(errno, std::generic_category(), "CAN bind");
        // Bound socket memory as well as the interface qdisc; never retry old motion frames.
        const int buffer = 4096;
        if (setsockopt(can_fd_write, SOL_SOCKET, SO_SNDBUF, &buffer, sizeof(buffer)) < 0)
            throw std::system_error(errno, std::generic_category(), "CAN TX buffer");
        const can_err_mask_t errors = CAN_ERR_BUSOFF | CAN_ERR_RESTARTED | CAN_ERR_CRTL | CAN_ERR_TX_TIMEOUT;
        if (setsockopt(can_fd_read, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0)
            throw std::system_error(errno, std::generic_category(), "CAN error filter");
        std::lock_guard<std::mutex> lock(links_mutex);
        links.emplace(this, std::make_shared<LinkState>());
    } catch (...) {
        if (can_fd_read >= 0) close(can_fd_read);
        if (can_fd_write >= 0) close(can_fd_write);
        throw;
    }
}

Can::~Can() {
    // Shutdown is local. It must work while the board has no power or CAN is bus-off.
    __atomic_store_n(&isDestroyed, true, __ATOMIC_RELEASE);
    if (recv_thread_ && recv_thread_->joinable()) recv_thread_->join();
    if (can_fd_read >= 0) close(can_fd_read);
    if (can_fd_write >= 0) close(can_fd_write);
    std::lock_guard<std::mutex> lock(links_mutex);
    links.erase(this);
}

void Can::can_start() {
    if (Started) return;
    if (set_recv_filter() < 0)
        throw std::system_error(errno, std::generic_category(), "CAN RX filter");
    recv_thread_ = std::make_unique<std::thread>(&Can::receive_thread, this);
    // No sender thread or application queue: current commands are submitted once.
    Started = true;
}

uint32_t Can::set_id_type(const CAN_ID_TYPE &type, const uint32_t &id) {
    if (type == CAN_ID_EXT) return id | CAN_EFF_FLAG;
    if (type == CAN_ID_RTR) return id | CAN_RTR_FLAG;
    if (type == CAN_ID_ERR) return id | CAN_ERR_FLAG;
    return id;
}

void Can::set_callback_map(const uint32_t &id,
        const std::function<void(const std::shared_ptr<can_frame>&)> callback) {
    std::lock_guard<std::mutex> lock(recv_callback_map_mutex);
    recv_callback_map[id] = callback;
}

void Can::register_msg(const uint32_t &id, const CAN_ID_TYPE &type,
        const std::function<void(const std::shared_ptr<can_frame>&)> callback) {
    if (filter_size >= static_cast<int>(filters.size())) throw std::length_error("too many CAN filters");
    set_callback_map(id, callback);
    auto &f = filters[filter_size++];
    f.can_id = set_id_type(type, id);
    f.can_mask = (type == CAN_ID_EXT ? CAN_EFF_MASK : CAN_SFF_MASK) | CAN_EFF_FLAG | CAN_RTR_FLAG;
    if (Started && set_recv_filter() < 0)
        throw std::system_error(errno, std::generic_category(), "CAN RX filter update");
}

void Can::send_can(const uint32_t &id, const CAN_ID_TYPE &type, const int &dlc,
                   const std::array<uint8_t,8> &data) {
    (void)send_can_with_respond(id, type, dlc, data);
}
void Can::send_can(const can_frame &frame) { (void)send_can_with_respond(frame); }
void Can::send_thread() {} // Retain symbol/layout compatibility with installed motor_lib.

bool Can::send_can_with_respond(const uint32_t &id, const CAN_ID_TYPE &type, const int &dlc,
                                const std::array<uint8_t,8> &data) {
    if (dlc < 0 || dlc > 8) { errno = EINVAL; return false; }
    can_frame frame{};
    frame.can_id = set_id_type(type, id);
    frame.can_dlc = static_cast<uint8_t>(dlc);
    std::memcpy(frame.data, data.data(), static_cast<std::size_t>(dlc));
    return send_can_with_respond(frame);
}

bool Can::send_can_with_respond(const can_frame &frame) {
    if (frame.can_dlc > 8) { errno = EINVAL; return false; }
    std::lock_guard<std::mutex> lock(can_fd_write_mutex);
    if (__atomic_load_n(&isDestroyed, __ATOMIC_ACQUIRE)) { errno = EBADF; return false; }
    const auto n = send(can_fd_write, &frame, sizeof(frame), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (n == static_cast<ssize_t>(sizeof(frame))) return true;
    const int error = n < 0 ? errno : EIO;
    fault(link_state(this), can_name, error);
    errno = error;
    return false; // Includes EAGAIN/ENOBUFS/ENETDOWN: no queue and no replay.
}

uint64_t Can::transport_generation() const {
    auto state = link_state(this);
    return state ? state->generation.load(std::memory_order_relaxed) : 0;
}

void Can::receive_thread() {
    auto state = link_state(this);
    while (!__atomic_load_n(&isDestroyed, __ATOMIC_ACQUIRE)) {
        pollfd pfd{can_fd_read, POLLIN, 0};
        const int count = poll(&pfd, 1, 100); // Poll wakeup only, never a command timeout.
        if (count < 0) {
            if (errno == EINTR) continue;
            fault(state, can_name, errno);
            return;
        }
        if (!count) continue;
        if (pfd.revents & (POLLNVAL | POLLHUP)) { fault(state, can_name, ENODEV); return; }
        if (!(pfd.revents & POLLIN)) {
            if (pfd.revents & POLLERR) {
                fault(state, can_name, EIO);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            continue;
        }
        auto frame = std::make_shared<can_frame>();
        const auto n = recv(can_fd_read, frame.get(), sizeof(*frame), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            fault(state, can_name, errno);
            continue;
        }
        if (n != static_cast<ssize_t>(sizeof(*frame)) || frame->can_dlc > 8) continue;
        if (frame->can_id & CAN_ERR_FLAG) { fault(state, can_name, EIO); continue; }
        if (frame->can_id & CAN_RTR_FLAG) continue;
        const uint32_t id = frame->can_id & (frame->can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
        std::function<void(std::shared_ptr<can_frame>)> callback;
        {
            std::lock_guard<std::mutex> lock(recv_callback_map_mutex);
            auto it = recv_callback_map.find(id);
            if (it != recv_callback_map.end()) callback = it->second;
        }
        if (callback) callback(frame); // Preserve EFF/RTR flags for protocol validation.
    }
}

int Can::set_recv_filter() {
    return setsockopt(can_fd_read, SOL_CAN_RAW, CAN_RAW_FILTER,
                      filters.data(), sizeof(can_filter)*filter_size);
}
bool Can::isStarted() const { return Started; }
