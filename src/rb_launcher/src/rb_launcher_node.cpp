// RoboCup 篮球机器人 · 弹射/运球/夹爪机构控制节点
//
// 帧协议直接移植自 ROBOCON 2025 的 ballrobot（serial_port_R1.cpp，未改协议）。
// 相对原实现的改动：
//   1. 从"服务端逻辑 + 手柄驱动"改成"服务驱动"：/rb_launcher/launch 一个服务搞定
//   2. 打开串口失败不再抛未捕获异常崩溃（原 R1_server 会 abort），改为降级 + 自动重连
//      （服务名用 ~/launch，解析为 /rb_launcher/launch）
//   3. 命令按固定频率重复发送（下位机靠重复帧取最新值，与原实现约定一致）
//
// 用法：
//   ros2 launch rb_launcher launcher.launch.py
//   ros2 service call /rb_launcher/launch rb_msgs/srv/Launch "{action: 0, speed: 2800, angle: 64}"
//   ros2 topic echo /rb_launcher/ok        # 串口是否正常

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <sys/stat.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "rb_msgs/srv/launch.hpp"
#include "serial_port/serial_port_R1.hpp"

namespace {

constexpr uint8_t kActionShoot = 0;
constexpr uint8_t kActionFastShoot = 1;
constexpr uint8_t kActionDribble = 2;
constexpr uint8_t kActionDoubleDribble = 3;
constexpr uint8_t kActionPawlToggle = 4;
constexpr uint8_t kActionSlidewayToggle = 5;
constexpr uint8_t kActionKeepLoop = 6;
constexpr uint8_t kActionStop = 7;

const char *actionName(uint8_t a) {
  switch (a) {
    case kActionShoot: return "SHOOT";
    case kActionFastShoot: return "FAST_SHOOT";
    case kActionDribble: return "DRIBBLE";
    case kActionDoubleDribble: return "DOUBLE_DRIBBLE";
    case kActionPawlToggle: return "PAWL_TOGGLE";
    case kActionSlidewayToggle: return "SLIDEWAY_TOGGLE";
    case kActionKeepLoop: return "KEEP_LOOP";
    case kActionStop: return "STOP";
    default: return "UNKNOWN";
  }
}

bool toFrameControl(uint8_t action, Message::FrameControl &out) {
  switch (action) {
    case kActionShoot: out = Message::FrameControl::SHOOT; return true;
    case kActionFastShoot: out = Message::FrameControl::FAST_SHOOT; return true;
    case kActionDribble: out = Message::FrameControl::DRIBBLE; return true;
    case kActionDoubleDribble: out = Message::FrameControl::DOUBLE_DRIBBLE; return true;
    case kActionPawlToggle: out = Message::FrameControl::PAWL; return true;
    case kActionSlidewayToggle: out = Message::FrameControl::SLIDEWAY; return true;
    case kActionKeepLoop: out = Message::FrameControl::KEEP_LOOP; return true;
    default: return false;
  }
}

// ── 串口设备名解析 ──────────────────────────────────────────────────────────
// 为什么需要：本机 udev 规则生成的是 /dev/usb2ttl，而配置里写的是上一届的
// /dev/R1_usb2ttl —— 对不上时节点只会一直报"打开机构串口失败"，机构不能用。
// 与其让人去猜设备名，不如按候选列表找**第一个真实存在**的：
//   ① 配置里写的名字（存在就优先用，兼容现场自定义）
//   ② fallback_ports 里的候选（相对 /dev 的名字或绝对路径都行）
// 找到哪个会在日志里明说，方便现场核对。全都没有才报错（仍会周期性重连）。
bool pathExists(const std::string &p) {
  struct stat st {};
  return ::stat(p.c_str(), &st) == 0;
}

std::string normalizeDev(const std::string &name) {
  if (name.empty() || name[0] == '/') return name;
  return "/dev/" + name;
}

std::string pickPort(const std::string &configured, const std::vector<std::string> &cands) {
  const std::string first = normalizeDev(configured);
  if (pathExists(first)) return first;
  for (const auto &c : cands) {
    const std::string p = normalizeDev(c);
    if (p == first) continue;
    if (pathExists(p)) return p;
  }
  return first;   // 都没有 → 返回配置值，让打开时报出原始名字
}

}  // namespace

class RbLauncherNode : public rclcpp::Node {
public:
  RbLauncherNode() : Node("rb_launcher") {
    port_ = declare_parameter<std::string>("port", "/dev/R1_usb2ttl");
    // 配置的 port 不存在时，按这个顺序找第一个存在的（相对 /dev 或绝对路径）。
    // ⚠️ 默认**空 = 不启用回退**，这是有意的安全选择：
    //    "usb2ttl" 这类名字是通配的（本机 udev 那条就没有端口/序列号限制），
    //    配置写错时会打开【别的设备】，把机构指令发过去 —— 机构可能乱动。
    //    （实测被集成测试抓到：期望"端口不存在→拒绝"，结果回退到真机构并成功。）
    //    要启用就显式列候选，并且只列确定属于本机构的设备。
    fallback_ports_ = declare_parameter<std::vector<std::string>>(
        "fallback_ports", std::vector<std::string>{});
    baud_ = static_cast<unsigned int>(declare_parameter<int>("baud", 115200));
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 50.0);
    default_speed_ = static_cast<uint16_t>(declare_parameter<int>("default_speed", 2800));
    default_angle_ = static_cast<uint16_t>(declare_parameter<int>("default_angle", 64));
    auto_reconnect_ = declare_parameter<bool>("auto_reconnect", true);
    reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 2.0);

    ok_pub_ = create_publisher<std_msgs::msg::Bool>("/rb_launcher/ok", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("/rb_launcher/status", 10);

    srv_ = create_service<rb_msgs::srv::Launch>(
        "~/launch", std::bind(&RbLauncherNode::onLaunch, this, std::placeholders::_1, std::placeholders::_2));

    openPort();

    const auto period = std::chrono::duration<double>(1.0 / std::max(send_rate_hz_, 1.0));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&RbLauncherNode::tick, this));

    if (auto_reconnect_) {
      reconnect_timer_ = create_wall_timer(
          std::chrono::milliseconds(static_cast<int64_t>(reconnect_period_s_ * 1000.0)),
          std::bind(&RbLauncherNode::tryReconnect, this));
    }
  }

  ~RbLauncherNode() override {
    std::lock_guard<std::mutex> lock(mtx_);
    serial_.reset();
  }

private:
  void openPort() {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
      ioc_ = std::make_unique<boost::asio::io_context>();
      const std::string actual = pickPort(port_, fallback_ports_);
      serial_ = std::make_unique<SerialPortR1>(*ioc_, actual, baud_);
      port_ok_ = true;
      publishOk();
      if (actual == normalizeDev(port_)) {
        RCLCPP_INFO(get_logger(), "机构串口已打开: %s @ %u", actual.c_str(), baud_);
      } else {
        RCLCPP_WARN(get_logger(),
                    "配置的串口 %s 不存在，已自动改用 %s @ %u"
                    "（要固定就补 udev 规则或改 port 参数）",
                    port_.c_str(), actual.c_str(), baud_);
      }
    } catch (const std::exception &e) {
      // 原实现（ballrobot/R1_server）在这里会抛未捕获异常直接 abort，
      // 现在改成降级：节点继续运行，只是报 ok=false，并周期性尝试重连。
      serial_.reset();
      port_ok_ = false;
      publishOk();
      // 把"试过哪些名字"打出来 —— 现场最常见的困惑是"设备明明插了却连不上"，
        // 有这份列表就能立刻分辨是名字不对还是驱动没认到设备。
        std::string tried = normalizeDev(port_);
        for (const auto &c : fallback_ports_) {
          const std::string n = normalizeDev(c);
          if (n != tried) tried += ", " + n;
        }
        RCLCPP_ERROR(get_logger(),
                     "打开机构串口失败: %s (%s)。已试过: %s。"
                     "节点继续运行并每 %.1fs 重连；请确认设备插好/驱动已加载，"
                     "或补一条 udev 规则把它固定成 port 里的名字。",
                     port_.c_str(), e.what(), tried.c_str(), reconnect_period_s_);
    }
  }

  void publishOk() {
    std_msgs::msg::Bool msg;
    msg.data = port_ok_;
    ok_pub_->publish(msg);
  }

  void tryReconnect() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (port_ok_) return;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "尝试重连机构串口 %s ...", port_.c_str());
    try {
      ioc_ = std::make_unique<boost::asio::io_context>();
      serial_ = std::make_unique<SerialPortR1>(*ioc_, port_, baud_);
      port_ok_ = true;
      RCLCPP_INFO(get_logger(), "机构串口重连成功: %s", port_.c_str());
    } catch (const std::exception &) {
      port_ok_ = false;
      serial_.reset();
    }
    publishOk();
  }

  void onLaunch(const std::shared_ptr<rb_msgs::srv::Launch::Request> req,
                std::shared_ptr<rb_msgs::srv::Launch::Response> res) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (req->action == kActionStop) {
      active_ = false;
      res->success = true;
      res->message = "已停止重复发送";
      publishStatus("STOP", true, res->message);
      return;
    }

    Message::FrameControl fc;
    if (!toFrameControl(req->action, fc)) {
      res->success = false;
      res->message = "未知 action: " + std::to_string(req->action);
      publishStatus("UNKNOWN", false, res->message);
      return;
    }

    cmd_ = fc;
    speed_ = req->speed != 0 ? req->speed : default_speed_;
    angle_ = req->angle != 0 ? req->angle : default_angle_;
    active_ = false;

    if (!port_ok_) {
      res->success = false;
      res->message = "串口未就绪，命令已拒绝（port=" + port_ + "）";
      publishStatus(actionName(req->action), false, res->message);
      return;
    }

    serial_->put_packet(cmd_, speed_, angle_);
    const bool sent = serial_->send_command();
    // Toggle commands are edges; repeating them toggles the mechanism again.
    active_ = sent && req->action != kActionPawlToggle && req->action != kActionSlidewayToggle;
    if (!sent) {
      port_ok_ = false;
      serial_.reset();
      publishOk();
    }
    res->success = sent;
    res->message = sent ? "已发送 " + std::string(actionName(req->action)) : "发送失败";
    publishStatus(actionName(req->action), sent, res->message);
  }

  void publishStatus(const std::string &action, bool success, const std::string &msg) {
    std_msgs::msg::String out;
    out.data = "action=" + action + " success=" + (success ? "true" : "false") + " " + msg;
    status_pub_->publish(out);
  }

  void tick() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!active_ || !port_ok_ || !serial_) return;

    serial_->put_packet(cmd_, speed_, angle_);
    if (serial_->send_command()) return;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "机构串口写入失败，标记为未就绪");
    port_ok_ = false;
    active_ = false;  // Never replay a failed action on reconnect.
    serial_.reset();
    publishOk();
  }

  std::string port_;
  std::vector<std::string> fallback_ports_;
  unsigned int baud_ = 115200;
  double send_rate_hz_ = 50.0;
  uint16_t default_speed_ = 2800;
  uint16_t default_angle_ = 64;
  bool auto_reconnect_ = true;
  double reconnect_period_s_ = 2.0;

  std::mutex mtx_;
  std::unique_ptr<boost::asio::io_context> ioc_;
  std::unique_ptr<SerialPortR1> serial_;
  bool port_ok_ = false;

  bool active_ = false;
  Message::FrameControl cmd_ = Message::FrameControl::SHOOT;
  uint16_t speed_ = 0;
  uint16_t angle_ = 0;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<rb_msgs::srv::Launch>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RbLauncherNode>());
  rclcpp::shutdown();
  return 0;
}
