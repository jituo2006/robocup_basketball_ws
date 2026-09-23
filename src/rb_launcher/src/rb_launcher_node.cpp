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

}  // namespace

class RbLauncherNode : public rclcpp::Node {
public:
  RbLauncherNode() : Node("rb_launcher") {
    port_ = declare_parameter<std::string>("port", "/dev/R1_usb2ttl");
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
      serial_ = std::make_unique<SerialPortR1>(*ioc_, port_, baud_);
      port_ok_ = true;
      publishOk();
      RCLCPP_INFO(get_logger(), "机构串口已打开: %s @ %u", port_.c_str(), baud_);
    } catch (const std::exception &e) {
      // 原实现（ballrobot/R1_server）在这里会抛未捕获异常直接 abort，
      // 现在改成降级：节点继续运行，只是报 ok=false，并周期性尝试重连。
      serial_.reset();
      port_ok_ = false;
      publishOk();
      RCLCPP_ERROR(get_logger(),
                   "打开机构串口失败: %s (%s)。节点继续运行；"
                   "请检查设备名与 udev 规则，或把参数 port 改成实际设备。",
                   port_.c_str(), e.what());
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
    active_ = true;

    if (!port_ok_) {
      res->success = false;
      res->message = "串口未就绪，命令已记录但未发出（port=" + port_ + "）";
      publishStatus(actionName(req->action), false, res->message);
      return;
    }

    const bool sent = serial_->send_command();
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

    if (serial_->send_command()) return;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "机构串口写入失败，标记为未就绪");
    port_ok_ = false;
    serial_.reset();
    publishOk();
  }

  std::string port_;
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
