#pragma once
#include <functional>
#include <memory>
#include <string>
namespace rclcpp {
template<class T> struct Subscription { using SharedPtr=std::shared_ptr<Subscription>; };
struct Node {
  explicit Node(const std::string&) {}
  virtual ~Node()=default;
  template<class T,class F> typename Subscription<T>::SharedPtr create_subscription(const std::string&,int,F) {
    return std::make_shared<Subscription<T>>();
  }
};
inline std::function<void()> shutdown_hook;
template<class F> void on_shutdown(F fn) { shutdown_hook=fn; }
}
#define RCLCPP_INFO(...) do {} while(0)
#define RCLCPP_WARN(...) do {} while(0)
#define RCLCPP_DEBUG(...) do {} while(0)
#define RCLCPP_DEBUG_THROTTLE(...) do {} while(0)
