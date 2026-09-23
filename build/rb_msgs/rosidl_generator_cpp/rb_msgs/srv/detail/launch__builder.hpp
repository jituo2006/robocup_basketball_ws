// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rb_msgs:srv/Launch.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__SRV__DETAIL__LAUNCH__BUILDER_HPP_
#define RB_MSGS__SRV__DETAIL__LAUNCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rb_msgs/srv/detail/launch__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rb_msgs
{

namespace srv
{

namespace builder
{

class Init_Launch_Request_angle
{
public:
  explicit Init_Launch_Request_angle(::rb_msgs::srv::Launch_Request & msg)
  : msg_(msg)
  {}
  ::rb_msgs::srv::Launch_Request angle(::rb_msgs::srv::Launch_Request::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::srv::Launch_Request msg_;
};

class Init_Launch_Request_speed
{
public:
  explicit Init_Launch_Request_speed(::rb_msgs::srv::Launch_Request & msg)
  : msg_(msg)
  {}
  Init_Launch_Request_angle speed(::rb_msgs::srv::Launch_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_Launch_Request_angle(msg_);
  }

private:
  ::rb_msgs::srv::Launch_Request msg_;
};

class Init_Launch_Request_action
{
public:
  Init_Launch_Request_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Launch_Request_speed action(::rb_msgs::srv::Launch_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_Launch_Request_speed(msg_);
  }

private:
  ::rb_msgs::srv::Launch_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::srv::Launch_Request>()
{
  return rb_msgs::srv::builder::Init_Launch_Request_action();
}

}  // namespace rb_msgs


namespace rb_msgs
{

namespace srv
{

namespace builder
{

class Init_Launch_Response_message
{
public:
  explicit Init_Launch_Response_message(::rb_msgs::srv::Launch_Response & msg)
  : msg_(msg)
  {}
  ::rb_msgs::srv::Launch_Response message(::rb_msgs::srv::Launch_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::srv::Launch_Response msg_;
};

class Init_Launch_Response_success
{
public:
  Init_Launch_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Launch_Response_message success(::rb_msgs::srv::Launch_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Launch_Response_message(msg_);
  }

private:
  ::rb_msgs::srv::Launch_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::srv::Launch_Response>()
{
  return rb_msgs::srv::builder::Init_Launch_Response_success();
}

}  // namespace rb_msgs

#endif  // RB_MSGS__SRV__DETAIL__LAUNCH__BUILDER_HPP_
