// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rb_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__SRV__DETAIL__SET_MISSION__BUILDER_HPP_
#define RB_MSGS__SRV__DETAIL__SET_MISSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rb_msgs/srv/detail/set_mission__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rb_msgs
{

namespace srv
{

namespace builder
{

class Init_SetMission_Request_mission
{
public:
  Init_SetMission_Request_mission()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rb_msgs::srv::SetMission_Request mission(::rb_msgs::srv::SetMission_Request::_mission_type arg)
  {
    msg_.mission = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::srv::SetMission_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::srv::SetMission_Request>()
{
  return rb_msgs::srv::builder::Init_SetMission_Request_mission();
}

}  // namespace rb_msgs


namespace rb_msgs
{

namespace srv
{

namespace builder
{

class Init_SetMission_Response_message
{
public:
  explicit Init_SetMission_Response_message(::rb_msgs::srv::SetMission_Response & msg)
  : msg_(msg)
  {}
  ::rb_msgs::srv::SetMission_Response message(::rb_msgs::srv::SetMission_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::srv::SetMission_Response msg_;
};

class Init_SetMission_Response_accepted
{
public:
  Init_SetMission_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMission_Response_message accepted(::rb_msgs::srv::SetMission_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_SetMission_Response_message(msg_);
  }

private:
  ::rb_msgs::srv::SetMission_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::srv::SetMission_Response>()
{
  return rb_msgs::srv::builder::Init_SetMission_Response_accepted();
}

}  // namespace rb_msgs

#endif  // RB_MSGS__SRV__DETAIL__SET_MISSION__BUILDER_HPP_
