// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rb_msgs:msg/MissionStatus.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__MISSION_STATUS__BUILDER_HPP_
#define RB_MSGS__MSG__DETAIL__MISSION_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rb_msgs/msg/detail/mission_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rb_msgs
{

namespace msg
{

namespace builder
{

class Init_MissionStatus_score_estimate
{
public:
  explicit Init_MissionStatus_score_estimate(::rb_msgs::msg::MissionStatus & msg)
  : msg_(msg)
  {}
  ::rb_msgs::msg::MissionStatus score_estimate(::rb_msgs::msg::MissionStatus::_score_estimate_type arg)
  {
    msg_.score_estimate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::msg::MissionStatus msg_;
};

class Init_MissionStatus_elapsed_s
{
public:
  explicit Init_MissionStatus_elapsed_s(::rb_msgs::msg::MissionStatus & msg)
  : msg_(msg)
  {}
  Init_MissionStatus_score_estimate elapsed_s(::rb_msgs::msg::MissionStatus::_elapsed_s_type arg)
  {
    msg_.elapsed_s = std::move(arg);
    return Init_MissionStatus_score_estimate(msg_);
  }

private:
  ::rb_msgs::msg::MissionStatus msg_;
};

class Init_MissionStatus_detail
{
public:
  explicit Init_MissionStatus_detail(::rb_msgs::msg::MissionStatus & msg)
  : msg_(msg)
  {}
  Init_MissionStatus_elapsed_s detail(::rb_msgs::msg::MissionStatus::_detail_type arg)
  {
    msg_.detail = std::move(arg);
    return Init_MissionStatus_elapsed_s(msg_);
  }

private:
  ::rb_msgs::msg::MissionStatus msg_;
};

class Init_MissionStatus_phase
{
public:
  explicit Init_MissionStatus_phase(::rb_msgs::msg::MissionStatus & msg)
  : msg_(msg)
  {}
  Init_MissionStatus_detail phase(::rb_msgs::msg::MissionStatus::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_MissionStatus_detail(msg_);
  }

private:
  ::rb_msgs::msg::MissionStatus msg_;
};

class Init_MissionStatus_mission
{
public:
  explicit Init_MissionStatus_mission(::rb_msgs::msg::MissionStatus & msg)
  : msg_(msg)
  {}
  Init_MissionStatus_phase mission(::rb_msgs::msg::MissionStatus::_mission_type arg)
  {
    msg_.mission = std::move(arg);
    return Init_MissionStatus_phase(msg_);
  }

private:
  ::rb_msgs::msg::MissionStatus msg_;
};

class Init_MissionStatus_header
{
public:
  Init_MissionStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MissionStatus_mission header(::rb_msgs::msg::MissionStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MissionStatus_mission(msg_);
  }

private:
  ::rb_msgs::msg::MissionStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::msg::MissionStatus>()
{
  return rb_msgs::msg::builder::Init_MissionStatus_header();
}

}  // namespace rb_msgs

#endif  // RB_MSGS__MSG__DETAIL__MISSION_STATUS__BUILDER_HPP_
