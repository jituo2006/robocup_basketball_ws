// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rb_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define RB_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rb_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rb_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_perception_ok
{
public:
  explicit Init_RobotState_perception_ok(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::rb_msgs::msg::RobotState perception_ok(::rb_msgs::msg::RobotState::_perception_ok_type arg)
  {
    msg_.perception_ok = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_launcher_ok
{
public:
  explicit Init_RobotState_launcher_ok(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_perception_ok launcher_ok(::rb_msgs::msg::RobotState::_launcher_ok_type arg)
  {
    msg_.launcher_ok = std::move(arg);
    return Init_RobotState_perception_ok(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_localization_ok
{
public:
  explicit Init_RobotState_localization_ok(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_launcher_ok localization_ok(::rb_msgs::msg::RobotState::_localization_ok_type arg)
  {
    msg_.localization_ok = std::move(arg);
    return Init_RobotState_launcher_ok(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_estop
{
public:
  explicit Init_RobotState_estop(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_localization_ok estop(::rb_msgs::msg::RobotState::_estop_type arg)
  {
    msg_.estop = std::move(arg);
    return Init_RobotState_localization_ok(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_in_shoot_zone_outside
{
public:
  explicit Init_RobotState_in_shoot_zone_outside(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_estop in_shoot_zone_outside(::rb_msgs::msg::RobotState::_in_shoot_zone_outside_type arg)
  {
    msg_.in_shoot_zone_outside = std::move(arg);
    return Init_RobotState_estop(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_in_pass_zone
{
public:
  explicit Init_RobotState_in_pass_zone(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_in_shoot_zone_outside in_pass_zone(::rb_msgs::msg::RobotState::_in_pass_zone_type arg)
  {
    msg_.in_pass_zone = std::move(arg);
    return Init_RobotState_in_shoot_zone_outside(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_ball_type
{
public:
  explicit Init_RobotState_ball_type(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_in_pass_zone ball_type(::rb_msgs::msg::RobotState::_ball_type_type arg)
  {
    msg_.ball_type = std::move(arg);
    return Init_RobotState_in_pass_zone(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_ball_count
{
public:
  explicit Init_RobotState_ball_count(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_ball_type ball_count(::rb_msgs::msg::RobotState::_ball_count_type arg)
  {
    msg_.ball_count = std::move(arg);
    return Init_RobotState_ball_type(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_has_ball
{
public:
  explicit Init_RobotState_has_ball(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_ball_count has_ball(::rb_msgs::msg::RobotState::_has_ball_type arg)
  {
    msg_.has_ball = std::move(arg);
    return Init_RobotState_ball_count(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_phase
{
public:
  explicit Init_RobotState_phase(::rb_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_has_ball phase(::rb_msgs::msg::RobotState::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_RobotState_has_ball(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

class Init_RobotState_header
{
public:
  Init_RobotState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_phase header(::rb_msgs::msg::RobotState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotState_phase(msg_);
  }

private:
  ::rb_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::msg::RobotState>()
{
  return rb_msgs::msg::builder::Init_RobotState_header();
}

}  // namespace rb_msgs

#endif  // RB_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
