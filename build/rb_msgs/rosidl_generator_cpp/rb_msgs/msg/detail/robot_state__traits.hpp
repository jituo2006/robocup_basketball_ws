// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rb_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define RB_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rb_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rb_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: has_ball
  {
    out << "has_ball: ";
    rosidl_generator_traits::value_to_yaml(msg.has_ball, out);
    out << ", ";
  }

  // member: ball_count
  {
    out << "ball_count: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_count, out);
    out << ", ";
  }

  // member: ball_type
  {
    out << "ball_type: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_type, out);
    out << ", ";
  }

  // member: in_pass_zone
  {
    out << "in_pass_zone: ";
    rosidl_generator_traits::value_to_yaml(msg.in_pass_zone, out);
    out << ", ";
  }

  // member: in_shoot_zone_outside
  {
    out << "in_shoot_zone_outside: ";
    rosidl_generator_traits::value_to_yaml(msg.in_shoot_zone_outside, out);
    out << ", ";
  }

  // member: estop
  {
    out << "estop: ";
    rosidl_generator_traits::value_to_yaml(msg.estop, out);
    out << ", ";
  }

  // member: localization_ok
  {
    out << "localization_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.localization_ok, out);
    out << ", ";
  }

  // member: launcher_ok
  {
    out << "launcher_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.launcher_ok, out);
    out << ", ";
  }

  // member: perception_ok
  {
    out << "perception_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.perception_ok, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: has_ball
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_ball: ";
    rosidl_generator_traits::value_to_yaml(msg.has_ball, out);
    out << "\n";
  }

  // member: ball_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_count: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_count, out);
    out << "\n";
  }

  // member: ball_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_type: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_type, out);
    out << "\n";
  }

  // member: in_pass_zone
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_pass_zone: ";
    rosidl_generator_traits::value_to_yaml(msg.in_pass_zone, out);
    out << "\n";
  }

  // member: in_shoot_zone_outside
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_shoot_zone_outside: ";
    rosidl_generator_traits::value_to_yaml(msg.in_shoot_zone_outside, out);
    out << "\n";
  }

  // member: estop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estop: ";
    rosidl_generator_traits::value_to_yaml(msg.estop, out);
    out << "\n";
  }

  // member: localization_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "localization_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.localization_ok, out);
    out << "\n";
  }

  // member: launcher_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "launcher_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.launcher_ok, out);
    out << "\n";
  }

  // member: perception_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "perception_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.perception_ok, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rb_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rb_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rb_msgs::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  rb_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rb_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rb_msgs::msg::RobotState & msg)
{
  return rb_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rb_msgs::msg::RobotState>()
{
  return "rb_msgs::msg::RobotState";
}

template<>
inline const char * name<rb_msgs::msg::RobotState>()
{
  return "rb_msgs/msg/RobotState";
}

template<>
struct has_fixed_size<rb_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rb_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rb_msgs::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RB_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
