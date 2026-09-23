// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rb_msgs:msg/MissionStatus.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__MISSION_STATUS__TRAITS_HPP_
#define RB_MSGS__MSG__DETAIL__MISSION_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rb_msgs/msg/detail/mission_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rb_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MissionStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: mission
  {
    out << "mission: ";
    rosidl_generator_traits::value_to_yaml(msg.mission, out);
    out << ", ";
  }

  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: detail
  {
    out << "detail: ";
    rosidl_generator_traits::value_to_yaml(msg.detail, out);
    out << ", ";
  }

  // member: elapsed_s
  {
    out << "elapsed_s: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_s, out);
    out << ", ";
  }

  // member: score_estimate
  {
    out << "score_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.score_estimate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MissionStatus & msg,
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

  // member: mission
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission: ";
    rosidl_generator_traits::value_to_yaml(msg.mission, out);
    out << "\n";
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

  // member: detail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detail: ";
    rosidl_generator_traits::value_to_yaml(msg.detail, out);
    out << "\n";
  }

  // member: elapsed_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elapsed_s: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_s, out);
    out << "\n";
  }

  // member: score_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.score_estimate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MissionStatus & msg, bool use_flow_style = false)
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
  const rb_msgs::msg::MissionStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  rb_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rb_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rb_msgs::msg::MissionStatus & msg)
{
  return rb_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rb_msgs::msg::MissionStatus>()
{
  return "rb_msgs::msg::MissionStatus";
}

template<>
inline const char * name<rb_msgs::msg::MissionStatus>()
{
  return "rb_msgs/msg/MissionStatus";
}

template<>
struct has_fixed_size<rb_msgs::msg::MissionStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rb_msgs::msg::MissionStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rb_msgs::msg::MissionStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RB_MSGS__MSG__DETAIL__MISSION_STATUS__TRAITS_HPP_
