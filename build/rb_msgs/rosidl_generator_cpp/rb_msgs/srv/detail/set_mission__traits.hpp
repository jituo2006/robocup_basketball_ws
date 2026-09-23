// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rb_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__SRV__DETAIL__SET_MISSION__TRAITS_HPP_
#define RB_MSGS__SRV__DETAIL__SET_MISSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rb_msgs/srv/detail/set_mission__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rb_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetMission_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mission
  {
    out << "mission: ";
    rosidl_generator_traits::value_to_yaml(msg.mission, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetMission_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mission
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission: ";
    rosidl_generator_traits::value_to_yaml(msg.mission, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetMission_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rb_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rb_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rb_msgs::srv::SetMission_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rb_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rb_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rb_msgs::srv::SetMission_Request & msg)
{
  return rb_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rb_msgs::srv::SetMission_Request>()
{
  return "rb_msgs::srv::SetMission_Request";
}

template<>
inline const char * name<rb_msgs::srv::SetMission_Request>()
{
  return "rb_msgs/srv/SetMission_Request";
}

template<>
struct has_fixed_size<rb_msgs::srv::SetMission_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rb_msgs::srv::SetMission_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rb_msgs::srv::SetMission_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rb_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetMission_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetMission_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetMission_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rb_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rb_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rb_msgs::srv::SetMission_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rb_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rb_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rb_msgs::srv::SetMission_Response & msg)
{
  return rb_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rb_msgs::srv::SetMission_Response>()
{
  return "rb_msgs::srv::SetMission_Response";
}

template<>
inline const char * name<rb_msgs::srv::SetMission_Response>()
{
  return "rb_msgs/srv/SetMission_Response";
}

template<>
struct has_fixed_size<rb_msgs::srv::SetMission_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rb_msgs::srv::SetMission_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rb_msgs::srv::SetMission_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rb_msgs::srv::SetMission>()
{
  return "rb_msgs::srv::SetMission";
}

template<>
inline const char * name<rb_msgs::srv::SetMission>()
{
  return "rb_msgs/srv/SetMission";
}

template<>
struct has_fixed_size<rb_msgs::srv::SetMission>
  : std::integral_constant<
    bool,
    has_fixed_size<rb_msgs::srv::SetMission_Request>::value &&
    has_fixed_size<rb_msgs::srv::SetMission_Response>::value
  >
{
};

template<>
struct has_bounded_size<rb_msgs::srv::SetMission>
  : std::integral_constant<
    bool,
    has_bounded_size<rb_msgs::srv::SetMission_Request>::value &&
    has_bounded_size<rb_msgs::srv::SetMission_Response>::value
  >
{
};

template<>
struct is_service<rb_msgs::srv::SetMission>
  : std::true_type
{
};

template<>
struct is_service_request<rb_msgs::srv::SetMission_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rb_msgs::srv::SetMission_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RB_MSGS__SRV__DETAIL__SET_MISSION__TRAITS_HPP_
