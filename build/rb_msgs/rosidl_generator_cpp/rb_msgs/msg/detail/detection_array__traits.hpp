// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rb_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__TRAITS_HPP_
#define RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rb_msgs/msg/detail/detection_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'detections'
#include "rb_msgs/msg/detail/detection__traits.hpp"

namespace rb_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DetectionArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: frame_id_seq
  {
    out << "frame_id_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id_seq, out);
    out << ", ";
  }

  // member: fx
  {
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << ", ";
  }

  // member: fy
  {
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << ", ";
  }

  // member: cx_cam
  {
    out << "cx_cam: ";
    rosidl_generator_traits::value_to_yaml(msg.cx_cam, out);
    out << ", ";
  }

  // member: cy_cam
  {
    out << "cy_cam: ";
    rosidl_generator_traits::value_to_yaml(msg.cy_cam, out);
    out << ", ";
  }

  // member: img_width
  {
    out << "img_width: ";
    rosidl_generator_traits::value_to_yaml(msg.img_width, out);
    out << ", ";
  }

  // member: img_height
  {
    out << "img_height: ";
    rosidl_generator_traits::value_to_yaml(msg.img_height, out);
    out << ", ";
  }

  // member: detections
  {
    if (msg.detections.size() == 0) {
      out << "detections: []";
    } else {
      out << "detections: [";
      size_t pending_items = msg.detections.size();
      for (auto item : msg.detections) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectionArray & msg,
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

  // member: frame_id_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_id_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id_seq, out);
    out << "\n";
  }

  // member: fx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << "\n";
  }

  // member: fy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << "\n";
  }

  // member: cx_cam
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cx_cam: ";
    rosidl_generator_traits::value_to_yaml(msg.cx_cam, out);
    out << "\n";
  }

  // member: cy_cam
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cy_cam: ";
    rosidl_generator_traits::value_to_yaml(msg.cy_cam, out);
    out << "\n";
  }

  // member: img_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "img_width: ";
    rosidl_generator_traits::value_to_yaml(msg.img_width, out);
    out << "\n";
  }

  // member: img_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "img_height: ";
    rosidl_generator_traits::value_to_yaml(msg.img_height, out);
    out << "\n";
  }

  // member: detections
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.detections.size() == 0) {
      out << "detections: []\n";
    } else {
      out << "detections:\n";
      for (auto item : msg.detections) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectionArray & msg, bool use_flow_style = false)
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
  const rb_msgs::msg::DetectionArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  rb_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rb_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rb_msgs::msg::DetectionArray & msg)
{
  return rb_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rb_msgs::msg::DetectionArray>()
{
  return "rb_msgs::msg::DetectionArray";
}

template<>
inline const char * name<rb_msgs::msg::DetectionArray>()
{
  return "rb_msgs/msg/DetectionArray";
}

template<>
struct has_fixed_size<rb_msgs::msg::DetectionArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rb_msgs::msg::DetectionArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rb_msgs::msg::DetectionArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__TRAITS_HPP_
