// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rb_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
#define RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rb_msgs/msg/detail/detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rb_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionArray_detections
{
public:
  explicit Init_DetectionArray_detections(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  ::rb_msgs::msg::DetectionArray detections(::rb_msgs::msg::DetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_img_height
{
public:
  explicit Init_DetectionArray_img_height(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_detections img_height(::rb_msgs::msg::DetectionArray::_img_height_type arg)
  {
    msg_.img_height = std::move(arg);
    return Init_DetectionArray_detections(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_img_width
{
public:
  explicit Init_DetectionArray_img_width(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_img_height img_width(::rb_msgs::msg::DetectionArray::_img_width_type arg)
  {
    msg_.img_width = std::move(arg);
    return Init_DetectionArray_img_height(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_cy_cam
{
public:
  explicit Init_DetectionArray_cy_cam(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_img_width cy_cam(::rb_msgs::msg::DetectionArray::_cy_cam_type arg)
  {
    msg_.cy_cam = std::move(arg);
    return Init_DetectionArray_img_width(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_cx_cam
{
public:
  explicit Init_DetectionArray_cx_cam(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_cy_cam cx_cam(::rb_msgs::msg::DetectionArray::_cx_cam_type arg)
  {
    msg_.cx_cam = std::move(arg);
    return Init_DetectionArray_cy_cam(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_fy
{
public:
  explicit Init_DetectionArray_fy(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_cx_cam fy(::rb_msgs::msg::DetectionArray::_fy_type arg)
  {
    msg_.fy = std::move(arg);
    return Init_DetectionArray_cx_cam(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_fx
{
public:
  explicit Init_DetectionArray_fx(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_fy fx(::rb_msgs::msg::DetectionArray::_fx_type arg)
  {
    msg_.fx = std::move(arg);
    return Init_DetectionArray_fy(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_frame_id_seq
{
public:
  explicit Init_DetectionArray_frame_id_seq(::rb_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  Init_DetectionArray_fx frame_id_seq(::rb_msgs::msg::DetectionArray::_frame_id_seq_type arg)
  {
    msg_.frame_id_seq = std::move(arg);
    return Init_DetectionArray_fx(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_header
{
public:
  Init_DetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionArray_frame_id_seq header(::rb_msgs::msg::DetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectionArray_frame_id_seq(msg_);
  }

private:
  ::rb_msgs::msg::DetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::msg::DetectionArray>()
{
  return rb_msgs::msg::builder::Init_DetectionArray_header();
}

}  // namespace rb_msgs

#endif  // RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
