// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rb_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define RB_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rb_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rb_msgs
{

namespace msg
{

namespace builder
{

class Init_Detection_diameter_m
{
public:
  explicit Init_Detection_diameter_m(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  ::rb_msgs::msg::Detection diameter_m(::rb_msgs::msg::Detection::_diameter_m_type arg)
  {
    msg_.diameter_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_distance_m
{
public:
  explicit Init_Detection_distance_m(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_diameter_m distance_m(::rb_msgs::msg::Detection::_distance_m_type arg)
  {
    msg_.distance_m = std::move(arg);
    return Init_Detection_diameter_m(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_bearing_rad
{
public:
  explicit Init_Detection_bearing_rad(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_distance_m bearing_rad(::rb_msgs::msg::Detection::_bearing_rad_type arg)
  {
    msg_.bearing_rad = std::move(arg);
    return Init_Detection_distance_m(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_bbox_px_h
{
public:
  explicit Init_Detection_bbox_px_h(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bearing_rad bbox_px_h(::rb_msgs::msg::Detection::_bbox_px_h_type arg)
  {
    msg_.bbox_px_h = std::move(arg);
    return Init_Detection_bearing_rad(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_bbox_px_w
{
public:
  explicit Init_Detection_bbox_px_w(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bbox_px_h bbox_px_w(::rb_msgs::msg::Detection::_bbox_px_w_type arg)
  {
    msg_.bbox_px_w = std::move(arg);
    return Init_Detection_bbox_px_h(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_py
{
public:
  explicit Init_Detection_py(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_bbox_px_w py(::rb_msgs::msg::Detection::_py_type arg)
  {
    msg_.py = std::move(arg);
    return Init_Detection_bbox_px_w(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_px
{
public:
  explicit Init_Detection_px(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_py px(::rb_msgs::msg::Detection::_px_type arg)
  {
    msg_.px = std::move(arg);
    return Init_Detection_py(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_height
{
public:
  explicit Init_Detection_height(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_px height(::rb_msgs::msg::Detection::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Detection_px(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_width
{
public:
  explicit Init_Detection_width(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_height width(::rb_msgs::msg::Detection::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Detection_height(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_cy
{
public:
  explicit Init_Detection_cy(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_width cy(::rb_msgs::msg::Detection::_cy_type arg)
  {
    msg_.cy = std::move(arg);
    return Init_Detection_width(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_cx
{
public:
  explicit Init_Detection_cx(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_cy cx(::rb_msgs::msg::Detection::_cx_type arg)
  {
    msg_.cx = std::move(arg);
    return Init_Detection_cy(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_y_max
{
public:
  explicit Init_Detection_y_max(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_cx y_max(::rb_msgs::msg::Detection::_y_max_type arg)
  {
    msg_.y_max = std::move(arg);
    return Init_Detection_cx(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_x_max
{
public:
  explicit Init_Detection_x_max(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_y_max x_max(::rb_msgs::msg::Detection::_x_max_type arg)
  {
    msg_.x_max = std::move(arg);
    return Init_Detection_y_max(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_y_min
{
public:
  explicit Init_Detection_y_min(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_x_max y_min(::rb_msgs::msg::Detection::_y_min_type arg)
  {
    msg_.y_min = std::move(arg);
    return Init_Detection_x_max(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_x_min
{
public:
  explicit Init_Detection_x_min(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_y_min x_min(::rb_msgs::msg::Detection::_x_min_type arg)
  {
    msg_.x_min = std::move(arg);
    return Init_Detection_y_min(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_confidence
{
public:
  explicit Init_Detection_confidence(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_x_min confidence(::rb_msgs::msg::Detection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Detection_x_min(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_label
{
public:
  explicit Init_Detection_label(::rb_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_confidence label(::rb_msgs::msg::Detection::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Detection_confidence(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

class Init_Detection_stamp
{
public:
  Init_Detection_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_label stamp(::rb_msgs::msg::Detection::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Detection_label(msg_);
  }

private:
  ::rb_msgs::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rb_msgs::msg::Detection>()
{
  return rb_msgs::msg::builder::Init_Detection_stamp();
}

}  // namespace rb_msgs

#endif  // RB_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
