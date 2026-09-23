// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rb_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_HPP_
#define RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'detections'
#include "rb_msgs/msg/detail/detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rb_msgs__msg__DetectionArray __attribute__((deprecated))
#else
# define DEPRECATED__rb_msgs__msg__DetectionArray __declspec(deprecated)
#endif

namespace rb_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectionArray_
{
  using Type = DetectionArray_<ContainerAllocator>;

  explicit DetectionArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id_seq = 0ul;
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->cx_cam = 0.0f;
      this->cy_cam = 0.0f;
      this->img_width = 0ul;
      this->img_height = 0ul;
    }
  }

  explicit DetectionArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id_seq = 0ul;
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->cx_cam = 0.0f;
      this->cy_cam = 0.0f;
      this->img_width = 0ul;
      this->img_height = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _frame_id_seq_type =
    uint32_t;
  _frame_id_seq_type frame_id_seq;
  using _fx_type =
    float;
  _fx_type fx;
  using _fy_type =
    float;
  _fy_type fy;
  using _cx_cam_type =
    float;
  _cx_cam_type cx_cam;
  using _cy_cam_type =
    float;
  _cy_cam_type cy_cam;
  using _img_width_type =
    uint32_t;
  _img_width_type img_width;
  using _img_height_type =
    uint32_t;
  _img_height_type img_height;
  using _detections_type =
    std::vector<rb_msgs::msg::Detection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rb_msgs::msg::Detection_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__frame_id_seq(
    const uint32_t & _arg)
  {
    this->frame_id_seq = _arg;
    return *this;
  }
  Type & set__fx(
    const float & _arg)
  {
    this->fx = _arg;
    return *this;
  }
  Type & set__fy(
    const float & _arg)
  {
    this->fy = _arg;
    return *this;
  }
  Type & set__cx_cam(
    const float & _arg)
  {
    this->cx_cam = _arg;
    return *this;
  }
  Type & set__cy_cam(
    const float & _arg)
  {
    this->cy_cam = _arg;
    return *this;
  }
  Type & set__img_width(
    const uint32_t & _arg)
  {
    this->img_width = _arg;
    return *this;
  }
  Type & set__img_height(
    const uint32_t & _arg)
  {
    this->img_height = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<rb_msgs::msg::Detection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rb_msgs::msg::Detection_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rb_msgs::msg::DetectionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const rb_msgs::msg::DetectionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rb_msgs::msg::DetectionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rb_msgs::msg::DetectionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rb_msgs__msg__DetectionArray
    std::shared_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rb_msgs__msg__DetectionArray
    std::shared_ptr<rb_msgs::msg::DetectionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectionArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->frame_id_seq != other.frame_id_seq) {
      return false;
    }
    if (this->fx != other.fx) {
      return false;
    }
    if (this->fy != other.fy) {
      return false;
    }
    if (this->cx_cam != other.cx_cam) {
      return false;
    }
    if (this->cy_cam != other.cy_cam) {
      return false;
    }
    if (this->img_width != other.img_width) {
      return false;
    }
    if (this->img_height != other.img_height) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectionArray_

// alias to use template instance with default allocator
using DetectionArray =
  rb_msgs::msg::DetectionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rb_msgs

#endif  // RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_HPP_
