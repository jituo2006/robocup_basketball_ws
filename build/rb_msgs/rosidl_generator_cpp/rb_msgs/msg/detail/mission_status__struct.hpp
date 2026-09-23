// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rb_msgs:msg/MissionStatus.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__MISSION_STATUS__STRUCT_HPP_
#define RB_MSGS__MSG__DETAIL__MISSION_STATUS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__rb_msgs__msg__MissionStatus __attribute__((deprecated))
#else
# define DEPRECATED__rb_msgs__msg__MissionStatus __declspec(deprecated)
#endif

namespace rb_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MissionStatus_
{
  using Type = MissionStatus_<ContainerAllocator>;

  explicit MissionStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = "";
      this->phase = "";
      this->detail = "";
      this->elapsed_s = 0.0f;
      this->score_estimate = 0l;
    }
  }

  explicit MissionStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    mission(_alloc),
    phase(_alloc),
    detail(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = "";
      this->phase = "";
      this->detail = "";
      this->elapsed_s = 0.0f;
      this->score_estimate = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mission_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type mission;
  using _phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _phase_type phase;
  using _detail_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _detail_type detail;
  using _elapsed_s_type =
    float;
  _elapsed_s_type elapsed_s;
  using _score_estimate_type =
    int32_t;
  _score_estimate_type score_estimate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mission(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission = _arg;
    return *this;
  }
  Type & set__phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__detail(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->detail = _arg;
    return *this;
  }
  Type & set__elapsed_s(
    const float & _arg)
  {
    this->elapsed_s = _arg;
    return *this;
  }
  Type & set__score_estimate(
    const int32_t & _arg)
  {
    this->score_estimate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rb_msgs::msg::MissionStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rb_msgs::msg::MissionStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rb_msgs::msg::MissionStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rb_msgs::msg::MissionStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rb_msgs__msg__MissionStatus
    std::shared_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rb_msgs__msg__MissionStatus
    std::shared_ptr<rb_msgs::msg::MissionStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MissionStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mission != other.mission) {
      return false;
    }
    if (this->phase != other.phase) {
      return false;
    }
    if (this->detail != other.detail) {
      return false;
    }
    if (this->elapsed_s != other.elapsed_s) {
      return false;
    }
    if (this->score_estimate != other.score_estimate) {
      return false;
    }
    return true;
  }
  bool operator!=(const MissionStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MissionStatus_

// alias to use template instance with default allocator
using MissionStatus =
  rb_msgs::msg::MissionStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rb_msgs

#endif  // RB_MSGS__MSG__DETAIL__MISSION_STATUS__STRUCT_HPP_
