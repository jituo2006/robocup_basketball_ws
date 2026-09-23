// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rb_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
#define RB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_

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
# define DEPRECATED__rb_msgs__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__rb_msgs__msg__RobotState __declspec(deprecated)
#endif

namespace rb_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = "";
      this->has_ball = false;
      this->ball_count = 0;
      this->ball_type = "";
      this->in_pass_zone = false;
      this->in_shoot_zone_outside = false;
      this->estop = false;
      this->localization_ok = false;
      this->launcher_ok = false;
      this->perception_ok = false;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    phase(_alloc),
    ball_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = "";
      this->has_ball = false;
      this->ball_count = 0;
      this->ball_type = "";
      this->in_pass_zone = false;
      this->in_shoot_zone_outside = false;
      this->estop = false;
      this->localization_ok = false;
      this->launcher_ok = false;
      this->perception_ok = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _phase_type phase;
  using _has_ball_type =
    bool;
  _has_ball_type has_ball;
  using _ball_count_type =
    uint8_t;
  _ball_count_type ball_count;
  using _ball_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ball_type_type ball_type;
  using _in_pass_zone_type =
    bool;
  _in_pass_zone_type in_pass_zone;
  using _in_shoot_zone_outside_type =
    bool;
  _in_shoot_zone_outside_type in_shoot_zone_outside;
  using _estop_type =
    bool;
  _estop_type estop;
  using _localization_ok_type =
    bool;
  _localization_ok_type localization_ok;
  using _launcher_ok_type =
    bool;
  _launcher_ok_type launcher_ok;
  using _perception_ok_type =
    bool;
  _perception_ok_type perception_ok;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__has_ball(
    const bool & _arg)
  {
    this->has_ball = _arg;
    return *this;
  }
  Type & set__ball_count(
    const uint8_t & _arg)
  {
    this->ball_count = _arg;
    return *this;
  }
  Type & set__ball_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ball_type = _arg;
    return *this;
  }
  Type & set__in_pass_zone(
    const bool & _arg)
  {
    this->in_pass_zone = _arg;
    return *this;
  }
  Type & set__in_shoot_zone_outside(
    const bool & _arg)
  {
    this->in_shoot_zone_outside = _arg;
    return *this;
  }
  Type & set__estop(
    const bool & _arg)
  {
    this->estop = _arg;
    return *this;
  }
  Type & set__localization_ok(
    const bool & _arg)
  {
    this->localization_ok = _arg;
    return *this;
  }
  Type & set__launcher_ok(
    const bool & _arg)
  {
    this->launcher_ok = _arg;
    return *this;
  }
  Type & set__perception_ok(
    const bool & _arg)
  {
    this->perception_ok = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rb_msgs::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const rb_msgs::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rb_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rb_msgs::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rb_msgs::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rb_msgs::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rb_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rb_msgs::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rb_msgs__msg__RobotState
    std::shared_ptr<rb_msgs::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rb_msgs__msg__RobotState
    std::shared_ptr<rb_msgs::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->phase != other.phase) {
      return false;
    }
    if (this->has_ball != other.has_ball) {
      return false;
    }
    if (this->ball_count != other.ball_count) {
      return false;
    }
    if (this->ball_type != other.ball_type) {
      return false;
    }
    if (this->in_pass_zone != other.in_pass_zone) {
      return false;
    }
    if (this->in_shoot_zone_outside != other.in_shoot_zone_outside) {
      return false;
    }
    if (this->estop != other.estop) {
      return false;
    }
    if (this->localization_ok != other.localization_ok) {
      return false;
    }
    if (this->launcher_ok != other.launcher_ok) {
      return false;
    }
    if (this->perception_ok != other.perception_ok) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  rb_msgs::msg::RobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rb_msgs

#endif  // RB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
