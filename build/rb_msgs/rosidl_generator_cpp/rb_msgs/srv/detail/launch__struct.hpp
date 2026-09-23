// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rb_msgs:srv/Launch.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__SRV__DETAIL__LAUNCH__STRUCT_HPP_
#define RB_MSGS__SRV__DETAIL__LAUNCH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rb_msgs__srv__Launch_Request __attribute__((deprecated))
#else
# define DEPRECATED__rb_msgs__srv__Launch_Request __declspec(deprecated)
#endif

namespace rb_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Launch_Request_
{
  using Type = Launch_Request_<ContainerAllocator>;

  explicit Launch_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action = 0;
      this->speed = 0;
      this->angle = 0;
    }
  }

  explicit Launch_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action = 0;
      this->speed = 0;
      this->angle = 0;
    }
  }

  // field types and members
  using _action_type =
    uint8_t;
  _action_type action;
  using _speed_type =
    uint16_t;
  _speed_type speed;
  using _angle_type =
    uint16_t;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__action(
    const uint8_t & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__speed(
    const uint16_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__angle(
    const uint16_t & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rb_msgs::srv::Launch_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rb_msgs::srv::Launch_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::Launch_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::Launch_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rb_msgs__srv__Launch_Request
    std::shared_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rb_msgs__srv__Launch_Request
    std::shared_ptr<rb_msgs::srv::Launch_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Launch_Request_ & other) const
  {
    if (this->action != other.action) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const Launch_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Launch_Request_

// alias to use template instance with default allocator
using Launch_Request =
  rb_msgs::srv::Launch_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rb_msgs


#ifndef _WIN32
# define DEPRECATED__rb_msgs__srv__Launch_Response __attribute__((deprecated))
#else
# define DEPRECATED__rb_msgs__srv__Launch_Response __declspec(deprecated)
#endif

namespace rb_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Launch_Response_
{
  using Type = Launch_Response_<ContainerAllocator>;

  explicit Launch_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Launch_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rb_msgs::srv::Launch_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rb_msgs::srv::Launch_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::Launch_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::Launch_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rb_msgs__srv__Launch_Response
    std::shared_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rb_msgs__srv__Launch_Response
    std::shared_ptr<rb_msgs::srv::Launch_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Launch_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Launch_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Launch_Response_

// alias to use template instance with default allocator
using Launch_Response =
  rb_msgs::srv::Launch_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rb_msgs

namespace rb_msgs
{

namespace srv
{

struct Launch
{
  using Request = rb_msgs::srv::Launch_Request;
  using Response = rb_msgs::srv::Launch_Response;
};

}  // namespace srv

}  // namespace rb_msgs

#endif  // RB_MSGS__SRV__DETAIL__LAUNCH__STRUCT_HPP_
