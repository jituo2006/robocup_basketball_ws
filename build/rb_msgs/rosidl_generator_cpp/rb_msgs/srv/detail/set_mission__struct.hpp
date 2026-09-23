// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rb_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_HPP_
#define RB_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rb_msgs__srv__SetMission_Request __attribute__((deprecated))
#else
# define DEPRECATED__rb_msgs__srv__SetMission_Request __declspec(deprecated)
#endif

namespace rb_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMission_Request_
{
  using Type = SetMission_Request_<ContainerAllocator>;

  explicit SetMission_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = "";
    }
  }

  explicit SetMission_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mission(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission = "";
    }
  }

  // field types and members
  using _mission_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_type mission;

  // setters for named parameter idiom
  Type & set__mission(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rb_msgs::srv::SetMission_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rb_msgs::srv::SetMission_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::SetMission_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::SetMission_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rb_msgs__srv__SetMission_Request
    std::shared_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rb_msgs__srv__SetMission_Request
    std::shared_ptr<rb_msgs::srv::SetMission_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMission_Request_ & other) const
  {
    if (this->mission != other.mission) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMission_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMission_Request_

// alias to use template instance with default allocator
using SetMission_Request =
  rb_msgs::srv::SetMission_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rb_msgs


#ifndef _WIN32
# define DEPRECATED__rb_msgs__srv__SetMission_Response __attribute__((deprecated))
#else
# define DEPRECATED__rb_msgs__srv__SetMission_Response __declspec(deprecated)
#endif

namespace rb_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMission_Response_
{
  using Type = SetMission_Response_<ContainerAllocator>;

  explicit SetMission_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->message = "";
    }
  }

  explicit SetMission_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->message = "";
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
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
    rb_msgs::srv::SetMission_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rb_msgs::srv::SetMission_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::SetMission_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rb_msgs::srv::SetMission_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rb_msgs__srv__SetMission_Response
    std::shared_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rb_msgs__srv__SetMission_Response
    std::shared_ptr<rb_msgs::srv::SetMission_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMission_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMission_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMission_Response_

// alias to use template instance with default allocator
using SetMission_Response =
  rb_msgs::srv::SetMission_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rb_msgs

namespace rb_msgs
{

namespace srv
{

struct SetMission
{
  using Request = rb_msgs::srv::SetMission_Request;
  using Response = rb_msgs::srv::SetMission_Response;
};

}  // namespace srv

}  // namespace rb_msgs

#endif  // RB_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_HPP_
