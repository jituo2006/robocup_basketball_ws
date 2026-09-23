// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rb_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_H_
#define RB_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mission'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetMission in the package rb_msgs.
typedef struct rb_msgs__srv__SetMission_Request
{
  rosidl_runtime_c__String mission;
} rb_msgs__srv__SetMission_Request;

// Struct for a sequence of rb_msgs__srv__SetMission_Request.
typedef struct rb_msgs__srv__SetMission_Request__Sequence
{
  rb_msgs__srv__SetMission_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__srv__SetMission_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetMission in the package rb_msgs.
typedef struct rb_msgs__srv__SetMission_Response
{
  bool accepted;
  rosidl_runtime_c__String message;
} rb_msgs__srv__SetMission_Response;

// Struct for a sequence of rb_msgs__srv__SetMission_Response.
typedef struct rb_msgs__srv__SetMission_Response__Sequence
{
  rb_msgs__srv__SetMission_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__srv__SetMission_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RB_MSGS__SRV__DETAIL__SET_MISSION__STRUCT_H_
