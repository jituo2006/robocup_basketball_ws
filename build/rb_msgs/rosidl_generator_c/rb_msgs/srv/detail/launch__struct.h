// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rb_msgs:srv/Launch.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__SRV__DETAIL__LAUNCH__STRUCT_H_
#define RB_MSGS__SRV__DETAIL__LAUNCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Launch in the package rb_msgs.
typedef struct rb_msgs__srv__Launch_Request
{
  uint8_t action;
  /// 转速 rpm；0 表示沿用节点默认值
  uint16_t speed;
  /// 角度；0 表示沿用节点默认值
  uint16_t angle;
} rb_msgs__srv__Launch_Request;

// Struct for a sequence of rb_msgs__srv__Launch_Request.
typedef struct rb_msgs__srv__Launch_Request__Sequence
{
  rb_msgs__srv__Launch_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__srv__Launch_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Launch in the package rb_msgs.
typedef struct rb_msgs__srv__Launch_Response
{
  bool success;
  rosidl_runtime_c__String message;
} rb_msgs__srv__Launch_Response;

// Struct for a sequence of rb_msgs__srv__Launch_Response.
typedef struct rb_msgs__srv__Launch_Response__Sequence
{
  rb_msgs__srv__Launch_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__srv__Launch_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RB_MSGS__SRV__DETAIL__LAUNCH__STRUCT_H_
