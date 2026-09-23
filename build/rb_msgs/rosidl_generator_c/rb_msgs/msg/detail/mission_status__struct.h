// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rb_msgs:msg/MissionStatus.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__MISSION_STATUS__STRUCT_H_
#define RB_MSGS__MSG__DETAIL__MISSION_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'mission'
// Member 'phase'
// Member 'detail'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MissionStatus in the package rb_msgs.
typedef struct rb_msgs__msg__MissionStatus
{
  std_msgs__msg__Header header;
  /// IDLE / PASS / SHOOT
  rosidl_runtime_c__String mission;
  /// 细分阶段名
  rosidl_runtime_c__String phase;
  /// 人类可读说明
  rosidl_runtime_c__String detail;
  /// 当前任务已用时
  float elapsed_s;
  /// 当前任务已得分估值
  int32_t score_estimate;
} rb_msgs__msg__MissionStatus;

// Struct for a sequence of rb_msgs__msg__MissionStatus.
typedef struct rb_msgs__msg__MissionStatus__Sequence
{
  rb_msgs__msg__MissionStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__msg__MissionStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RB_MSGS__MSG__DETAIL__MISSION_STATUS__STRUCT_H_
