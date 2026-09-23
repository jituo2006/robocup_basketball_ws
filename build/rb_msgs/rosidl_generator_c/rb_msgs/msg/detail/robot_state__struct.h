// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rb_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define RB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

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
// Member 'phase'
// Member 'ball_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotState in the package rb_msgs.
typedef struct rb_msgs__msg__RobotState
{
  std_msgs__msg__Header header;
  /// 任务阶段（与 rb_mission 的状态机一致）
  rosidl_runtime_c__String phase;
  /// 是否持球
  bool has_ball;
  /// 持球数量（规则：单次持球 >2 颗违规）
  uint8_t ball_count;
  /// ball_basketball / ball_volleyball / none
  rosidl_runtime_c__String ball_type;
  /// 是否在传球区内（决定传球得分 10 分 / 5 分）
  bool in_pass_zone;
  /// 是否在投篮边界线外（决定投篮 10 分 / 5 分）
  bool in_shoot_zone_outside;
  /// 急停/自主拍停是否触发
  bool estop;
  /// 定位是否可用
  bool localization_ok;
  /// 发射机构串口是否正常
  bool launcher_ok;
  /// 视觉是否正常
  bool perception_ok;
} rb_msgs__msg__RobotState;

// Struct for a sequence of rb_msgs__msg__RobotState.
typedef struct rb_msgs__msg__RobotState__Sequence
{
  rb_msgs__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RB_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
