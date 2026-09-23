// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rb_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
#define RB_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Detection in the package rb_msgs.
/**
  * 单个视觉检测目标（归一化图像坐标 + 方位角 + 粗略距离）
 */
typedef struct rb_msgs__msg__Detection
{
  builtin_interfaces__msg__Time stamp;
  /// 类别标签。约定取值：
  ///   ball_basketball / ball_volleyball / ball_unknown   —— 球
  ///   hoop          —— 迷你篮筐（含篮圈中心）
  ///   pass_rack     —— 传球训练架（rack_ring 为其顶部圆环）
  ///   rack_ring     —— 传球架顶部圆环（50 分目标）
  ///   pillar        —— 定位柱（规则提供的视觉基准）
  ///   obstacle      —— 干扰球
  rosidl_runtime_c__String label;
  /// 0..1
  float confidence;
  /// 归一化 bbox（0..1，左上角为原点）
  float x_min;
  float y_min;
  float x_max;
  float y_max;
  /// 中心 x（归一化）
  float cx;
  /// 中心 y（归一化）
  float cy;
  /// 归一化宽
  float width;
  /// 归一化高
  float height;
  /// 像素坐标（便于直接画图/复算）
  float px;
  float py;
  float bbox_px_w;
  float bbox_px_h;
  /// 几何估计
  /// 水平方位角，相对相机光轴，左正右负（由标定决定）
  float bearing_rad;
  /// 距离估计，未知填 NaN
  float distance_m;
  /// 目标物理直径（用于单目测距，未知填 NaN）
  float diameter_m;
} rb_msgs__msg__Detection;

// Struct for a sequence of rb_msgs__msg__Detection.
typedef struct rb_msgs__msg__Detection__Sequence
{
  rb_msgs__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RB_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
