// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rb_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
#define RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_

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
// Member 'detections'
#include "rb_msgs/msg/detail/detection__struct.h"

/// Struct defined in msg/DetectionArray in the package rb_msgs.
typedef struct rb_msgs__msg__DetectionArray
{
  std_msgs__msg__Header header;
  uint32_t frame_id_seq;
  /// 相机内参（来自标定；未标定时为 0）
  float fx;
  float fy;
  float cx_cam;
  float cy_cam;
  uint32_t img_width;
  uint32_t img_height;
  rb_msgs__msg__Detection__Sequence detections;
} rb_msgs__msg__DetectionArray;

// Struct for a sequence of rb_msgs__msg__DetectionArray.
typedef struct rb_msgs__msg__DetectionArray__Sequence
{
  rb_msgs__msg__DetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rb_msgs__msg__DetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RB_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
