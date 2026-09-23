// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rb_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "rb_msgs/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `phase`
// Member `ball_type`
#include "rosidl_runtime_c/string_functions.h"

bool
rb_msgs__msg__RobotState__init(rb_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rb_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__init(&msg->phase)) {
    rb_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // has_ball
  // ball_count
  // ball_type
  if (!rosidl_runtime_c__String__init(&msg->ball_type)) {
    rb_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // in_pass_zone
  // in_shoot_zone_outside
  // estop
  // localization_ok
  // launcher_ok
  // perception_ok
  return true;
}

void
rb_msgs__msg__RobotState__fini(rb_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // phase
  rosidl_runtime_c__String__fini(&msg->phase);
  // has_ball
  // ball_count
  // ball_type
  rosidl_runtime_c__String__fini(&msg->ball_type);
  // in_pass_zone
  // in_shoot_zone_outside
  // estop
  // localization_ok
  // launcher_ok
  // perception_ok
}

bool
rb_msgs__msg__RobotState__are_equal(const rb_msgs__msg__RobotState * lhs, const rb_msgs__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->phase), &(rhs->phase)))
  {
    return false;
  }
  // has_ball
  if (lhs->has_ball != rhs->has_ball) {
    return false;
  }
  // ball_count
  if (lhs->ball_count != rhs->ball_count) {
    return false;
  }
  // ball_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->ball_type), &(rhs->ball_type)))
  {
    return false;
  }
  // in_pass_zone
  if (lhs->in_pass_zone != rhs->in_pass_zone) {
    return false;
  }
  // in_shoot_zone_outside
  if (lhs->in_shoot_zone_outside != rhs->in_shoot_zone_outside) {
    return false;
  }
  // estop
  if (lhs->estop != rhs->estop) {
    return false;
  }
  // localization_ok
  if (lhs->localization_ok != rhs->localization_ok) {
    return false;
  }
  // launcher_ok
  if (lhs->launcher_ok != rhs->launcher_ok) {
    return false;
  }
  // perception_ok
  if (lhs->perception_ok != rhs->perception_ok) {
    return false;
  }
  return true;
}

bool
rb_msgs__msg__RobotState__copy(
  const rb_msgs__msg__RobotState * input,
  rb_msgs__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__copy(
      &(input->phase), &(output->phase)))
  {
    return false;
  }
  // has_ball
  output->has_ball = input->has_ball;
  // ball_count
  output->ball_count = input->ball_count;
  // ball_type
  if (!rosidl_runtime_c__String__copy(
      &(input->ball_type), &(output->ball_type)))
  {
    return false;
  }
  // in_pass_zone
  output->in_pass_zone = input->in_pass_zone;
  // in_shoot_zone_outside
  output->in_shoot_zone_outside = input->in_shoot_zone_outside;
  // estop
  output->estop = input->estop;
  // localization_ok
  output->localization_ok = input->localization_ok;
  // launcher_ok
  output->launcher_ok = input->launcher_ok;
  // perception_ok
  output->perception_ok = input->perception_ok;
  return true;
}

rb_msgs__msg__RobotState *
rb_msgs__msg__RobotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__msg__RobotState * msg = (rb_msgs__msg__RobotState *)allocator.allocate(sizeof(rb_msgs__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rb_msgs__msg__RobotState));
  bool success = rb_msgs__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rb_msgs__msg__RobotState__destroy(rb_msgs__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rb_msgs__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rb_msgs__msg__RobotState__Sequence__init(rb_msgs__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__msg__RobotState * data = NULL;

  if (size) {
    data = (rb_msgs__msg__RobotState *)allocator.zero_allocate(size, sizeof(rb_msgs__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rb_msgs__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rb_msgs__msg__RobotState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rb_msgs__msg__RobotState__Sequence__fini(rb_msgs__msg__RobotState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rb_msgs__msg__RobotState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rb_msgs__msg__RobotState__Sequence *
rb_msgs__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__msg__RobotState__Sequence * array = (rb_msgs__msg__RobotState__Sequence *)allocator.allocate(sizeof(rb_msgs__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rb_msgs__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rb_msgs__msg__RobotState__Sequence__destroy(rb_msgs__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rb_msgs__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rb_msgs__msg__RobotState__Sequence__are_equal(const rb_msgs__msg__RobotState__Sequence * lhs, const rb_msgs__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rb_msgs__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rb_msgs__msg__RobotState__Sequence__copy(
  const rb_msgs__msg__RobotState__Sequence * input,
  rb_msgs__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rb_msgs__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rb_msgs__msg__RobotState * data =
      (rb_msgs__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rb_msgs__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rb_msgs__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rb_msgs__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
