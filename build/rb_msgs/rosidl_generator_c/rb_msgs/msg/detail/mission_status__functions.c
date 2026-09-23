// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rb_msgs:msg/MissionStatus.idl
// generated code does not contain a copyright notice
#include "rb_msgs/msg/detail/mission_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `mission`
// Member `phase`
// Member `detail`
#include "rosidl_runtime_c/string_functions.h"

bool
rb_msgs__msg__MissionStatus__init(rb_msgs__msg__MissionStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rb_msgs__msg__MissionStatus__fini(msg);
    return false;
  }
  // mission
  if (!rosidl_runtime_c__String__init(&msg->mission)) {
    rb_msgs__msg__MissionStatus__fini(msg);
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__init(&msg->phase)) {
    rb_msgs__msg__MissionStatus__fini(msg);
    return false;
  }
  // detail
  if (!rosidl_runtime_c__String__init(&msg->detail)) {
    rb_msgs__msg__MissionStatus__fini(msg);
    return false;
  }
  // elapsed_s
  // score_estimate
  return true;
}

void
rb_msgs__msg__MissionStatus__fini(rb_msgs__msg__MissionStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // mission
  rosidl_runtime_c__String__fini(&msg->mission);
  // phase
  rosidl_runtime_c__String__fini(&msg->phase);
  // detail
  rosidl_runtime_c__String__fini(&msg->detail);
  // elapsed_s
  // score_estimate
}

bool
rb_msgs__msg__MissionStatus__are_equal(const rb_msgs__msg__MissionStatus * lhs, const rb_msgs__msg__MissionStatus * rhs)
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
  // mission
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission), &(rhs->mission)))
  {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->phase), &(rhs->phase)))
  {
    return false;
  }
  // detail
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->detail), &(rhs->detail)))
  {
    return false;
  }
  // elapsed_s
  if (lhs->elapsed_s != rhs->elapsed_s) {
    return false;
  }
  // score_estimate
  if (lhs->score_estimate != rhs->score_estimate) {
    return false;
  }
  return true;
}

bool
rb_msgs__msg__MissionStatus__copy(
  const rb_msgs__msg__MissionStatus * input,
  rb_msgs__msg__MissionStatus * output)
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
  // mission
  if (!rosidl_runtime_c__String__copy(
      &(input->mission), &(output->mission)))
  {
    return false;
  }
  // phase
  if (!rosidl_runtime_c__String__copy(
      &(input->phase), &(output->phase)))
  {
    return false;
  }
  // detail
  if (!rosidl_runtime_c__String__copy(
      &(input->detail), &(output->detail)))
  {
    return false;
  }
  // elapsed_s
  output->elapsed_s = input->elapsed_s;
  // score_estimate
  output->score_estimate = input->score_estimate;
  return true;
}

rb_msgs__msg__MissionStatus *
rb_msgs__msg__MissionStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__msg__MissionStatus * msg = (rb_msgs__msg__MissionStatus *)allocator.allocate(sizeof(rb_msgs__msg__MissionStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rb_msgs__msg__MissionStatus));
  bool success = rb_msgs__msg__MissionStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rb_msgs__msg__MissionStatus__destroy(rb_msgs__msg__MissionStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rb_msgs__msg__MissionStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rb_msgs__msg__MissionStatus__Sequence__init(rb_msgs__msg__MissionStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__msg__MissionStatus * data = NULL;

  if (size) {
    data = (rb_msgs__msg__MissionStatus *)allocator.zero_allocate(size, sizeof(rb_msgs__msg__MissionStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rb_msgs__msg__MissionStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rb_msgs__msg__MissionStatus__fini(&data[i - 1]);
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
rb_msgs__msg__MissionStatus__Sequence__fini(rb_msgs__msg__MissionStatus__Sequence * array)
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
      rb_msgs__msg__MissionStatus__fini(&array->data[i]);
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

rb_msgs__msg__MissionStatus__Sequence *
rb_msgs__msg__MissionStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__msg__MissionStatus__Sequence * array = (rb_msgs__msg__MissionStatus__Sequence *)allocator.allocate(sizeof(rb_msgs__msg__MissionStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rb_msgs__msg__MissionStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rb_msgs__msg__MissionStatus__Sequence__destroy(rb_msgs__msg__MissionStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rb_msgs__msg__MissionStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rb_msgs__msg__MissionStatus__Sequence__are_equal(const rb_msgs__msg__MissionStatus__Sequence * lhs, const rb_msgs__msg__MissionStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rb_msgs__msg__MissionStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rb_msgs__msg__MissionStatus__Sequence__copy(
  const rb_msgs__msg__MissionStatus__Sequence * input,
  rb_msgs__msg__MissionStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rb_msgs__msg__MissionStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rb_msgs__msg__MissionStatus * data =
      (rb_msgs__msg__MissionStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rb_msgs__msg__MissionStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rb_msgs__msg__MissionStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rb_msgs__msg__MissionStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
