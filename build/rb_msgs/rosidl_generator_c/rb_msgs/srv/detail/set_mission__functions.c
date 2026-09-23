// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rb_msgs:srv/SetMission.idl
// generated code does not contain a copyright notice
#include "rb_msgs/srv/detail/set_mission__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `mission`
#include "rosidl_runtime_c/string_functions.h"

bool
rb_msgs__srv__SetMission_Request__init(rb_msgs__srv__SetMission_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mission
  if (!rosidl_runtime_c__String__init(&msg->mission)) {
    rb_msgs__srv__SetMission_Request__fini(msg);
    return false;
  }
  return true;
}

void
rb_msgs__srv__SetMission_Request__fini(rb_msgs__srv__SetMission_Request * msg)
{
  if (!msg) {
    return;
  }
  // mission
  rosidl_runtime_c__String__fini(&msg->mission);
}

bool
rb_msgs__srv__SetMission_Request__are_equal(const rb_msgs__srv__SetMission_Request * lhs, const rb_msgs__srv__SetMission_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mission
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission), &(rhs->mission)))
  {
    return false;
  }
  return true;
}

bool
rb_msgs__srv__SetMission_Request__copy(
  const rb_msgs__srv__SetMission_Request * input,
  rb_msgs__srv__SetMission_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mission
  if (!rosidl_runtime_c__String__copy(
      &(input->mission), &(output->mission)))
  {
    return false;
  }
  return true;
}

rb_msgs__srv__SetMission_Request *
rb_msgs__srv__SetMission_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__srv__SetMission_Request * msg = (rb_msgs__srv__SetMission_Request *)allocator.allocate(sizeof(rb_msgs__srv__SetMission_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rb_msgs__srv__SetMission_Request));
  bool success = rb_msgs__srv__SetMission_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rb_msgs__srv__SetMission_Request__destroy(rb_msgs__srv__SetMission_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rb_msgs__srv__SetMission_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rb_msgs__srv__SetMission_Request__Sequence__init(rb_msgs__srv__SetMission_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__srv__SetMission_Request * data = NULL;

  if (size) {
    data = (rb_msgs__srv__SetMission_Request *)allocator.zero_allocate(size, sizeof(rb_msgs__srv__SetMission_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rb_msgs__srv__SetMission_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rb_msgs__srv__SetMission_Request__fini(&data[i - 1]);
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
rb_msgs__srv__SetMission_Request__Sequence__fini(rb_msgs__srv__SetMission_Request__Sequence * array)
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
      rb_msgs__srv__SetMission_Request__fini(&array->data[i]);
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

rb_msgs__srv__SetMission_Request__Sequence *
rb_msgs__srv__SetMission_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__srv__SetMission_Request__Sequence * array = (rb_msgs__srv__SetMission_Request__Sequence *)allocator.allocate(sizeof(rb_msgs__srv__SetMission_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rb_msgs__srv__SetMission_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rb_msgs__srv__SetMission_Request__Sequence__destroy(rb_msgs__srv__SetMission_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rb_msgs__srv__SetMission_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rb_msgs__srv__SetMission_Request__Sequence__are_equal(const rb_msgs__srv__SetMission_Request__Sequence * lhs, const rb_msgs__srv__SetMission_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rb_msgs__srv__SetMission_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rb_msgs__srv__SetMission_Request__Sequence__copy(
  const rb_msgs__srv__SetMission_Request__Sequence * input,
  rb_msgs__srv__SetMission_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rb_msgs__srv__SetMission_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rb_msgs__srv__SetMission_Request * data =
      (rb_msgs__srv__SetMission_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rb_msgs__srv__SetMission_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rb_msgs__srv__SetMission_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rb_msgs__srv__SetMission_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
rb_msgs__srv__SetMission_Response__init(rb_msgs__srv__SetMission_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    rb_msgs__srv__SetMission_Response__fini(msg);
    return false;
  }
  return true;
}

void
rb_msgs__srv__SetMission_Response__fini(rb_msgs__srv__SetMission_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
rb_msgs__srv__SetMission_Response__are_equal(const rb_msgs__srv__SetMission_Response * lhs, const rb_msgs__srv__SetMission_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
rb_msgs__srv__SetMission_Response__copy(
  const rb_msgs__srv__SetMission_Response * input,
  rb_msgs__srv__SetMission_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

rb_msgs__srv__SetMission_Response *
rb_msgs__srv__SetMission_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__srv__SetMission_Response * msg = (rb_msgs__srv__SetMission_Response *)allocator.allocate(sizeof(rb_msgs__srv__SetMission_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rb_msgs__srv__SetMission_Response));
  bool success = rb_msgs__srv__SetMission_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rb_msgs__srv__SetMission_Response__destroy(rb_msgs__srv__SetMission_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rb_msgs__srv__SetMission_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rb_msgs__srv__SetMission_Response__Sequence__init(rb_msgs__srv__SetMission_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__srv__SetMission_Response * data = NULL;

  if (size) {
    data = (rb_msgs__srv__SetMission_Response *)allocator.zero_allocate(size, sizeof(rb_msgs__srv__SetMission_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rb_msgs__srv__SetMission_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rb_msgs__srv__SetMission_Response__fini(&data[i - 1]);
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
rb_msgs__srv__SetMission_Response__Sequence__fini(rb_msgs__srv__SetMission_Response__Sequence * array)
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
      rb_msgs__srv__SetMission_Response__fini(&array->data[i]);
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

rb_msgs__srv__SetMission_Response__Sequence *
rb_msgs__srv__SetMission_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rb_msgs__srv__SetMission_Response__Sequence * array = (rb_msgs__srv__SetMission_Response__Sequence *)allocator.allocate(sizeof(rb_msgs__srv__SetMission_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rb_msgs__srv__SetMission_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rb_msgs__srv__SetMission_Response__Sequence__destroy(rb_msgs__srv__SetMission_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rb_msgs__srv__SetMission_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rb_msgs__srv__SetMission_Response__Sequence__are_equal(const rb_msgs__srv__SetMission_Response__Sequence * lhs, const rb_msgs__srv__SetMission_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rb_msgs__srv__SetMission_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rb_msgs__srv__SetMission_Response__Sequence__copy(
  const rb_msgs__srv__SetMission_Response__Sequence * input,
  rb_msgs__srv__SetMission_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rb_msgs__srv__SetMission_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rb_msgs__srv__SetMission_Response * data =
      (rb_msgs__srv__SetMission_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rb_msgs__srv__SetMission_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rb_msgs__srv__SetMission_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rb_msgs__srv__SetMission_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
