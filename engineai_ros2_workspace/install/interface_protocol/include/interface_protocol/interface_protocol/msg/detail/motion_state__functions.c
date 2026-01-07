// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_protocol:msg/MotionState.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/motion_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `current_motion_task`
#include "rosidl_runtime_c/string_functions.h"

bool
interface_protocol__msg__MotionState__init(interface_protocol__msg__MotionState * msg)
{
  if (!msg) {
    return false;
  }
  // current_motion_task
  if (!rosidl_runtime_c__String__init(&msg->current_motion_task)) {
    interface_protocol__msg__MotionState__fini(msg);
    return false;
  }
  return true;
}

void
interface_protocol__msg__MotionState__fini(interface_protocol__msg__MotionState * msg)
{
  if (!msg) {
    return;
  }
  // current_motion_task
  rosidl_runtime_c__String__fini(&msg->current_motion_task);
}

bool
interface_protocol__msg__MotionState__are_equal(const interface_protocol__msg__MotionState * lhs, const interface_protocol__msg__MotionState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_motion_task
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_motion_task), &(rhs->current_motion_task)))
  {
    return false;
  }
  return true;
}

bool
interface_protocol__msg__MotionState__copy(
  const interface_protocol__msg__MotionState * input,
  interface_protocol__msg__MotionState * output)
{
  if (!input || !output) {
    return false;
  }
  // current_motion_task
  if (!rosidl_runtime_c__String__copy(
      &(input->current_motion_task), &(output->current_motion_task)))
  {
    return false;
  }
  return true;
}

interface_protocol__msg__MotionState *
interface_protocol__msg__MotionState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotionState * msg = (interface_protocol__msg__MotionState *)allocator.allocate(sizeof(interface_protocol__msg__MotionState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_protocol__msg__MotionState));
  bool success = interface_protocol__msg__MotionState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_protocol__msg__MotionState__destroy(interface_protocol__msg__MotionState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_protocol__msg__MotionState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_protocol__msg__MotionState__Sequence__init(interface_protocol__msg__MotionState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotionState * data = NULL;

  if (size) {
    data = (interface_protocol__msg__MotionState *)allocator.zero_allocate(size, sizeof(interface_protocol__msg__MotionState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_protocol__msg__MotionState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_protocol__msg__MotionState__fini(&data[i - 1]);
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
interface_protocol__msg__MotionState__Sequence__fini(interface_protocol__msg__MotionState__Sequence * array)
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
      interface_protocol__msg__MotionState__fini(&array->data[i]);
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

interface_protocol__msg__MotionState__Sequence *
interface_protocol__msg__MotionState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotionState__Sequence * array = (interface_protocol__msg__MotionState__Sequence *)allocator.allocate(sizeof(interface_protocol__msg__MotionState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_protocol__msg__MotionState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_protocol__msg__MotionState__Sequence__destroy(interface_protocol__msg__MotionState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_protocol__msg__MotionState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_protocol__msg__MotionState__Sequence__are_equal(const interface_protocol__msg__MotionState__Sequence * lhs, const interface_protocol__msg__MotionState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_protocol__msg__MotionState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_protocol__msg__MotionState__Sequence__copy(
  const interface_protocol__msg__MotionState__Sequence * input,
  interface_protocol__msg__MotionState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_protocol__msg__MotionState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_protocol__msg__MotionState * data =
      (interface_protocol__msg__MotionState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_protocol__msg__MotionState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_protocol__msg__MotionState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_protocol__msg__MotionState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
