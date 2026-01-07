// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_protocol:msg/Heartbeat.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/heartbeat__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `node_name`
// Member `node_status`
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

bool
interface_protocol__msg__Heartbeat__init(interface_protocol__msg__Heartbeat * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    interface_protocol__msg__Heartbeat__fini(msg);
    return false;
  }
  // node_name
  if (!rosidl_runtime_c__String__init(&msg->node_name)) {
    interface_protocol__msg__Heartbeat__fini(msg);
    return false;
  }
  // startup_timestamp
  // node_status
  if (!rosidl_runtime_c__String__init(&msg->node_status)) {
    interface_protocol__msg__Heartbeat__fini(msg);
    return false;
  }
  // error_code
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    interface_protocol__msg__Heartbeat__fini(msg);
    return false;
  }
  return true;
}

void
interface_protocol__msg__Heartbeat__fini(interface_protocol__msg__Heartbeat * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // node_name
  rosidl_runtime_c__String__fini(&msg->node_name);
  // startup_timestamp
  // node_status
  rosidl_runtime_c__String__fini(&msg->node_status);
  // error_code
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
}

bool
interface_protocol__msg__Heartbeat__are_equal(const interface_protocol__msg__Heartbeat * lhs, const interface_protocol__msg__Heartbeat * rhs)
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
  // node_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->node_name), &(rhs->node_name)))
  {
    return false;
  }
  // startup_timestamp
  if (lhs->startup_timestamp != rhs->startup_timestamp) {
    return false;
  }
  // node_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->node_status), &(rhs->node_status)))
  {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // error_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->error_message), &(rhs->error_message)))
  {
    return false;
  }
  return true;
}

bool
interface_protocol__msg__Heartbeat__copy(
  const interface_protocol__msg__Heartbeat * input,
  interface_protocol__msg__Heartbeat * output)
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
  // node_name
  if (!rosidl_runtime_c__String__copy(
      &(input->node_name), &(output->node_name)))
  {
    return false;
  }
  // startup_timestamp
  output->startup_timestamp = input->startup_timestamp;
  // node_status
  if (!rosidl_runtime_c__String__copy(
      &(input->node_status), &(output->node_status)))
  {
    return false;
  }
  // error_code
  output->error_code = input->error_code;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  return true;
}

interface_protocol__msg__Heartbeat *
interface_protocol__msg__Heartbeat__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__Heartbeat * msg = (interface_protocol__msg__Heartbeat *)allocator.allocate(sizeof(interface_protocol__msg__Heartbeat), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_protocol__msg__Heartbeat));
  bool success = interface_protocol__msg__Heartbeat__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_protocol__msg__Heartbeat__destroy(interface_protocol__msg__Heartbeat * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_protocol__msg__Heartbeat__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_protocol__msg__Heartbeat__Sequence__init(interface_protocol__msg__Heartbeat__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__Heartbeat * data = NULL;

  if (size) {
    data = (interface_protocol__msg__Heartbeat *)allocator.zero_allocate(size, sizeof(interface_protocol__msg__Heartbeat), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_protocol__msg__Heartbeat__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_protocol__msg__Heartbeat__fini(&data[i - 1]);
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
interface_protocol__msg__Heartbeat__Sequence__fini(interface_protocol__msg__Heartbeat__Sequence * array)
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
      interface_protocol__msg__Heartbeat__fini(&array->data[i]);
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

interface_protocol__msg__Heartbeat__Sequence *
interface_protocol__msg__Heartbeat__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__Heartbeat__Sequence * array = (interface_protocol__msg__Heartbeat__Sequence *)allocator.allocate(sizeof(interface_protocol__msg__Heartbeat__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_protocol__msg__Heartbeat__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_protocol__msg__Heartbeat__Sequence__destroy(interface_protocol__msg__Heartbeat__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_protocol__msg__Heartbeat__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_protocol__msg__Heartbeat__Sequence__are_equal(const interface_protocol__msg__Heartbeat__Sequence * lhs, const interface_protocol__msg__Heartbeat__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_protocol__msg__Heartbeat__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_protocol__msg__Heartbeat__Sequence__copy(
  const interface_protocol__msg__Heartbeat__Sequence * input,
  interface_protocol__msg__Heartbeat__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_protocol__msg__Heartbeat);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_protocol__msg__Heartbeat * data =
      (interface_protocol__msg__Heartbeat *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_protocol__msg__Heartbeat__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_protocol__msg__Heartbeat__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_protocol__msg__Heartbeat__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
