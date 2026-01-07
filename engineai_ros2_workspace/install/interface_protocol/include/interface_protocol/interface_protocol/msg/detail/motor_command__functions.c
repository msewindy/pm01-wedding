// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_protocol:msg/MotorCommand.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/motor_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
// Member `velocity`
// Member `feed_forward_torque`
// Member `torque`
// Member `stiffness`
// Member `damping`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
interface_protocol__msg__MotorCommand__init(interface_protocol__msg__MotorCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    interface_protocol__msg__MotorCommand__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    interface_protocol__msg__MotorCommand__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    interface_protocol__msg__MotorCommand__fini(msg);
    return false;
  }
  // feed_forward_torque
  if (!rosidl_runtime_c__double__Sequence__init(&msg->feed_forward_torque, 0)) {
    interface_protocol__msg__MotorCommand__fini(msg);
    return false;
  }
  // torque
  if (!rosidl_runtime_c__double__Sequence__init(&msg->torque, 0)) {
    interface_protocol__msg__MotorCommand__fini(msg);
    return false;
  }
  // stiffness
  if (!rosidl_runtime_c__double__Sequence__init(&msg->stiffness, 0)) {
    interface_protocol__msg__MotorCommand__fini(msg);
    return false;
  }
  // damping
  if (!rosidl_runtime_c__double__Sequence__init(&msg->damping, 0)) {
    interface_protocol__msg__MotorCommand__fini(msg);
    return false;
  }
  return true;
}

void
interface_protocol__msg__MotorCommand__fini(interface_protocol__msg__MotorCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  rosidl_runtime_c__double__Sequence__fini(&msg->position);
  // velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->velocity);
  // feed_forward_torque
  rosidl_runtime_c__double__Sequence__fini(&msg->feed_forward_torque);
  // torque
  rosidl_runtime_c__double__Sequence__fini(&msg->torque);
  // stiffness
  rosidl_runtime_c__double__Sequence__fini(&msg->stiffness);
  // damping
  rosidl_runtime_c__double__Sequence__fini(&msg->damping);
}

bool
interface_protocol__msg__MotorCommand__are_equal(const interface_protocol__msg__MotorCommand * lhs, const interface_protocol__msg__MotorCommand * rhs)
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
  // position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // feed_forward_torque
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->feed_forward_torque), &(rhs->feed_forward_torque)))
  {
    return false;
  }
  // torque
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->torque), &(rhs->torque)))
  {
    return false;
  }
  // stiffness
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->stiffness), &(rhs->stiffness)))
  {
    return false;
  }
  // damping
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->damping), &(rhs->damping)))
  {
    return false;
  }
  return true;
}

bool
interface_protocol__msg__MotorCommand__copy(
  const interface_protocol__msg__MotorCommand * input,
  interface_protocol__msg__MotorCommand * output)
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
  // position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // feed_forward_torque
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->feed_forward_torque), &(output->feed_forward_torque)))
  {
    return false;
  }
  // torque
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->torque), &(output->torque)))
  {
    return false;
  }
  // stiffness
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->stiffness), &(output->stiffness)))
  {
    return false;
  }
  // damping
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->damping), &(output->damping)))
  {
    return false;
  }
  return true;
}

interface_protocol__msg__MotorCommand *
interface_protocol__msg__MotorCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotorCommand * msg = (interface_protocol__msg__MotorCommand *)allocator.allocate(sizeof(interface_protocol__msg__MotorCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_protocol__msg__MotorCommand));
  bool success = interface_protocol__msg__MotorCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_protocol__msg__MotorCommand__destroy(interface_protocol__msg__MotorCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_protocol__msg__MotorCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_protocol__msg__MotorCommand__Sequence__init(interface_protocol__msg__MotorCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotorCommand * data = NULL;

  if (size) {
    data = (interface_protocol__msg__MotorCommand *)allocator.zero_allocate(size, sizeof(interface_protocol__msg__MotorCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_protocol__msg__MotorCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_protocol__msg__MotorCommand__fini(&data[i - 1]);
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
interface_protocol__msg__MotorCommand__Sequence__fini(interface_protocol__msg__MotorCommand__Sequence * array)
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
      interface_protocol__msg__MotorCommand__fini(&array->data[i]);
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

interface_protocol__msg__MotorCommand__Sequence *
interface_protocol__msg__MotorCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotorCommand__Sequence * array = (interface_protocol__msg__MotorCommand__Sequence *)allocator.allocate(sizeof(interface_protocol__msg__MotorCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_protocol__msg__MotorCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_protocol__msg__MotorCommand__Sequence__destroy(interface_protocol__msg__MotorCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_protocol__msg__MotorCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_protocol__msg__MotorCommand__Sequence__are_equal(const interface_protocol__msg__MotorCommand__Sequence * lhs, const interface_protocol__msg__MotorCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_protocol__msg__MotorCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_protocol__msg__MotorCommand__Sequence__copy(
  const interface_protocol__msg__MotorCommand__Sequence * input,
  interface_protocol__msg__MotorCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_protocol__msg__MotorCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_protocol__msg__MotorCommand * data =
      (interface_protocol__msg__MotorCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_protocol__msg__MotorCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_protocol__msg__MotorCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_protocol__msg__MotorCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
