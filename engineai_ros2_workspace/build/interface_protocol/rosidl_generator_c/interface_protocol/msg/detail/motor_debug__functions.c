// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_protocol:msg/MotorDebug.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/motor_debug__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tau_cmd`
// Member `mos_temperature`
// Member `motor_temperature`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
interface_protocol__msg__MotorDebug__init(interface_protocol__msg__MotorDebug * msg)
{
  if (!msg) {
    return false;
  }
  // tau_cmd
  if (!rosidl_runtime_c__double__Sequence__init(&msg->tau_cmd, 0)) {
    interface_protocol__msg__MotorDebug__fini(msg);
    return false;
  }
  // mos_temperature
  if (!rosidl_runtime_c__double__Sequence__init(&msg->mos_temperature, 0)) {
    interface_protocol__msg__MotorDebug__fini(msg);
    return false;
  }
  // motor_temperature
  if (!rosidl_runtime_c__double__Sequence__init(&msg->motor_temperature, 0)) {
    interface_protocol__msg__MotorDebug__fini(msg);
    return false;
  }
  return true;
}

void
interface_protocol__msg__MotorDebug__fini(interface_protocol__msg__MotorDebug * msg)
{
  if (!msg) {
    return;
  }
  // tau_cmd
  rosidl_runtime_c__double__Sequence__fini(&msg->tau_cmd);
  // mos_temperature
  rosidl_runtime_c__double__Sequence__fini(&msg->mos_temperature);
  // motor_temperature
  rosidl_runtime_c__double__Sequence__fini(&msg->motor_temperature);
}

bool
interface_protocol__msg__MotorDebug__are_equal(const interface_protocol__msg__MotorDebug * lhs, const interface_protocol__msg__MotorDebug * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tau_cmd
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->tau_cmd), &(rhs->tau_cmd)))
  {
    return false;
  }
  // mos_temperature
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->mos_temperature), &(rhs->mos_temperature)))
  {
    return false;
  }
  // motor_temperature
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->motor_temperature), &(rhs->motor_temperature)))
  {
    return false;
  }
  return true;
}

bool
interface_protocol__msg__MotorDebug__copy(
  const interface_protocol__msg__MotorDebug * input,
  interface_protocol__msg__MotorDebug * output)
{
  if (!input || !output) {
    return false;
  }
  // tau_cmd
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->tau_cmd), &(output->tau_cmd)))
  {
    return false;
  }
  // mos_temperature
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->mos_temperature), &(output->mos_temperature)))
  {
    return false;
  }
  // motor_temperature
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->motor_temperature), &(output->motor_temperature)))
  {
    return false;
  }
  return true;
}

interface_protocol__msg__MotorDebug *
interface_protocol__msg__MotorDebug__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotorDebug * msg = (interface_protocol__msg__MotorDebug *)allocator.allocate(sizeof(interface_protocol__msg__MotorDebug), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_protocol__msg__MotorDebug));
  bool success = interface_protocol__msg__MotorDebug__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_protocol__msg__MotorDebug__destroy(interface_protocol__msg__MotorDebug * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_protocol__msg__MotorDebug__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_protocol__msg__MotorDebug__Sequence__init(interface_protocol__msg__MotorDebug__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotorDebug * data = NULL;

  if (size) {
    data = (interface_protocol__msg__MotorDebug *)allocator.zero_allocate(size, sizeof(interface_protocol__msg__MotorDebug), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_protocol__msg__MotorDebug__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_protocol__msg__MotorDebug__fini(&data[i - 1]);
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
interface_protocol__msg__MotorDebug__Sequence__fini(interface_protocol__msg__MotorDebug__Sequence * array)
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
      interface_protocol__msg__MotorDebug__fini(&array->data[i]);
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

interface_protocol__msg__MotorDebug__Sequence *
interface_protocol__msg__MotorDebug__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__MotorDebug__Sequence * array = (interface_protocol__msg__MotorDebug__Sequence *)allocator.allocate(sizeof(interface_protocol__msg__MotorDebug__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_protocol__msg__MotorDebug__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_protocol__msg__MotorDebug__Sequence__destroy(interface_protocol__msg__MotorDebug__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_protocol__msg__MotorDebug__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_protocol__msg__MotorDebug__Sequence__are_equal(const interface_protocol__msg__MotorDebug__Sequence * lhs, const interface_protocol__msg__MotorDebug__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_protocol__msg__MotorDebug__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_protocol__msg__MotorDebug__Sequence__copy(
  const interface_protocol__msg__MotorDebug__Sequence * input,
  interface_protocol__msg__MotorDebug__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_protocol__msg__MotorDebug);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_protocol__msg__MotorDebug * data =
      (interface_protocol__msg__MotorDebug *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_protocol__msg__MotorDebug__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_protocol__msg__MotorDebug__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_protocol__msg__MotorDebug__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
