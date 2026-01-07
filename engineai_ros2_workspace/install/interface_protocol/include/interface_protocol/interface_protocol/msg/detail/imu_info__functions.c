// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_protocol:msg/ImuInfo.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/imu_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `quaternion`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `rpy`
// Member `linear_acceleration`
// Member `angular_velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
interface_protocol__msg__ImuInfo__init(interface_protocol__msg__ImuInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    interface_protocol__msg__ImuInfo__fini(msg);
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__Quaternion__init(&msg->quaternion)) {
    interface_protocol__msg__ImuInfo__fini(msg);
    return false;
  }
  // rpy
  if (!geometry_msgs__msg__Vector3__init(&msg->rpy)) {
    interface_protocol__msg__ImuInfo__fini(msg);
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_acceleration)) {
    interface_protocol__msg__ImuInfo__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    interface_protocol__msg__ImuInfo__fini(msg);
    return false;
  }
  return true;
}

void
interface_protocol__msg__ImuInfo__fini(interface_protocol__msg__ImuInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // quaternion
  geometry_msgs__msg__Quaternion__fini(&msg->quaternion);
  // rpy
  geometry_msgs__msg__Vector3__fini(&msg->rpy);
  // linear_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->linear_acceleration);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
}

bool
interface_protocol__msg__ImuInfo__are_equal(const interface_protocol__msg__ImuInfo * lhs, const interface_protocol__msg__ImuInfo * rhs)
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
  // quaternion
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->quaternion), &(rhs->quaternion)))
  {
    return false;
  }
  // rpy
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->rpy), &(rhs->rpy)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->linear_acceleration), &(rhs->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->angular_velocity), &(rhs->angular_velocity)))
  {
    return false;
  }
  return true;
}

bool
interface_protocol__msg__ImuInfo__copy(
  const interface_protocol__msg__ImuInfo * input,
  interface_protocol__msg__ImuInfo * output)
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
  // quaternion
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->quaternion), &(output->quaternion)))
  {
    return false;
  }
  // rpy
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->rpy), &(output->rpy)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->linear_acceleration), &(output->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->angular_velocity), &(output->angular_velocity)))
  {
    return false;
  }
  return true;
}

interface_protocol__msg__ImuInfo *
interface_protocol__msg__ImuInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__ImuInfo * msg = (interface_protocol__msg__ImuInfo *)allocator.allocate(sizeof(interface_protocol__msg__ImuInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_protocol__msg__ImuInfo));
  bool success = interface_protocol__msg__ImuInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_protocol__msg__ImuInfo__destroy(interface_protocol__msg__ImuInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_protocol__msg__ImuInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_protocol__msg__ImuInfo__Sequence__init(interface_protocol__msg__ImuInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__ImuInfo * data = NULL;

  if (size) {
    data = (interface_protocol__msg__ImuInfo *)allocator.zero_allocate(size, sizeof(interface_protocol__msg__ImuInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_protocol__msg__ImuInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_protocol__msg__ImuInfo__fini(&data[i - 1]);
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
interface_protocol__msg__ImuInfo__Sequence__fini(interface_protocol__msg__ImuInfo__Sequence * array)
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
      interface_protocol__msg__ImuInfo__fini(&array->data[i]);
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

interface_protocol__msg__ImuInfo__Sequence *
interface_protocol__msg__ImuInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__ImuInfo__Sequence * array = (interface_protocol__msg__ImuInfo__Sequence *)allocator.allocate(sizeof(interface_protocol__msg__ImuInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_protocol__msg__ImuInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_protocol__msg__ImuInfo__Sequence__destroy(interface_protocol__msg__ImuInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_protocol__msg__ImuInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_protocol__msg__ImuInfo__Sequence__are_equal(const interface_protocol__msg__ImuInfo__Sequence * lhs, const interface_protocol__msg__ImuInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_protocol__msg__ImuInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_protocol__msg__ImuInfo__Sequence__copy(
  const interface_protocol__msg__ImuInfo__Sequence * input,
  interface_protocol__msg__ImuInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_protocol__msg__ImuInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_protocol__msg__ImuInfo * data =
      (interface_protocol__msg__ImuInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_protocol__msg__ImuInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_protocol__msg__ImuInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_protocol__msg__ImuInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
