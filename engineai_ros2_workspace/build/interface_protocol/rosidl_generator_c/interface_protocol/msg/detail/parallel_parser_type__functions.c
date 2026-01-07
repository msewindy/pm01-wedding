// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_protocol:msg/ParallelParserType.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/parallel_parser_type__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interface_protocol__msg__ParallelParserType__init(interface_protocol__msg__ParallelParserType * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
interface_protocol__msg__ParallelParserType__fini(interface_protocol__msg__ParallelParserType * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
interface_protocol__msg__ParallelParserType__are_equal(const interface_protocol__msg__ParallelParserType * lhs, const interface_protocol__msg__ParallelParserType * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
interface_protocol__msg__ParallelParserType__copy(
  const interface_protocol__msg__ParallelParserType * input,
  interface_protocol__msg__ParallelParserType * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

interface_protocol__msg__ParallelParserType *
interface_protocol__msg__ParallelParserType__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__ParallelParserType * msg = (interface_protocol__msg__ParallelParserType *)allocator.allocate(sizeof(interface_protocol__msg__ParallelParserType), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_protocol__msg__ParallelParserType));
  bool success = interface_protocol__msg__ParallelParserType__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_protocol__msg__ParallelParserType__destroy(interface_protocol__msg__ParallelParserType * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_protocol__msg__ParallelParserType__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_protocol__msg__ParallelParserType__Sequence__init(interface_protocol__msg__ParallelParserType__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__ParallelParserType * data = NULL;

  if (size) {
    data = (interface_protocol__msg__ParallelParserType *)allocator.zero_allocate(size, sizeof(interface_protocol__msg__ParallelParserType), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_protocol__msg__ParallelParserType__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_protocol__msg__ParallelParserType__fini(&data[i - 1]);
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
interface_protocol__msg__ParallelParserType__Sequence__fini(interface_protocol__msg__ParallelParserType__Sequence * array)
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
      interface_protocol__msg__ParallelParserType__fini(&array->data[i]);
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

interface_protocol__msg__ParallelParserType__Sequence *
interface_protocol__msg__ParallelParserType__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__ParallelParserType__Sequence * array = (interface_protocol__msg__ParallelParserType__Sequence *)allocator.allocate(sizeof(interface_protocol__msg__ParallelParserType__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_protocol__msg__ParallelParserType__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_protocol__msg__ParallelParserType__Sequence__destroy(interface_protocol__msg__ParallelParserType__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_protocol__msg__ParallelParserType__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_protocol__msg__ParallelParserType__Sequence__are_equal(const interface_protocol__msg__ParallelParserType__Sequence * lhs, const interface_protocol__msg__ParallelParserType__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_protocol__msg__ParallelParserType__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_protocol__msg__ParallelParserType__Sequence__copy(
  const interface_protocol__msg__ParallelParserType__Sequence * input,
  interface_protocol__msg__ParallelParserType__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_protocol__msg__ParallelParserType);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_protocol__msg__ParallelParserType * data =
      (interface_protocol__msg__ParallelParserType *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_protocol__msg__ParallelParserType__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_protocol__msg__ParallelParserType__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_protocol__msg__ParallelParserType__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
