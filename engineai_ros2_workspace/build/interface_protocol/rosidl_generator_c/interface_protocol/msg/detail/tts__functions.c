// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_protocol:msg/Tts.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/tts__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `text`
// Member `language`
// Member `speaker`
#include "rosidl_runtime_c/string_functions.h"

bool
interface_protocol__msg__Tts__init(interface_protocol__msg__Tts * msg)
{
  if (!msg) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__init(&msg->text)) {
    interface_protocol__msg__Tts__fini(msg);
    return false;
  }
  // language
  if (!rosidl_runtime_c__String__init(&msg->language)) {
    interface_protocol__msg__Tts__fini(msg);
    return false;
  }
  // speaker
  if (!rosidl_runtime_c__String__init(&msg->speaker)) {
    interface_protocol__msg__Tts__fini(msg);
    return false;
  }
  // rate
  return true;
}

void
interface_protocol__msg__Tts__fini(interface_protocol__msg__Tts * msg)
{
  if (!msg) {
    return;
  }
  // text
  rosidl_runtime_c__String__fini(&msg->text);
  // language
  rosidl_runtime_c__String__fini(&msg->language);
  // speaker
  rosidl_runtime_c__String__fini(&msg->speaker);
  // rate
}

bool
interface_protocol__msg__Tts__are_equal(const interface_protocol__msg__Tts * lhs, const interface_protocol__msg__Tts * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  // language
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->language), &(rhs->language)))
  {
    return false;
  }
  // speaker
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->speaker), &(rhs->speaker)))
  {
    return false;
  }
  // rate
  if (lhs->rate != rhs->rate) {
    return false;
  }
  return true;
}

bool
interface_protocol__msg__Tts__copy(
  const interface_protocol__msg__Tts * input,
  interface_protocol__msg__Tts * output)
{
  if (!input || !output) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__String__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  // language
  if (!rosidl_runtime_c__String__copy(
      &(input->language), &(output->language)))
  {
    return false;
  }
  // speaker
  if (!rosidl_runtime_c__String__copy(
      &(input->speaker), &(output->speaker)))
  {
    return false;
  }
  // rate
  output->rate = input->rate;
  return true;
}

interface_protocol__msg__Tts *
interface_protocol__msg__Tts__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__Tts * msg = (interface_protocol__msg__Tts *)allocator.allocate(sizeof(interface_protocol__msg__Tts), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_protocol__msg__Tts));
  bool success = interface_protocol__msg__Tts__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_protocol__msg__Tts__destroy(interface_protocol__msg__Tts * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_protocol__msg__Tts__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_protocol__msg__Tts__Sequence__init(interface_protocol__msg__Tts__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__Tts * data = NULL;

  if (size) {
    data = (interface_protocol__msg__Tts *)allocator.zero_allocate(size, sizeof(interface_protocol__msg__Tts), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_protocol__msg__Tts__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_protocol__msg__Tts__fini(&data[i - 1]);
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
interface_protocol__msg__Tts__Sequence__fini(interface_protocol__msg__Tts__Sequence * array)
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
      interface_protocol__msg__Tts__fini(&array->data[i]);
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

interface_protocol__msg__Tts__Sequence *
interface_protocol__msg__Tts__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_protocol__msg__Tts__Sequence * array = (interface_protocol__msg__Tts__Sequence *)allocator.allocate(sizeof(interface_protocol__msg__Tts__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_protocol__msg__Tts__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_protocol__msg__Tts__Sequence__destroy(interface_protocol__msg__Tts__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_protocol__msg__Tts__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_protocol__msg__Tts__Sequence__are_equal(const interface_protocol__msg__Tts__Sequence * lhs, const interface_protocol__msg__Tts__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_protocol__msg__Tts__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_protocol__msg__Tts__Sequence__copy(
  const interface_protocol__msg__Tts__Sequence * input,
  interface_protocol__msg__Tts__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_protocol__msg__Tts);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_protocol__msg__Tts * data =
      (interface_protocol__msg__Tts *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_protocol__msg__Tts__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_protocol__msg__Tts__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_protocol__msg__Tts__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
