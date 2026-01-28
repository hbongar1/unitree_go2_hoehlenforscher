// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from go2_msgs:msg/Entrance.idl
// generated code does not contain a copyright notice
#include "go2_msgs/msg/detail/entrance__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
go2_msgs__msg__Entrance__init(go2_msgs__msg__Entrance * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    go2_msgs__msg__Entrance__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    go2_msgs__msg__Entrance__fini(msg);
    return false;
  }
  // width
  // height
  return true;
}

void
go2_msgs__msg__Entrance__fini(go2_msgs__msg__Entrance * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // width
  // height
}

bool
go2_msgs__msg__Entrance__are_equal(const go2_msgs__msg__Entrance * lhs, const go2_msgs__msg__Entrance * rhs)
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
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  return true;
}

bool
go2_msgs__msg__Entrance__copy(
  const go2_msgs__msg__Entrance * input,
  go2_msgs__msg__Entrance * output)
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
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  return true;
}

go2_msgs__msg__Entrance *
go2_msgs__msg__Entrance__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_msgs__msg__Entrance * msg = (go2_msgs__msg__Entrance *)allocator.allocate(sizeof(go2_msgs__msg__Entrance), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(go2_msgs__msg__Entrance));
  bool success = go2_msgs__msg__Entrance__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
go2_msgs__msg__Entrance__destroy(go2_msgs__msg__Entrance * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    go2_msgs__msg__Entrance__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
go2_msgs__msg__Entrance__Sequence__init(go2_msgs__msg__Entrance__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_msgs__msg__Entrance * data = NULL;

  if (size) {
    data = (go2_msgs__msg__Entrance *)allocator.zero_allocate(size, sizeof(go2_msgs__msg__Entrance), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = go2_msgs__msg__Entrance__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        go2_msgs__msg__Entrance__fini(&data[i - 1]);
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
go2_msgs__msg__Entrance__Sequence__fini(go2_msgs__msg__Entrance__Sequence * array)
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
      go2_msgs__msg__Entrance__fini(&array->data[i]);
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

go2_msgs__msg__Entrance__Sequence *
go2_msgs__msg__Entrance__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_msgs__msg__Entrance__Sequence * array = (go2_msgs__msg__Entrance__Sequence *)allocator.allocate(sizeof(go2_msgs__msg__Entrance__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = go2_msgs__msg__Entrance__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
go2_msgs__msg__Entrance__Sequence__destroy(go2_msgs__msg__Entrance__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    go2_msgs__msg__Entrance__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
go2_msgs__msg__Entrance__Sequence__are_equal(const go2_msgs__msg__Entrance__Sequence * lhs, const go2_msgs__msg__Entrance__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!go2_msgs__msg__Entrance__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
go2_msgs__msg__Entrance__Sequence__copy(
  const go2_msgs__msg__Entrance__Sequence * input,
  go2_msgs__msg__Entrance__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(go2_msgs__msg__Entrance);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    go2_msgs__msg__Entrance * data =
      (go2_msgs__msg__Entrance *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!go2_msgs__msg__Entrance__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          go2_msgs__msg__Entrance__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!go2_msgs__msg__Entrance__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
