// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from entrance_detection_msgs:msg/EntranceEvent.idl
// generated code does not contain a copyright notice
#include "entrance_detection_msgs/msg/detail/entrance_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position_3d`
// Member `position_2d`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
entrance_detection_msgs__msg__EntranceEvent__init(entrance_detection_msgs__msg__EntranceEvent * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    entrance_detection_msgs__msg__EntranceEvent__fini(msg);
    return false;
  }
  // position_3d
  if (!geometry_msgs__msg__Point__init(&msg->position_3d)) {
    entrance_detection_msgs__msg__EntranceEvent__fini(msg);
    return false;
  }
  // position_2d
  if (!geometry_msgs__msg__Point__init(&msg->position_2d)) {
    entrance_detection_msgs__msg__EntranceEvent__fini(msg);
    return false;
  }
  // width
  // height
  // distance
  // confidence
  // is_passable
  return true;
}

void
entrance_detection_msgs__msg__EntranceEvent__fini(entrance_detection_msgs__msg__EntranceEvent * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position_3d
  geometry_msgs__msg__Point__fini(&msg->position_3d);
  // position_2d
  geometry_msgs__msg__Point__fini(&msg->position_2d);
  // width
  // height
  // distance
  // confidence
  // is_passable
}

bool
entrance_detection_msgs__msg__EntranceEvent__are_equal(const entrance_detection_msgs__msg__EntranceEvent * lhs, const entrance_detection_msgs__msg__EntranceEvent * rhs)
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
  // position_3d
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position_3d), &(rhs->position_3d)))
  {
    return false;
  }
  // position_2d
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position_2d), &(rhs->position_2d)))
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
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // is_passable
  if (lhs->is_passable != rhs->is_passable) {
    return false;
  }
  return true;
}

bool
entrance_detection_msgs__msg__EntranceEvent__copy(
  const entrance_detection_msgs__msg__EntranceEvent * input,
  entrance_detection_msgs__msg__EntranceEvent * output)
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
  // position_3d
  if (!geometry_msgs__msg__Point__copy(
      &(input->position_3d), &(output->position_3d)))
  {
    return false;
  }
  // position_2d
  if (!geometry_msgs__msg__Point__copy(
      &(input->position_2d), &(output->position_2d)))
  {
    return false;
  }
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // distance
  output->distance = input->distance;
  // confidence
  output->confidence = input->confidence;
  // is_passable
  output->is_passable = input->is_passable;
  return true;
}

entrance_detection_msgs__msg__EntranceEvent *
entrance_detection_msgs__msg__EntranceEvent__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__EntranceEvent * msg = (entrance_detection_msgs__msg__EntranceEvent *)allocator.allocate(sizeof(entrance_detection_msgs__msg__EntranceEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(entrance_detection_msgs__msg__EntranceEvent));
  bool success = entrance_detection_msgs__msg__EntranceEvent__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
entrance_detection_msgs__msg__EntranceEvent__destroy(entrance_detection_msgs__msg__EntranceEvent * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    entrance_detection_msgs__msg__EntranceEvent__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
entrance_detection_msgs__msg__EntranceEvent__Sequence__init(entrance_detection_msgs__msg__EntranceEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__EntranceEvent * data = NULL;

  if (size) {
    data = (entrance_detection_msgs__msg__EntranceEvent *)allocator.zero_allocate(size, sizeof(entrance_detection_msgs__msg__EntranceEvent), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = entrance_detection_msgs__msg__EntranceEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        entrance_detection_msgs__msg__EntranceEvent__fini(&data[i - 1]);
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
entrance_detection_msgs__msg__EntranceEvent__Sequence__fini(entrance_detection_msgs__msg__EntranceEvent__Sequence * array)
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
      entrance_detection_msgs__msg__EntranceEvent__fini(&array->data[i]);
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

entrance_detection_msgs__msg__EntranceEvent__Sequence *
entrance_detection_msgs__msg__EntranceEvent__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__EntranceEvent__Sequence * array = (entrance_detection_msgs__msg__EntranceEvent__Sequence *)allocator.allocate(sizeof(entrance_detection_msgs__msg__EntranceEvent__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = entrance_detection_msgs__msg__EntranceEvent__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
entrance_detection_msgs__msg__EntranceEvent__Sequence__destroy(entrance_detection_msgs__msg__EntranceEvent__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    entrance_detection_msgs__msg__EntranceEvent__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
entrance_detection_msgs__msg__EntranceEvent__Sequence__are_equal(const entrance_detection_msgs__msg__EntranceEvent__Sequence * lhs, const entrance_detection_msgs__msg__EntranceEvent__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!entrance_detection_msgs__msg__EntranceEvent__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
entrance_detection_msgs__msg__EntranceEvent__Sequence__copy(
  const entrance_detection_msgs__msg__EntranceEvent__Sequence * input,
  entrance_detection_msgs__msg__EntranceEvent__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(entrance_detection_msgs__msg__EntranceEvent);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    entrance_detection_msgs__msg__EntranceEvent * data =
      (entrance_detection_msgs__msg__EntranceEvent *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!entrance_detection_msgs__msg__EntranceEvent__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          entrance_detection_msgs__msg__EntranceEvent__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!entrance_detection_msgs__msg__EntranceEvent__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
