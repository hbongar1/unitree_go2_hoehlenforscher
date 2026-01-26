// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from entrance_detection_msgs:msg/DepthData.idl
// generated code does not contain a copyright notice
#include "entrance_detection_msgs/msg/detail/depth_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `depth_image`
// Member `color_image`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__functions.h"

bool
entrance_detection_msgs__msg__DepthData__init(entrance_detection_msgs__msg__DepthData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    entrance_detection_msgs__msg__DepthData__fini(msg);
    return false;
  }
  // depth_image
  if (!sensor_msgs__msg__Image__init(&msg->depth_image)) {
    entrance_detection_msgs__msg__DepthData__fini(msg);
    return false;
  }
  // color_image
  if (!sensor_msgs__msg__Image__init(&msg->color_image)) {
    entrance_detection_msgs__msg__DepthData__fini(msg);
    return false;
  }
  // depth_scale
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__init(&msg->camera_info)) {
    entrance_detection_msgs__msg__DepthData__fini(msg);
    return false;
  }
  return true;
}

void
entrance_detection_msgs__msg__DepthData__fini(entrance_detection_msgs__msg__DepthData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // depth_image
  sensor_msgs__msg__Image__fini(&msg->depth_image);
  // color_image
  sensor_msgs__msg__Image__fini(&msg->color_image);
  // depth_scale
  // camera_info
  sensor_msgs__msg__CameraInfo__fini(&msg->camera_info);
}

bool
entrance_detection_msgs__msg__DepthData__are_equal(const entrance_detection_msgs__msg__DepthData * lhs, const entrance_detection_msgs__msg__DepthData * rhs)
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
  // depth_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->depth_image), &(rhs->depth_image)))
  {
    return false;
  }
  // color_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->color_image), &(rhs->color_image)))
  {
    return false;
  }
  // depth_scale
  if (lhs->depth_scale != rhs->depth_scale) {
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__are_equal(
      &(lhs->camera_info), &(rhs->camera_info)))
  {
    return false;
  }
  return true;
}

bool
entrance_detection_msgs__msg__DepthData__copy(
  const entrance_detection_msgs__msg__DepthData * input,
  entrance_detection_msgs__msg__DepthData * output)
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
  // depth_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->depth_image), &(output->depth_image)))
  {
    return false;
  }
  // color_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->color_image), &(output->color_image)))
  {
    return false;
  }
  // depth_scale
  output->depth_scale = input->depth_scale;
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__copy(
      &(input->camera_info), &(output->camera_info)))
  {
    return false;
  }
  return true;
}

entrance_detection_msgs__msg__DepthData *
entrance_detection_msgs__msg__DepthData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__DepthData * msg = (entrance_detection_msgs__msg__DepthData *)allocator.allocate(sizeof(entrance_detection_msgs__msg__DepthData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(entrance_detection_msgs__msg__DepthData));
  bool success = entrance_detection_msgs__msg__DepthData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
entrance_detection_msgs__msg__DepthData__destroy(entrance_detection_msgs__msg__DepthData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    entrance_detection_msgs__msg__DepthData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
entrance_detection_msgs__msg__DepthData__Sequence__init(entrance_detection_msgs__msg__DepthData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__DepthData * data = NULL;

  if (size) {
    data = (entrance_detection_msgs__msg__DepthData *)allocator.zero_allocate(size, sizeof(entrance_detection_msgs__msg__DepthData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = entrance_detection_msgs__msg__DepthData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        entrance_detection_msgs__msg__DepthData__fini(&data[i - 1]);
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
entrance_detection_msgs__msg__DepthData__Sequence__fini(entrance_detection_msgs__msg__DepthData__Sequence * array)
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
      entrance_detection_msgs__msg__DepthData__fini(&array->data[i]);
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

entrance_detection_msgs__msg__DepthData__Sequence *
entrance_detection_msgs__msg__DepthData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__DepthData__Sequence * array = (entrance_detection_msgs__msg__DepthData__Sequence *)allocator.allocate(sizeof(entrance_detection_msgs__msg__DepthData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = entrance_detection_msgs__msg__DepthData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
entrance_detection_msgs__msg__DepthData__Sequence__destroy(entrance_detection_msgs__msg__DepthData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    entrance_detection_msgs__msg__DepthData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
entrance_detection_msgs__msg__DepthData__Sequence__are_equal(const entrance_detection_msgs__msg__DepthData__Sequence * lhs, const entrance_detection_msgs__msg__DepthData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!entrance_detection_msgs__msg__DepthData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
entrance_detection_msgs__msg__DepthData__Sequence__copy(
  const entrance_detection_msgs__msg__DepthData__Sequence * input,
  entrance_detection_msgs__msg__DepthData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(entrance_detection_msgs__msg__DepthData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    entrance_detection_msgs__msg__DepthData * data =
      (entrance_detection_msgs__msg__DepthData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!entrance_detection_msgs__msg__DepthData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          entrance_detection_msgs__msg__DepthData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!entrance_detection_msgs__msg__DepthData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
