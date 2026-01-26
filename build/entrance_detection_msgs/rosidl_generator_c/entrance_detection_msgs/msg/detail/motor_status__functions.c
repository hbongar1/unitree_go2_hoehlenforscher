// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from entrance_detection_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice
#include "entrance_detection_msgs/msg/detail/motor_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `posture_mode`
// Member `error_message`
#include "rosidl_runtime_c/string_functions.h"

bool
entrance_detection_msgs__msg__MotorStatus__init(entrance_detection_msgs__msg__MotorStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    entrance_detection_msgs__msg__MotorStatus__fini(msg);
    return false;
  }
  // current_body_height
  // posture_mode
  if (!rosidl_runtime_c__String__init(&msg->posture_mode)) {
    entrance_detection_msgs__msg__MotorStatus__fini(msg);
    return false;
  }
  // in_motion
  // ready
  // error_message
  if (!rosidl_runtime_c__String__init(&msg->error_message)) {
    entrance_detection_msgs__msg__MotorStatus__fini(msg);
    return false;
  }
  return true;
}

void
entrance_detection_msgs__msg__MotorStatus__fini(entrance_detection_msgs__msg__MotorStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // current_body_height
  // posture_mode
  rosidl_runtime_c__String__fini(&msg->posture_mode);
  // in_motion
  // ready
  // error_message
  rosidl_runtime_c__String__fini(&msg->error_message);
}

bool
entrance_detection_msgs__msg__MotorStatus__are_equal(const entrance_detection_msgs__msg__MotorStatus * lhs, const entrance_detection_msgs__msg__MotorStatus * rhs)
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
  // current_body_height
  if (lhs->current_body_height != rhs->current_body_height) {
    return false;
  }
  // posture_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->posture_mode), &(rhs->posture_mode)))
  {
    return false;
  }
  // in_motion
  if (lhs->in_motion != rhs->in_motion) {
    return false;
  }
  // ready
  if (lhs->ready != rhs->ready) {
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
entrance_detection_msgs__msg__MotorStatus__copy(
  const entrance_detection_msgs__msg__MotorStatus * input,
  entrance_detection_msgs__msg__MotorStatus * output)
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
  // current_body_height
  output->current_body_height = input->current_body_height;
  // posture_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->posture_mode), &(output->posture_mode)))
  {
    return false;
  }
  // in_motion
  output->in_motion = input->in_motion;
  // ready
  output->ready = input->ready;
  // error_message
  if (!rosidl_runtime_c__String__copy(
      &(input->error_message), &(output->error_message)))
  {
    return false;
  }
  return true;
}

entrance_detection_msgs__msg__MotorStatus *
entrance_detection_msgs__msg__MotorStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__MotorStatus * msg = (entrance_detection_msgs__msg__MotorStatus *)allocator.allocate(sizeof(entrance_detection_msgs__msg__MotorStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(entrance_detection_msgs__msg__MotorStatus));
  bool success = entrance_detection_msgs__msg__MotorStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
entrance_detection_msgs__msg__MotorStatus__destroy(entrance_detection_msgs__msg__MotorStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    entrance_detection_msgs__msg__MotorStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
entrance_detection_msgs__msg__MotorStatus__Sequence__init(entrance_detection_msgs__msg__MotorStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__MotorStatus * data = NULL;

  if (size) {
    data = (entrance_detection_msgs__msg__MotorStatus *)allocator.zero_allocate(size, sizeof(entrance_detection_msgs__msg__MotorStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = entrance_detection_msgs__msg__MotorStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        entrance_detection_msgs__msg__MotorStatus__fini(&data[i - 1]);
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
entrance_detection_msgs__msg__MotorStatus__Sequence__fini(entrance_detection_msgs__msg__MotorStatus__Sequence * array)
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
      entrance_detection_msgs__msg__MotorStatus__fini(&array->data[i]);
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

entrance_detection_msgs__msg__MotorStatus__Sequence *
entrance_detection_msgs__msg__MotorStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  entrance_detection_msgs__msg__MotorStatus__Sequence * array = (entrance_detection_msgs__msg__MotorStatus__Sequence *)allocator.allocate(sizeof(entrance_detection_msgs__msg__MotorStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = entrance_detection_msgs__msg__MotorStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
entrance_detection_msgs__msg__MotorStatus__Sequence__destroy(entrance_detection_msgs__msg__MotorStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    entrance_detection_msgs__msg__MotorStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
entrance_detection_msgs__msg__MotorStatus__Sequence__are_equal(const entrance_detection_msgs__msg__MotorStatus__Sequence * lhs, const entrance_detection_msgs__msg__MotorStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!entrance_detection_msgs__msg__MotorStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
entrance_detection_msgs__msg__MotorStatus__Sequence__copy(
  const entrance_detection_msgs__msg__MotorStatus__Sequence * input,
  entrance_detection_msgs__msg__MotorStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(entrance_detection_msgs__msg__MotorStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    entrance_detection_msgs__msg__MotorStatus * data =
      (entrance_detection_msgs__msg__MotorStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!entrance_detection_msgs__msg__MotorStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          entrance_detection_msgs__msg__MotorStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!entrance_detection_msgs__msg__MotorStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
