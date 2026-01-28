// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from go2_msgs:msg/Entrance.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__MSG__DETAIL__ENTRANCE__STRUCT_H_
#define GO2_MSGS__MSG__DETAIL__ENTRANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Entrance in the package go2_msgs.
/**
  * Entrance.msg
  * Detected entrance with position and dimensions
 */
typedef struct go2_msgs__msg__Entrance
{
  std_msgs__msg__Header header;
  /// Position of entrance center (meters)
  geometry_msgs__msg__Point position;
  /// Entrance dimensions
  /// Width in meters
  float width;
  /// Height in meters
  float height;
} go2_msgs__msg__Entrance;

// Struct for a sequence of go2_msgs__msg__Entrance.
typedef struct go2_msgs__msg__Entrance__Sequence
{
  go2_msgs__msg__Entrance * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__msg__Entrance__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GO2_MSGS__MSG__DETAIL__ENTRANCE__STRUCT_H_
