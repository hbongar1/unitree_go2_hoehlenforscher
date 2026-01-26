// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from entrance_detection_msgs:msg/EntranceEvent.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__STRUCT_H_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__STRUCT_H_

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
// Member 'position_3d'
// Member 'position_2d'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/EntranceEvent in the package entrance_detection_msgs.
/**
  * EntranceEvent.msg
  * Detected entrance with 3D geometry information
 */
typedef struct entrance_detection_msgs__msg__EntranceEvent
{
  std_msgs__msg__Header header;
  /// 3D position of entrance center in camera frame (meters)
  geometry_msgs__msg__Point position_3d;
  /// 2D position of entrance center in image (pixels)
  geometry_msgs__msg__Point position_2d;
  /// Entrance dimensions
  /// Width in meters
  float width;
  /// Height in meters
  float height;
  /// Distance from camera in meters
  float distance;
  /// Detection quality
  /// Confidence score 0.0 - 1.0
  float confidence;
  /// Passability assessment
  /// True if entrance is wide and tall enough for robot
  bool is_passable;
} entrance_detection_msgs__msg__EntranceEvent;

// Struct for a sequence of entrance_detection_msgs__msg__EntranceEvent.
typedef struct entrance_detection_msgs__msg__EntranceEvent__Sequence
{
  entrance_detection_msgs__msg__EntranceEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} entrance_detection_msgs__msg__EntranceEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__STRUCT_H_
