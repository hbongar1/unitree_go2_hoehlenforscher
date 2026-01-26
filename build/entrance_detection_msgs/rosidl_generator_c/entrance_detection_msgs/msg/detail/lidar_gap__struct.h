// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from entrance_detection_msgs:msg/LidarGap.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__STRUCT_H_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__STRUCT_H_

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
// Member 'center'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/LidarGap in the package entrance_detection_msgs.
/**
  * LidarGap.msg
  * Detected gap in LiDAR scan (potential entrance)
 */
typedef struct entrance_detection_msgs__msg__LidarGap
{
  std_msgs__msg__Header header;
  /// Gap properties
  /// Width of gap in meters
  float width;
  /// Distance to gap center in meters
  float distance;
  /// Angle in degrees (0 = forward)
  float angle;
  /// Gap location (2D Cartesian)
  /// (x, y, 0) in LiDAR frame
  geometry_msgs__msg__Point center;
  /// Confidence
  /// Quality score 0.0 - 1.0
  float quality;
  /// True if gap is within camera field of view
  bool in_camera_fov;
} entrance_detection_msgs__msg__LidarGap;

// Struct for a sequence of entrance_detection_msgs__msg__LidarGap.
typedef struct entrance_detection_msgs__msg__LidarGap__Sequence
{
  entrance_detection_msgs__msg__LidarGap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} entrance_detection_msgs__msg__LidarGap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__STRUCT_H_
