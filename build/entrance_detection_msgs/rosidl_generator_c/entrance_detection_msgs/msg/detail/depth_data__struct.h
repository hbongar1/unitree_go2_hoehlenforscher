// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from entrance_detection_msgs:msg/DepthData.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__STRUCT_H_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__STRUCT_H_

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
// Member 'depth_image'
// Member 'color_image'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.h"

/// Struct defined in msg/DepthData in the package entrance_detection_msgs.
/**
  * DepthData.msg
  * Raw RealSense depth camera data
 */
typedef struct entrance_detection_msgs__msg__DepthData
{
  std_msgs__msg__Header header;
  /// Depth frame (640x480, 16-bit unsigned integers, z16 format)
  sensor_msgs__msg__Image depth_image;
  /// Color frame (640x480, BGR8 format)
  sensor_msgs__msg__Image color_image;
  /// Depth scale factor (converts depth units to meters)
  /// Typical value: ~0.001 (1 unit = 1mm)
  float depth_scale;
  /// Camera intrinsics for 3D projection
  sensor_msgs__msg__CameraInfo camera_info;
} entrance_detection_msgs__msg__DepthData;

// Struct for a sequence of entrance_detection_msgs__msg__DepthData.
typedef struct entrance_detection_msgs__msg__DepthData__Sequence
{
  entrance_detection_msgs__msg__DepthData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} entrance_detection_msgs__msg__DepthData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__STRUCT_H_
