// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from entrance_detection_msgs:msg/EntranceState.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__STRUCT_H_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__STRUCT_H_

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
// Member 'state'
// Member 'description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/EntranceState in the package entrance_detection_msgs.
/**
  * EntranceState.msg
  * Current state and decision for entrance navigation
 */
typedef struct entrance_detection_msgs__msg__EntranceState
{
  std_msgs__msg__Header header;
  /// State machine state
  /// NORMAL, NIEDRIG, LIEGEND, BLOCKIERT, ANALYSE
  rosidl_runtime_c__String state;
  /// Required robot configuration
  /// Required body height in meters
  float required_height;
  /// Safety clearance above obstacle in meters
  float safety_clearance;
  /// Action flag
  /// True if robot must change posture
  bool action_required;
  /// Additional information
  /// Human-readable state description
  rosidl_runtime_c__String description;
} entrance_detection_msgs__msg__EntranceState;

// Struct for a sequence of entrance_detection_msgs__msg__EntranceState.
typedef struct entrance_detection_msgs__msg__EntranceState__Sequence
{
  entrance_detection_msgs__msg__EntranceState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} entrance_detection_msgs__msg__EntranceState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__STRUCT_H_
