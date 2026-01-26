// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from entrance_detection_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__STRUCT_H_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__STRUCT_H_

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
// Member 'posture_mode'
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorStatus in the package entrance_detection_msgs.
/**
  * MotorStatus.msg
  * Feedback from motor controller (Unitree GO2)
 */
typedef struct entrance_detection_msgs__msg__MotorStatus
{
  std_msgs__msg__Header header;
  /// Current robot state
  /// Current body height in meters
  float current_body_height;
  /// STAND, CROUCH, PRONE
  rosidl_runtime_c__String posture_mode;
  /// Motion status
  /// True if currently executing height change
  bool in_motion;
  /// True if ready to accept new commands
  bool ready;
  /// Error handling
  /// Empty if OK, otherwise contains error description
  rosidl_runtime_c__String error_message;
} entrance_detection_msgs__msg__MotorStatus;

// Struct for a sequence of entrance_detection_msgs__msg__MotorStatus.
typedef struct entrance_detection_msgs__msg__MotorStatus__Sequence
{
  entrance_detection_msgs__msg__MotorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} entrance_detection_msgs__msg__MotorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__STRUCT_H_
