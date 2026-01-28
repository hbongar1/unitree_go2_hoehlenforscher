// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from go2_msgs:action/NavigateToEntrance.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__STRUCT_H_
#define GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_Goal
{
  /// Goal: Target entrance to navigate through
  geometry_msgs__msg__Point target_position;
  float entrance_width;
  float entrance_height;
} go2_msgs__action__NavigateToEntrance_Goal;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_Goal.
typedef struct go2_msgs__action__NavigateToEntrance_Goal__Sequence
{
  go2_msgs__action__NavigateToEntrance_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_Result
{
  bool success;
  rosidl_runtime_c__String message;
} go2_msgs__action__NavigateToEntrance_Result;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_Result.
typedef struct go2_msgs__action__NavigateToEntrance_Result__Sequence
{
  go2_msgs__action__NavigateToEntrance_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_Feedback
{
  rosidl_runtime_c__String status;
  float distance_remaining;
  float progress_percentage;
} go2_msgs__action__NavigateToEntrance_Feedback;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_Feedback.
typedef struct go2_msgs__action__NavigateToEntrance_Feedback__Sequence
{
  go2_msgs__action__NavigateToEntrance_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "go2_msgs/action/detail/navigate_to_entrance__struct.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  go2_msgs__action__NavigateToEntrance_Goal goal;
} go2_msgs__action__NavigateToEntrance_SendGoal_Request;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_SendGoal_Request.
typedef struct go2_msgs__action__NavigateToEntrance_SendGoal_Request__Sequence
{
  go2_msgs__action__NavigateToEntrance_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} go2_msgs__action__NavigateToEntrance_SendGoal_Response;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_SendGoal_Response.
typedef struct go2_msgs__action__NavigateToEntrance_SendGoal_Response__Sequence
{
  go2_msgs__action__NavigateToEntrance_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} go2_msgs__action__NavigateToEntrance_GetResult_Request;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_GetResult_Request.
typedef struct go2_msgs__action__NavigateToEntrance_GetResult_Request__Sequence
{
  go2_msgs__action__NavigateToEntrance_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "go2_msgs/action/detail/navigate_to_entrance__struct.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_GetResult_Response
{
  int8_t status;
  go2_msgs__action__NavigateToEntrance_Result result;
} go2_msgs__action__NavigateToEntrance_GetResult_Response;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_GetResult_Response.
typedef struct go2_msgs__action__NavigateToEntrance_GetResult_Response__Sequence
{
  go2_msgs__action__NavigateToEntrance_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "go2_msgs/action/detail/navigate_to_entrance__struct.h"

/// Struct defined in action/NavigateToEntrance in the package go2_msgs.
typedef struct go2_msgs__action__NavigateToEntrance_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  go2_msgs__action__NavigateToEntrance_Feedback feedback;
} go2_msgs__action__NavigateToEntrance_FeedbackMessage;

// Struct for a sequence of go2_msgs__action__NavigateToEntrance_FeedbackMessage.
typedef struct go2_msgs__action__NavigateToEntrance_FeedbackMessage__Sequence
{
  go2_msgs__action__NavigateToEntrance_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__NavigateToEntrance_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__STRUCT_H_
