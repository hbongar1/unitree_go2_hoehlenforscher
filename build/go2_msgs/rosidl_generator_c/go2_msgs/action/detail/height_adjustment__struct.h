// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from go2_msgs:action/HeightAdjustment.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__ACTION__DETAIL__HEIGHT_ADJUSTMENT__STRUCT_H_
#define GO2_MSGS__ACTION__DETAIL__HEIGHT_ADJUSTMENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_Goal
{
  bool passable;
  float required_height_adjustment;
} go2_msgs__action__HeightAdjustment_Goal;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_Goal.
typedef struct go2_msgs__action__HeightAdjustment_Goal__Sequence
{
  go2_msgs__action__HeightAdjustment_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_Result
{
  bool success;
  rosidl_runtime_c__String message;
} go2_msgs__action__HeightAdjustment_Result;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_Result.
typedef struct go2_msgs__action__HeightAdjustment_Result__Sequence
{
  go2_msgs__action__HeightAdjustment_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_Feedback
{
  rosidl_runtime_c__String status;
} go2_msgs__action__HeightAdjustment_Feedback;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_Feedback.
typedef struct go2_msgs__action__HeightAdjustment_Feedback__Sequence
{
  go2_msgs__action__HeightAdjustment_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "go2_msgs/action/detail/height_adjustment__struct.h"

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  go2_msgs__action__HeightAdjustment_Goal goal;
} go2_msgs__action__HeightAdjustment_SendGoal_Request;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_SendGoal_Request.
typedef struct go2_msgs__action__HeightAdjustment_SendGoal_Request__Sequence
{
  go2_msgs__action__HeightAdjustment_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} go2_msgs__action__HeightAdjustment_SendGoal_Response;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_SendGoal_Response.
typedef struct go2_msgs__action__HeightAdjustment_SendGoal_Response__Sequence
{
  go2_msgs__action__HeightAdjustment_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} go2_msgs__action__HeightAdjustment_GetResult_Request;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_GetResult_Request.
typedef struct go2_msgs__action__HeightAdjustment_GetResult_Request__Sequence
{
  go2_msgs__action__HeightAdjustment_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "go2_msgs/action/detail/height_adjustment__struct.h"

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_GetResult_Response
{
  int8_t status;
  go2_msgs__action__HeightAdjustment_Result result;
} go2_msgs__action__HeightAdjustment_GetResult_Response;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_GetResult_Response.
typedef struct go2_msgs__action__HeightAdjustment_GetResult_Response__Sequence
{
  go2_msgs__action__HeightAdjustment_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "go2_msgs/action/detail/height_adjustment__struct.h"

/// Struct defined in action/HeightAdjustment in the package go2_msgs.
typedef struct go2_msgs__action__HeightAdjustment_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  go2_msgs__action__HeightAdjustment_Feedback feedback;
} go2_msgs__action__HeightAdjustment_FeedbackMessage;

// Struct for a sequence of go2_msgs__action__HeightAdjustment_FeedbackMessage.
typedef struct go2_msgs__action__HeightAdjustment_FeedbackMessage__Sequence
{
  go2_msgs__action__HeightAdjustment_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_msgs__action__HeightAdjustment_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GO2_MSGS__ACTION__DETAIL__HEIGHT_ADJUSTMENT__STRUCT_H_
