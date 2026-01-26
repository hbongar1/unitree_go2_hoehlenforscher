// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from entrance_detection_msgs:msg/LidarGap.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__FUNCTIONS_H_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "entrance_detection_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "entrance_detection_msgs/msg/detail/lidar_gap__struct.h"

/// Initialize msg/LidarGap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * entrance_detection_msgs__msg__LidarGap
 * )) before or use
 * entrance_detection_msgs__msg__LidarGap__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
bool
entrance_detection_msgs__msg__LidarGap__init(entrance_detection_msgs__msg__LidarGap * msg);

/// Finalize msg/LidarGap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
void
entrance_detection_msgs__msg__LidarGap__fini(entrance_detection_msgs__msg__LidarGap * msg);

/// Create msg/LidarGap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * entrance_detection_msgs__msg__LidarGap__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
entrance_detection_msgs__msg__LidarGap *
entrance_detection_msgs__msg__LidarGap__create();

/// Destroy msg/LidarGap message.
/**
 * It calls
 * entrance_detection_msgs__msg__LidarGap__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
void
entrance_detection_msgs__msg__LidarGap__destroy(entrance_detection_msgs__msg__LidarGap * msg);

/// Check for msg/LidarGap message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
bool
entrance_detection_msgs__msg__LidarGap__are_equal(const entrance_detection_msgs__msg__LidarGap * lhs, const entrance_detection_msgs__msg__LidarGap * rhs);

/// Copy a msg/LidarGap message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
bool
entrance_detection_msgs__msg__LidarGap__copy(
  const entrance_detection_msgs__msg__LidarGap * input,
  entrance_detection_msgs__msg__LidarGap * output);

/// Initialize array of msg/LidarGap messages.
/**
 * It allocates the memory for the number of elements and calls
 * entrance_detection_msgs__msg__LidarGap__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
bool
entrance_detection_msgs__msg__LidarGap__Sequence__init(entrance_detection_msgs__msg__LidarGap__Sequence * array, size_t size);

/// Finalize array of msg/LidarGap messages.
/**
 * It calls
 * entrance_detection_msgs__msg__LidarGap__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
void
entrance_detection_msgs__msg__LidarGap__Sequence__fini(entrance_detection_msgs__msg__LidarGap__Sequence * array);

/// Create array of msg/LidarGap messages.
/**
 * It allocates the memory for the array and calls
 * entrance_detection_msgs__msg__LidarGap__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
entrance_detection_msgs__msg__LidarGap__Sequence *
entrance_detection_msgs__msg__LidarGap__Sequence__create(size_t size);

/// Destroy array of msg/LidarGap messages.
/**
 * It calls
 * entrance_detection_msgs__msg__LidarGap__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
void
entrance_detection_msgs__msg__LidarGap__Sequence__destroy(entrance_detection_msgs__msg__LidarGap__Sequence * array);

/// Check for msg/LidarGap message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
bool
entrance_detection_msgs__msg__LidarGap__Sequence__are_equal(const entrance_detection_msgs__msg__LidarGap__Sequence * lhs, const entrance_detection_msgs__msg__LidarGap__Sequence * rhs);

/// Copy an array of msg/LidarGap messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_entrance_detection_msgs
bool
entrance_detection_msgs__msg__LidarGap__Sequence__copy(
  const entrance_detection_msgs__msg__LidarGap__Sequence * input,
  entrance_detection_msgs__msg__LidarGap__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__FUNCTIONS_H_
