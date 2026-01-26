// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from entrance_detection_msgs:msg/DepthData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "entrance_detection_msgs/msg/detail/depth_data__rosidl_typesupport_introspection_c.h"
#include "entrance_detection_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "entrance_detection_msgs/msg/detail/depth_data__functions.h"
#include "entrance_detection_msgs/msg/detail/depth_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `depth_image`
// Member `color_image`
#include "sensor_msgs/msg/image.h"
// Member `depth_image`
// Member `color_image`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `camera_info`
#include "sensor_msgs/msg/camera_info.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  entrance_detection_msgs__msg__DepthData__init(message_memory);
}

void entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_fini_function(void * message_memory)
{
  entrance_detection_msgs__msg__DepthData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__DepthData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "depth_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__DepthData, depth_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "color_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__DepthData, color_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "depth_scale",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__DepthData, depth_scale),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "camera_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__DepthData, camera_info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_members = {
  "entrance_detection_msgs__msg",  // message namespace
  "DepthData",  // message name
  5,  // number of fields
  sizeof(entrance_detection_msgs__msg__DepthData),
  entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_member_array,  // message members
  entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_init_function,  // function to initialize message memory (memory has to be allocated)
  entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_type_support_handle = {
  0,
  &entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_entrance_detection_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, entrance_detection_msgs, msg, DepthData)() {
  entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CameraInfo)();
  if (!entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_type_support_handle.typesupport_identifier) {
    entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &entrance_detection_msgs__msg__DepthData__rosidl_typesupport_introspection_c__DepthData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
