// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from entrance_detection_msgs:msg/EntranceEvent.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "entrance_detection_msgs/msg/detail/entrance_event__rosidl_typesupport_introspection_c.h"
#include "entrance_detection_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "entrance_detection_msgs/msg/detail/entrance_event__functions.h"
#include "entrance_detection_msgs/msg/detail/entrance_event__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position_3d`
// Member `position_2d`
#include "geometry_msgs/msg/point.h"
// Member `position_3d`
// Member `position_2d`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  entrance_detection_msgs__msg__EntranceEvent__init(message_memory);
}

void entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_fini_function(void * message_memory)
{
  entrance_detection_msgs__msg__EntranceEvent__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_3d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, position_3d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_2d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, position_2d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_passable",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceEvent, is_passable),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_members = {
  "entrance_detection_msgs__msg",  // message namespace
  "EntranceEvent",  // message name
  8,  // number of fields
  sizeof(entrance_detection_msgs__msg__EntranceEvent),
  entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_member_array,  // message members
  entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_init_function,  // function to initialize message memory (memory has to be allocated)
  entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_type_support_handle = {
  0,
  &entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_entrance_detection_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, entrance_detection_msgs, msg, EntranceEvent)() {
  entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_type_support_handle.typesupport_identifier) {
    entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &entrance_detection_msgs__msg__EntranceEvent__rosidl_typesupport_introspection_c__EntranceEvent_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
