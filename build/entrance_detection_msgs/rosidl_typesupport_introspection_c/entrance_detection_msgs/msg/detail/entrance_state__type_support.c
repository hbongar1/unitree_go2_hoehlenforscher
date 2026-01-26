// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from entrance_detection_msgs:msg/EntranceState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "entrance_detection_msgs/msg/detail/entrance_state__rosidl_typesupport_introspection_c.h"
#include "entrance_detection_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "entrance_detection_msgs/msg/detail/entrance_state__functions.h"
#include "entrance_detection_msgs/msg/detail/entrance_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `state`
// Member `description`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  entrance_detection_msgs__msg__EntranceState__init(message_memory);
}

void entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_fini_function(void * message_memory)
{
  entrance_detection_msgs__msg__EntranceState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceState, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "required_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceState, required_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safety_clearance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceState, safety_clearance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "action_required",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceState, action_required),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(entrance_detection_msgs__msg__EntranceState, description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_members = {
  "entrance_detection_msgs__msg",  // message namespace
  "EntranceState",  // message name
  6,  // number of fields
  sizeof(entrance_detection_msgs__msg__EntranceState),
  entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_member_array,  // message members
  entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_init_function,  // function to initialize message memory (memory has to be allocated)
  entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_type_support_handle = {
  0,
  &entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_entrance_detection_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, entrance_detection_msgs, msg, EntranceState)() {
  entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_type_support_handle.typesupport_identifier) {
    entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &entrance_detection_msgs__msg__EntranceState__rosidl_typesupport_introspection_c__EntranceState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
