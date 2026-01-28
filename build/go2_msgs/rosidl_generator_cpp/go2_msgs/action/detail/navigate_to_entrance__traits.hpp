// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from go2_msgs:action/NavigateToEntrance.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__TRAITS_HPP_
#define GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "go2_msgs/action/detail/navigate_to_entrance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_position
  {
    out << "target_position: ";
    to_flow_style_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: entrance_width
  {
    out << "entrance_width: ";
    rosidl_generator_traits::value_to_yaml(msg.entrance_width, out);
    out << ", ";
  }

  // member: entrance_height
  {
    out << "entrance_height: ";
    rosidl_generator_traits::value_to_yaml(msg.entrance_height, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position:\n";
    to_block_style_yaml(msg.target_position, out, indentation + 2);
  }

  // member: entrance_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "entrance_width: ";
    rosidl_generator_traits::value_to_yaml(msg.entrance_width, out);
    out << "\n";
  }

  // member: entrance_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "entrance_height: ";
    rosidl_generator_traits::value_to_yaml(msg.entrance_height, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_Goal & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_Goal>()
{
  return "go2_msgs::action::NavigateToEntrance_Goal";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_Goal>()
{
  return "go2_msgs/action/NavigateToEntrance_Goal";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_Goal>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_Goal>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_Result & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_Result>()
{
  return "go2_msgs::action::NavigateToEntrance_Result";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_Result>()
{
  return "go2_msgs/action/NavigateToEntrance_Result";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: distance_remaining
  {
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << ", ";
  }

  // member: progress_percentage
  {
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: distance_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_remaining, out);
    out << "\n";
  }

  // member: progress_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percentage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_Feedback & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_Feedback>()
{
  return "go2_msgs::action::NavigateToEntrance_Feedback";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_Feedback>()
{
  return "go2_msgs/action/NavigateToEntrance_Feedback";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "go2_msgs/action/detail/navigate_to_entrance__traits.hpp"

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_SendGoal_Request & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_SendGoal_Request>()
{
  return "go2_msgs::action::NavigateToEntrance_SendGoal_Request";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_SendGoal_Request>()
{
  return "go2_msgs/action/NavigateToEntrance_SendGoal_Request";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<go2_msgs::action::NavigateToEntrance_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<go2_msgs::action::NavigateToEntrance_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_SendGoal_Response & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_SendGoal_Response>()
{
  return "go2_msgs::action::NavigateToEntrance_SendGoal_Response";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_SendGoal_Response>()
{
  return "go2_msgs/action/NavigateToEntrance_SendGoal_Response";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_SendGoal>()
{
  return "go2_msgs::action::NavigateToEntrance_SendGoal";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_SendGoal>()
{
  return "go2_msgs/action/NavigateToEntrance_SendGoal";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<go2_msgs::action::NavigateToEntrance_SendGoal_Request>::value &&
    has_fixed_size<go2_msgs::action::NavigateToEntrance_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<go2_msgs::action::NavigateToEntrance_SendGoal_Request>::value &&
    has_bounded_size<go2_msgs::action::NavigateToEntrance_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<go2_msgs::action::NavigateToEntrance_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<go2_msgs::action::NavigateToEntrance_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<go2_msgs::action::NavigateToEntrance_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_GetResult_Request & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_GetResult_Request>()
{
  return "go2_msgs::action::NavigateToEntrance_GetResult_Request";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_GetResult_Request>()
{
  return "go2_msgs/action/NavigateToEntrance_GetResult_Request";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "go2_msgs/action/detail/navigate_to_entrance__traits.hpp"

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_GetResult_Response & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_GetResult_Response>()
{
  return "go2_msgs::action::NavigateToEntrance_GetResult_Response";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_GetResult_Response>()
{
  return "go2_msgs/action/NavigateToEntrance_GetResult_Response";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<go2_msgs::action::NavigateToEntrance_Result>::value> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<go2_msgs::action::NavigateToEntrance_Result>::value> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_GetResult>()
{
  return "go2_msgs::action::NavigateToEntrance_GetResult";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_GetResult>()
{
  return "go2_msgs/action/NavigateToEntrance_GetResult";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<go2_msgs::action::NavigateToEntrance_GetResult_Request>::value &&
    has_fixed_size<go2_msgs::action::NavigateToEntrance_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<go2_msgs::action::NavigateToEntrance_GetResult_Request>::value &&
    has_bounded_size<go2_msgs::action::NavigateToEntrance_GetResult_Response>::value
  >
{
};

template<>
struct is_service<go2_msgs::action::NavigateToEntrance_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<go2_msgs::action::NavigateToEntrance_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<go2_msgs::action::NavigateToEntrance_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "go2_msgs/action/detail/navigate_to_entrance__traits.hpp"

namespace go2_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const NavigateToEntrance_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigateToEntrance_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigateToEntrance_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace go2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use go2_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const go2_msgs::action::NavigateToEntrance_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  go2_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use go2_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const go2_msgs::action::NavigateToEntrance_FeedbackMessage & msg)
{
  return go2_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<go2_msgs::action::NavigateToEntrance_FeedbackMessage>()
{
  return "go2_msgs::action::NavigateToEntrance_FeedbackMessage";
}

template<>
inline const char * name<go2_msgs::action::NavigateToEntrance_FeedbackMessage>()
{
  return "go2_msgs/action/NavigateToEntrance_FeedbackMessage";
}

template<>
struct has_fixed_size<go2_msgs::action::NavigateToEntrance_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<go2_msgs::action::NavigateToEntrance_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<go2_msgs::action::NavigateToEntrance_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<go2_msgs::action::NavigateToEntrance_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<go2_msgs::action::NavigateToEntrance_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<go2_msgs::action::NavigateToEntrance>
  : std::true_type
{
};

template<>
struct is_action_goal<go2_msgs::action::NavigateToEntrance_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<go2_msgs::action::NavigateToEntrance_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<go2_msgs::action::NavigateToEntrance_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // GO2_MSGS__ACTION__DETAIL__NAVIGATE_TO_ENTRANCE__TRAITS_HPP_
