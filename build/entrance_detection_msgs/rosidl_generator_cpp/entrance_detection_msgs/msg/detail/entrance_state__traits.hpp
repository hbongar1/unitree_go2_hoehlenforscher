// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from entrance_detection_msgs:msg/EntranceState.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__TRAITS_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "entrance_detection_msgs/msg/detail/entrance_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace entrance_detection_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EntranceState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: required_height
  {
    out << "required_height: ";
    rosidl_generator_traits::value_to_yaml(msg.required_height, out);
    out << ", ";
  }

  // member: safety_clearance
  {
    out << "safety_clearance: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_clearance, out);
    out << ", ";
  }

  // member: action_required
  {
    out << "action_required: ";
    rosidl_generator_traits::value_to_yaml(msg.action_required, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EntranceState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: required_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "required_height: ";
    rosidl_generator_traits::value_to_yaml(msg.required_height, out);
    out << "\n";
  }

  // member: safety_clearance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_clearance: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_clearance, out);
    out << "\n";
  }

  // member: action_required
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action_required: ";
    rosidl_generator_traits::value_to_yaml(msg.action_required, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EntranceState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace entrance_detection_msgs

namespace rosidl_generator_traits
{

[[deprecated("use entrance_detection_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const entrance_detection_msgs::msg::EntranceState & msg,
  std::ostream & out, size_t indentation = 0)
{
  entrance_detection_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use entrance_detection_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const entrance_detection_msgs::msg::EntranceState & msg)
{
  return entrance_detection_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<entrance_detection_msgs::msg::EntranceState>()
{
  return "entrance_detection_msgs::msg::EntranceState";
}

template<>
inline const char * name<entrance_detection_msgs::msg::EntranceState>()
{
  return "entrance_detection_msgs/msg/EntranceState";
}

template<>
struct has_fixed_size<entrance_detection_msgs::msg::EntranceState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<entrance_detection_msgs::msg::EntranceState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<entrance_detection_msgs::msg::EntranceState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__TRAITS_HPP_
