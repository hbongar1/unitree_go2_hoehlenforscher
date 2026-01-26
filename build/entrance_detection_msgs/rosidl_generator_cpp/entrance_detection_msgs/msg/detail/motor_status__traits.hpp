// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from entrance_detection_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "entrance_detection_msgs/msg/detail/motor_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace entrance_detection_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: current_body_height
  {
    out << "current_body_height: ";
    rosidl_generator_traits::value_to_yaml(msg.current_body_height, out);
    out << ", ";
  }

  // member: posture_mode
  {
    out << "posture_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.posture_mode, out);
    out << ", ";
  }

  // member: in_motion
  {
    out << "in_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.in_motion, out);
    out << ", ";
  }

  // member: ready
  {
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorStatus & msg,
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

  // member: current_body_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_body_height: ";
    rosidl_generator_traits::value_to_yaml(msg.current_body_height, out);
    out << "\n";
  }

  // member: posture_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "posture_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.posture_mode, out);
    out << "\n";
  }

  // member: in_motion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.in_motion, out);
    out << "\n";
  }

  // member: ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << "\n";
  }

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorStatus & msg, bool use_flow_style = false)
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
  const entrance_detection_msgs::msg::MotorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  entrance_detection_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use entrance_detection_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const entrance_detection_msgs::msg::MotorStatus & msg)
{
  return entrance_detection_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<entrance_detection_msgs::msg::MotorStatus>()
{
  return "entrance_detection_msgs::msg::MotorStatus";
}

template<>
inline const char * name<entrance_detection_msgs::msg::MotorStatus>()
{
  return "entrance_detection_msgs/msg/MotorStatus";
}

template<>
struct has_fixed_size<entrance_detection_msgs::msg::MotorStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<entrance_detection_msgs::msg::MotorStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<entrance_detection_msgs::msg::MotorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_
