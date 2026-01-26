// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from entrance_detection_msgs:msg/EntranceEvent.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__TRAITS_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "entrance_detection_msgs/msg/detail/entrance_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position_3d'
// Member 'position_2d'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace entrance_detection_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EntranceEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position_3d
  {
    out << "position_3d: ";
    to_flow_style_yaml(msg.position_3d, out);
    out << ", ";
  }

  // member: position_2d
  {
    out << "position_2d: ";
    to_flow_style_yaml(msg.position_2d, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: is_passable
  {
    out << "is_passable: ";
    rosidl_generator_traits::value_to_yaml(msg.is_passable, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EntranceEvent & msg,
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

  // member: position_3d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_3d:\n";
    to_block_style_yaml(msg.position_3d, out, indentation + 2);
  }

  // member: position_2d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_2d:\n";
    to_block_style_yaml(msg.position_2d, out, indentation + 2);
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: is_passable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_passable: ";
    rosidl_generator_traits::value_to_yaml(msg.is_passable, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EntranceEvent & msg, bool use_flow_style = false)
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
  const entrance_detection_msgs::msg::EntranceEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  entrance_detection_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use entrance_detection_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const entrance_detection_msgs::msg::EntranceEvent & msg)
{
  return entrance_detection_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<entrance_detection_msgs::msg::EntranceEvent>()
{
  return "entrance_detection_msgs::msg::EntranceEvent";
}

template<>
inline const char * name<entrance_detection_msgs::msg::EntranceEvent>()
{
  return "entrance_detection_msgs/msg/EntranceEvent";
}

template<>
struct has_fixed_size<entrance_detection_msgs::msg::EntranceEvent>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<entrance_detection_msgs::msg::EntranceEvent>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<entrance_detection_msgs::msg::EntranceEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__TRAITS_HPP_
