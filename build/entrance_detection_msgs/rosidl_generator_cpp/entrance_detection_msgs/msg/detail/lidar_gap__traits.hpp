// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from entrance_detection_msgs:msg/LidarGap.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__TRAITS_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "entrance_detection_msgs/msg/detail/lidar_gap__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'center'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace entrance_detection_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LidarGap & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: center
  {
    out << "center: ";
    to_flow_style_yaml(msg.center, out);
    out << ", ";
  }

  // member: quality
  {
    out << "quality: ";
    rosidl_generator_traits::value_to_yaml(msg.quality, out);
    out << ", ";
  }

  // member: in_camera_fov
  {
    out << "in_camera_fov: ";
    rosidl_generator_traits::value_to_yaml(msg.in_camera_fov, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LidarGap & msg,
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

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
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

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center:\n";
    to_block_style_yaml(msg.center, out, indentation + 2);
  }

  // member: quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quality: ";
    rosidl_generator_traits::value_to_yaml(msg.quality, out);
    out << "\n";
  }

  // member: in_camera_fov
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_camera_fov: ";
    rosidl_generator_traits::value_to_yaml(msg.in_camera_fov, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LidarGap & msg, bool use_flow_style = false)
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
  const entrance_detection_msgs::msg::LidarGap & msg,
  std::ostream & out, size_t indentation = 0)
{
  entrance_detection_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use entrance_detection_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const entrance_detection_msgs::msg::LidarGap & msg)
{
  return entrance_detection_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<entrance_detection_msgs::msg::LidarGap>()
{
  return "entrance_detection_msgs::msg::LidarGap";
}

template<>
inline const char * name<entrance_detection_msgs::msg::LidarGap>()
{
  return "entrance_detection_msgs/msg/LidarGap";
}

template<>
struct has_fixed_size<entrance_detection_msgs::msg::LidarGap>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<entrance_detection_msgs::msg::LidarGap>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<entrance_detection_msgs::msg::LidarGap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__TRAITS_HPP_
