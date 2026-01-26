// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from entrance_detection_msgs:msg/DepthData.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__TRAITS_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "entrance_detection_msgs/msg/detail/depth_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'depth_image'
// Member 'color_image'
#include "sensor_msgs/msg/detail/image__traits.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__traits.hpp"

namespace entrance_detection_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DepthData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: depth_image
  {
    out << "depth_image: ";
    to_flow_style_yaml(msg.depth_image, out);
    out << ", ";
  }

  // member: color_image
  {
    out << "color_image: ";
    to_flow_style_yaml(msg.color_image, out);
    out << ", ";
  }

  // member: depth_scale
  {
    out << "depth_scale: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_scale, out);
    out << ", ";
  }

  // member: camera_info
  {
    out << "camera_info: ";
    to_flow_style_yaml(msg.camera_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DepthData & msg,
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

  // member: depth_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_image:\n";
    to_block_style_yaml(msg.depth_image, out, indentation + 2);
  }

  // member: color_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_image:\n";
    to_block_style_yaml(msg.color_image, out, indentation + 2);
  }

  // member: depth_scale
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_scale: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_scale, out);
    out << "\n";
  }

  // member: camera_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_info:\n";
    to_block_style_yaml(msg.camera_info, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DepthData & msg, bool use_flow_style = false)
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
  const entrance_detection_msgs::msg::DepthData & msg,
  std::ostream & out, size_t indentation = 0)
{
  entrance_detection_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use entrance_detection_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const entrance_detection_msgs::msg::DepthData & msg)
{
  return entrance_detection_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<entrance_detection_msgs::msg::DepthData>()
{
  return "entrance_detection_msgs::msg::DepthData";
}

template<>
inline const char * name<entrance_detection_msgs::msg::DepthData>()
{
  return "entrance_detection_msgs/msg/DepthData";
}

template<>
struct has_fixed_size<entrance_detection_msgs::msg::DepthData>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::CameraInfo>::value && has_fixed_size<sensor_msgs::msg::Image>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<entrance_detection_msgs::msg::DepthData>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::CameraInfo>::value && has_bounded_size<sensor_msgs::msg::Image>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<entrance_detection_msgs::msg::DepthData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__TRAITS_HPP_
