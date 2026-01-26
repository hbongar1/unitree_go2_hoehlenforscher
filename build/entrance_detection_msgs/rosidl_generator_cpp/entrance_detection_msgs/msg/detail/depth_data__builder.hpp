// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from entrance_detection_msgs:msg/DepthData.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__BUILDER_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "entrance_detection_msgs/msg/detail/depth_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace entrance_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_DepthData_camera_info
{
public:
  explicit Init_DepthData_camera_info(::entrance_detection_msgs::msg::DepthData & msg)
  : msg_(msg)
  {}
  ::entrance_detection_msgs::msg::DepthData camera_info(::entrance_detection_msgs::msg::DepthData::_camera_info_type arg)
  {
    msg_.camera_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::entrance_detection_msgs::msg::DepthData msg_;
};

class Init_DepthData_depth_scale
{
public:
  explicit Init_DepthData_depth_scale(::entrance_detection_msgs::msg::DepthData & msg)
  : msg_(msg)
  {}
  Init_DepthData_camera_info depth_scale(::entrance_detection_msgs::msg::DepthData::_depth_scale_type arg)
  {
    msg_.depth_scale = std::move(arg);
    return Init_DepthData_camera_info(msg_);
  }

private:
  ::entrance_detection_msgs::msg::DepthData msg_;
};

class Init_DepthData_color_image
{
public:
  explicit Init_DepthData_color_image(::entrance_detection_msgs::msg::DepthData & msg)
  : msg_(msg)
  {}
  Init_DepthData_depth_scale color_image(::entrance_detection_msgs::msg::DepthData::_color_image_type arg)
  {
    msg_.color_image = std::move(arg);
    return Init_DepthData_depth_scale(msg_);
  }

private:
  ::entrance_detection_msgs::msg::DepthData msg_;
};

class Init_DepthData_depth_image
{
public:
  explicit Init_DepthData_depth_image(::entrance_detection_msgs::msg::DepthData & msg)
  : msg_(msg)
  {}
  Init_DepthData_color_image depth_image(::entrance_detection_msgs::msg::DepthData::_depth_image_type arg)
  {
    msg_.depth_image = std::move(arg);
    return Init_DepthData_color_image(msg_);
  }

private:
  ::entrance_detection_msgs::msg::DepthData msg_;
};

class Init_DepthData_header
{
public:
  Init_DepthData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DepthData_depth_image header(::entrance_detection_msgs::msg::DepthData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DepthData_depth_image(msg_);
  }

private:
  ::entrance_detection_msgs::msg::DepthData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::entrance_detection_msgs::msg::DepthData>()
{
  return entrance_detection_msgs::msg::builder::Init_DepthData_header();
}

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__BUILDER_HPP_
