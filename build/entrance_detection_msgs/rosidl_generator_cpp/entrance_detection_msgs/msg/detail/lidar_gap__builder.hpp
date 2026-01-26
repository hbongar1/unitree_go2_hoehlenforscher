// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from entrance_detection_msgs:msg/LidarGap.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__BUILDER_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "entrance_detection_msgs/msg/detail/lidar_gap__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace entrance_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_LidarGap_in_camera_fov
{
public:
  explicit Init_LidarGap_in_camera_fov(::entrance_detection_msgs::msg::LidarGap & msg)
  : msg_(msg)
  {}
  ::entrance_detection_msgs::msg::LidarGap in_camera_fov(::entrance_detection_msgs::msg::LidarGap::_in_camera_fov_type arg)
  {
    msg_.in_camera_fov = std::move(arg);
    return std::move(msg_);
  }

private:
  ::entrance_detection_msgs::msg::LidarGap msg_;
};

class Init_LidarGap_quality
{
public:
  explicit Init_LidarGap_quality(::entrance_detection_msgs::msg::LidarGap & msg)
  : msg_(msg)
  {}
  Init_LidarGap_in_camera_fov quality(::entrance_detection_msgs::msg::LidarGap::_quality_type arg)
  {
    msg_.quality = std::move(arg);
    return Init_LidarGap_in_camera_fov(msg_);
  }

private:
  ::entrance_detection_msgs::msg::LidarGap msg_;
};

class Init_LidarGap_center
{
public:
  explicit Init_LidarGap_center(::entrance_detection_msgs::msg::LidarGap & msg)
  : msg_(msg)
  {}
  Init_LidarGap_quality center(::entrance_detection_msgs::msg::LidarGap::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_LidarGap_quality(msg_);
  }

private:
  ::entrance_detection_msgs::msg::LidarGap msg_;
};

class Init_LidarGap_angle
{
public:
  explicit Init_LidarGap_angle(::entrance_detection_msgs::msg::LidarGap & msg)
  : msg_(msg)
  {}
  Init_LidarGap_center angle(::entrance_detection_msgs::msg::LidarGap::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_LidarGap_center(msg_);
  }

private:
  ::entrance_detection_msgs::msg::LidarGap msg_;
};

class Init_LidarGap_distance
{
public:
  explicit Init_LidarGap_distance(::entrance_detection_msgs::msg::LidarGap & msg)
  : msg_(msg)
  {}
  Init_LidarGap_angle distance(::entrance_detection_msgs::msg::LidarGap::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_LidarGap_angle(msg_);
  }

private:
  ::entrance_detection_msgs::msg::LidarGap msg_;
};

class Init_LidarGap_width
{
public:
  explicit Init_LidarGap_width(::entrance_detection_msgs::msg::LidarGap & msg)
  : msg_(msg)
  {}
  Init_LidarGap_distance width(::entrance_detection_msgs::msg::LidarGap::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_LidarGap_distance(msg_);
  }

private:
  ::entrance_detection_msgs::msg::LidarGap msg_;
};

class Init_LidarGap_header
{
public:
  Init_LidarGap_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarGap_width header(::entrance_detection_msgs::msg::LidarGap::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LidarGap_width(msg_);
  }

private:
  ::entrance_detection_msgs::msg::LidarGap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::entrance_detection_msgs::msg::LidarGap>()
{
  return entrance_detection_msgs::msg::builder::Init_LidarGap_header();
}

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__BUILDER_HPP_
