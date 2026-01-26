// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from entrance_detection_msgs:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "entrance_detection_msgs/msg/detail/motor_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace entrance_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorStatus_error_message
{
public:
  explicit Init_MotorStatus_error_message(::entrance_detection_msgs::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  ::entrance_detection_msgs::msg::MotorStatus error_message(::entrance_detection_msgs::msg::MotorStatus::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::entrance_detection_msgs::msg::MotorStatus msg_;
};

class Init_MotorStatus_ready
{
public:
  explicit Init_MotorStatus_ready(::entrance_detection_msgs::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_error_message ready(::entrance_detection_msgs::msg::MotorStatus::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return Init_MotorStatus_error_message(msg_);
  }

private:
  ::entrance_detection_msgs::msg::MotorStatus msg_;
};

class Init_MotorStatus_in_motion
{
public:
  explicit Init_MotorStatus_in_motion(::entrance_detection_msgs::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_ready in_motion(::entrance_detection_msgs::msg::MotorStatus::_in_motion_type arg)
  {
    msg_.in_motion = std::move(arg);
    return Init_MotorStatus_ready(msg_);
  }

private:
  ::entrance_detection_msgs::msg::MotorStatus msg_;
};

class Init_MotorStatus_posture_mode
{
public:
  explicit Init_MotorStatus_posture_mode(::entrance_detection_msgs::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_in_motion posture_mode(::entrance_detection_msgs::msg::MotorStatus::_posture_mode_type arg)
  {
    msg_.posture_mode = std::move(arg);
    return Init_MotorStatus_in_motion(msg_);
  }

private:
  ::entrance_detection_msgs::msg::MotorStatus msg_;
};

class Init_MotorStatus_current_body_height
{
public:
  explicit Init_MotorStatus_current_body_height(::entrance_detection_msgs::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_posture_mode current_body_height(::entrance_detection_msgs::msg::MotorStatus::_current_body_height_type arg)
  {
    msg_.current_body_height = std::move(arg);
    return Init_MotorStatus_posture_mode(msg_);
  }

private:
  ::entrance_detection_msgs::msg::MotorStatus msg_;
};

class Init_MotorStatus_header
{
public:
  Init_MotorStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorStatus_current_body_height header(::entrance_detection_msgs::msg::MotorStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorStatus_current_body_height(msg_);
  }

private:
  ::entrance_detection_msgs::msg::MotorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::entrance_detection_msgs::msg::MotorStatus>()
{
  return entrance_detection_msgs::msg::builder::Init_MotorStatus_header();
}

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_
