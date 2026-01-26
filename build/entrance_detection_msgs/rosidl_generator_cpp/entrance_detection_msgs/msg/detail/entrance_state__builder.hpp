// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from entrance_detection_msgs:msg/EntranceState.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__BUILDER_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "entrance_detection_msgs/msg/detail/entrance_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace entrance_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_EntranceState_description
{
public:
  explicit Init_EntranceState_description(::entrance_detection_msgs::msg::EntranceState & msg)
  : msg_(msg)
  {}
  ::entrance_detection_msgs::msg::EntranceState description(::entrance_detection_msgs::msg::EntranceState::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceState msg_;
};

class Init_EntranceState_action_required
{
public:
  explicit Init_EntranceState_action_required(::entrance_detection_msgs::msg::EntranceState & msg)
  : msg_(msg)
  {}
  Init_EntranceState_description action_required(::entrance_detection_msgs::msg::EntranceState::_action_required_type arg)
  {
    msg_.action_required = std::move(arg);
    return Init_EntranceState_description(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceState msg_;
};

class Init_EntranceState_safety_clearance
{
public:
  explicit Init_EntranceState_safety_clearance(::entrance_detection_msgs::msg::EntranceState & msg)
  : msg_(msg)
  {}
  Init_EntranceState_action_required safety_clearance(::entrance_detection_msgs::msg::EntranceState::_safety_clearance_type arg)
  {
    msg_.safety_clearance = std::move(arg);
    return Init_EntranceState_action_required(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceState msg_;
};

class Init_EntranceState_required_height
{
public:
  explicit Init_EntranceState_required_height(::entrance_detection_msgs::msg::EntranceState & msg)
  : msg_(msg)
  {}
  Init_EntranceState_safety_clearance required_height(::entrance_detection_msgs::msg::EntranceState::_required_height_type arg)
  {
    msg_.required_height = std::move(arg);
    return Init_EntranceState_safety_clearance(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceState msg_;
};

class Init_EntranceState_state
{
public:
  explicit Init_EntranceState_state(::entrance_detection_msgs::msg::EntranceState & msg)
  : msg_(msg)
  {}
  Init_EntranceState_required_height state(::entrance_detection_msgs::msg::EntranceState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_EntranceState_required_height(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceState msg_;
};

class Init_EntranceState_header
{
public:
  Init_EntranceState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EntranceState_state header(::entrance_detection_msgs::msg::EntranceState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EntranceState_state(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::entrance_detection_msgs::msg::EntranceState>()
{
  return entrance_detection_msgs::msg::builder::Init_EntranceState_header();
}

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__BUILDER_HPP_
