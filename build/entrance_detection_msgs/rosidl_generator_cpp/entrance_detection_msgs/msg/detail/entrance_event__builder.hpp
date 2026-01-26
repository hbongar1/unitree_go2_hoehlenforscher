// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from entrance_detection_msgs:msg/EntranceEvent.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__BUILDER_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "entrance_detection_msgs/msg/detail/entrance_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace entrance_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_EntranceEvent_is_passable
{
public:
  explicit Init_EntranceEvent_is_passable(::entrance_detection_msgs::msg::EntranceEvent & msg)
  : msg_(msg)
  {}
  ::entrance_detection_msgs::msg::EntranceEvent is_passable(::entrance_detection_msgs::msg::EntranceEvent::_is_passable_type arg)
  {
    msg_.is_passable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

class Init_EntranceEvent_confidence
{
public:
  explicit Init_EntranceEvent_confidence(::entrance_detection_msgs::msg::EntranceEvent & msg)
  : msg_(msg)
  {}
  Init_EntranceEvent_is_passable confidence(::entrance_detection_msgs::msg::EntranceEvent::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_EntranceEvent_is_passable(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

class Init_EntranceEvent_distance
{
public:
  explicit Init_EntranceEvent_distance(::entrance_detection_msgs::msg::EntranceEvent & msg)
  : msg_(msg)
  {}
  Init_EntranceEvent_confidence distance(::entrance_detection_msgs::msg::EntranceEvent::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_EntranceEvent_confidence(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

class Init_EntranceEvent_height
{
public:
  explicit Init_EntranceEvent_height(::entrance_detection_msgs::msg::EntranceEvent & msg)
  : msg_(msg)
  {}
  Init_EntranceEvent_distance height(::entrance_detection_msgs::msg::EntranceEvent::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_EntranceEvent_distance(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

class Init_EntranceEvent_width
{
public:
  explicit Init_EntranceEvent_width(::entrance_detection_msgs::msg::EntranceEvent & msg)
  : msg_(msg)
  {}
  Init_EntranceEvent_height width(::entrance_detection_msgs::msg::EntranceEvent::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_EntranceEvent_height(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

class Init_EntranceEvent_position_2d
{
public:
  explicit Init_EntranceEvent_position_2d(::entrance_detection_msgs::msg::EntranceEvent & msg)
  : msg_(msg)
  {}
  Init_EntranceEvent_width position_2d(::entrance_detection_msgs::msg::EntranceEvent::_position_2d_type arg)
  {
    msg_.position_2d = std::move(arg);
    return Init_EntranceEvent_width(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

class Init_EntranceEvent_position_3d
{
public:
  explicit Init_EntranceEvent_position_3d(::entrance_detection_msgs::msg::EntranceEvent & msg)
  : msg_(msg)
  {}
  Init_EntranceEvent_position_2d position_3d(::entrance_detection_msgs::msg::EntranceEvent::_position_3d_type arg)
  {
    msg_.position_3d = std::move(arg);
    return Init_EntranceEvent_position_2d(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

class Init_EntranceEvent_header
{
public:
  Init_EntranceEvent_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EntranceEvent_position_3d header(::entrance_detection_msgs::msg::EntranceEvent::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EntranceEvent_position_3d(msg_);
  }

private:
  ::entrance_detection_msgs::msg::EntranceEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::entrance_detection_msgs::msg::EntranceEvent>()
{
  return entrance_detection_msgs::msg::builder::Init_EntranceEvent_header();
}

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__BUILDER_HPP_
