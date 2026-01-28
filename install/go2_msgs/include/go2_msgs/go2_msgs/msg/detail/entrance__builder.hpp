// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_msgs:msg/Entrance.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__MSG__DETAIL__ENTRANCE__BUILDER_HPP_
#define GO2_MSGS__MSG__DETAIL__ENTRANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "go2_msgs/msg/detail/entrance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace go2_msgs
{

namespace msg
{

namespace builder
{

class Init_Entrance_height
{
public:
  explicit Init_Entrance_height(::go2_msgs::msg::Entrance & msg)
  : msg_(msg)
  {}
  ::go2_msgs::msg::Entrance height(::go2_msgs::msg::Entrance::_height_type arg)
  {
    msg_.height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_msgs::msg::Entrance msg_;
};

class Init_Entrance_width
{
public:
  explicit Init_Entrance_width(::go2_msgs::msg::Entrance & msg)
  : msg_(msg)
  {}
  Init_Entrance_height width(::go2_msgs::msg::Entrance::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Entrance_height(msg_);
  }

private:
  ::go2_msgs::msg::Entrance msg_;
};

class Init_Entrance_position
{
public:
  explicit Init_Entrance_position(::go2_msgs::msg::Entrance & msg)
  : msg_(msg)
  {}
  Init_Entrance_width position(::go2_msgs::msg::Entrance::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Entrance_width(msg_);
  }

private:
  ::go2_msgs::msg::Entrance msg_;
};

class Init_Entrance_header
{
public:
  Init_Entrance_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Entrance_position header(::go2_msgs::msg::Entrance::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Entrance_position(msg_);
  }

private:
  ::go2_msgs::msg::Entrance msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_msgs::msg::Entrance>()
{
  return go2_msgs::msg::builder::Init_Entrance_header();
}

}  // namespace go2_msgs

#endif  // GO2_MSGS__MSG__DETAIL__ENTRANCE__BUILDER_HPP_
