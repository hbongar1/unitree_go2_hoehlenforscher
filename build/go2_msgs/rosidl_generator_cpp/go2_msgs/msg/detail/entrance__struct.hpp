// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from go2_msgs:msg/Entrance.idl
// generated code does not contain a copyright notice

#ifndef GO2_MSGS__MSG__DETAIL__ENTRANCE__STRUCT_HPP_
#define GO2_MSGS__MSG__DETAIL__ENTRANCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__go2_msgs__msg__Entrance __attribute__((deprecated))
#else
# define DEPRECATED__go2_msgs__msg__Entrance __declspec(deprecated)
#endif

namespace go2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Entrance_
{
  using Type = Entrance_<ContainerAllocator>;

  explicit Entrance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->height = 0.0f;
    }
  }

  explicit Entrance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->height = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _width_type =
    float;
  _width_type width;
  using _height_type =
    float;
  _height_type height;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    go2_msgs::msg::Entrance_<ContainerAllocator> *;
  using ConstRawPtr =
    const go2_msgs::msg::Entrance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<go2_msgs::msg::Entrance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<go2_msgs::msg::Entrance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      go2_msgs::msg::Entrance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<go2_msgs::msg::Entrance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      go2_msgs::msg::Entrance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<go2_msgs::msg::Entrance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<go2_msgs::msg::Entrance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<go2_msgs::msg::Entrance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__go2_msgs__msg__Entrance
    std::shared_ptr<go2_msgs::msg::Entrance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__go2_msgs__msg__Entrance
    std::shared_ptr<go2_msgs::msg::Entrance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Entrance_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    return true;
  }
  bool operator!=(const Entrance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Entrance_

// alias to use template instance with default allocator
using Entrance =
  go2_msgs::msg::Entrance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace go2_msgs

#endif  // GO2_MSGS__MSG__DETAIL__ENTRANCE__STRUCT_HPP_
