// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from entrance_detection_msgs:msg/EntranceEvent.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__STRUCT_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__STRUCT_HPP_

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
// Member 'position_3d'
// Member 'position_2d'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__entrance_detection_msgs__msg__EntranceEvent __attribute__((deprecated))
#else
# define DEPRECATED__entrance_detection_msgs__msg__EntranceEvent __declspec(deprecated)
#endif

namespace entrance_detection_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EntranceEvent_
{
  using Type = EntranceEvent_<ContainerAllocator>;

  explicit EntranceEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position_3d(_init),
    position_2d(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->height = 0.0f;
      this->distance = 0.0f;
      this->confidence = 0.0f;
      this->is_passable = false;
    }
  }

  explicit EntranceEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position_3d(_alloc, _init),
    position_2d(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->height = 0.0f;
      this->distance = 0.0f;
      this->confidence = 0.0f;
      this->is_passable = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_3d_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_3d_type position_3d;
  using _position_2d_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_2d_type position_2d;
  using _width_type =
    float;
  _width_type width;
  using _height_type =
    float;
  _height_type height;
  using _distance_type =
    float;
  _distance_type distance;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _is_passable_type =
    bool;
  _is_passable_type is_passable;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position_3d(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position_3d = _arg;
    return *this;
  }
  Type & set__position_2d(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position_2d = _arg;
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
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__is_passable(
    const bool & _arg)
  {
    this->is_passable = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__entrance_detection_msgs__msg__EntranceEvent
    std::shared_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__entrance_detection_msgs__msg__EntranceEvent
    std::shared_ptr<entrance_detection_msgs::msg::EntranceEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EntranceEvent_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position_3d != other.position_3d) {
      return false;
    }
    if (this->position_2d != other.position_2d) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->is_passable != other.is_passable) {
      return false;
    }
    return true;
  }
  bool operator!=(const EntranceEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EntranceEvent_

// alias to use template instance with default allocator
using EntranceEvent =
  entrance_detection_msgs::msg::EntranceEvent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_EVENT__STRUCT_HPP_
