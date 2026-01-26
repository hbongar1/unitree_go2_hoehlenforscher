// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from entrance_detection_msgs:msg/LidarGap.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__STRUCT_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__STRUCT_HPP_

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
// Member 'center'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__entrance_detection_msgs__msg__LidarGap __attribute__((deprecated))
#else
# define DEPRECATED__entrance_detection_msgs__msg__LidarGap __declspec(deprecated)
#endif

namespace entrance_detection_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LidarGap_
{
  using Type = LidarGap_<ContainerAllocator>;

  explicit LidarGap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    center(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->distance = 0.0f;
      this->angle = 0.0f;
      this->quality = 0.0f;
      this->in_camera_fov = false;
    }
  }

  explicit LidarGap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    center(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->distance = 0.0f;
      this->angle = 0.0f;
      this->quality = 0.0f;
      this->in_camera_fov = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _width_type =
    float;
  _width_type width;
  using _distance_type =
    float;
  _distance_type distance;
  using _angle_type =
    float;
  _angle_type angle;
  using _center_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _center_type center;
  using _quality_type =
    float;
  _quality_type quality;
  using _in_camera_fov_type =
    bool;
  _in_camera_fov_type in_camera_fov;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__center(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->center = _arg;
    return *this;
  }
  Type & set__quality(
    const float & _arg)
  {
    this->quality = _arg;
    return *this;
  }
  Type & set__in_camera_fov(
    const bool & _arg)
  {
    this->in_camera_fov = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    entrance_detection_msgs::msg::LidarGap_<ContainerAllocator> *;
  using ConstRawPtr =
    const entrance_detection_msgs::msg::LidarGap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::LidarGap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::LidarGap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__entrance_detection_msgs__msg__LidarGap
    std::shared_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__entrance_detection_msgs__msg__LidarGap
    std::shared_ptr<entrance_detection_msgs::msg::LidarGap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarGap_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->center != other.center) {
      return false;
    }
    if (this->quality != other.quality) {
      return false;
    }
    if (this->in_camera_fov != other.in_camera_fov) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarGap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarGap_

// alias to use template instance with default allocator
using LidarGap =
  entrance_detection_msgs::msg::LidarGap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__LIDAR_GAP__STRUCT_HPP_
