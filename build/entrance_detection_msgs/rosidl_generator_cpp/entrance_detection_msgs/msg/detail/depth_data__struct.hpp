// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from entrance_detection_msgs:msg/DepthData.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__STRUCT_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__STRUCT_HPP_

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
// Member 'depth_image'
// Member 'color_image'
#include "sensor_msgs/msg/detail/image__struct.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__entrance_detection_msgs__msg__DepthData __attribute__((deprecated))
#else
# define DEPRECATED__entrance_detection_msgs__msg__DepthData __declspec(deprecated)
#endif

namespace entrance_detection_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DepthData_
{
  using Type = DepthData_<ContainerAllocator>;

  explicit DepthData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    depth_image(_init),
    color_image(_init),
    camera_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->depth_scale = 0.0f;
    }
  }

  explicit DepthData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    depth_image(_alloc, _init),
    color_image(_alloc, _init),
    camera_info(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->depth_scale = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _depth_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _depth_image_type depth_image;
  using _color_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _color_image_type color_image;
  using _depth_scale_type =
    float;
  _depth_scale_type depth_scale;
  using _camera_info_type =
    sensor_msgs::msg::CameraInfo_<ContainerAllocator>;
  _camera_info_type camera_info;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__depth_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->depth_image = _arg;
    return *this;
  }
  Type & set__color_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->color_image = _arg;
    return *this;
  }
  Type & set__depth_scale(
    const float & _arg)
  {
    this->depth_scale = _arg;
    return *this;
  }
  Type & set__camera_info(
    const sensor_msgs::msg::CameraInfo_<ContainerAllocator> & _arg)
  {
    this->camera_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    entrance_detection_msgs::msg::DepthData_<ContainerAllocator> *;
  using ConstRawPtr =
    const entrance_detection_msgs::msg::DepthData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::DepthData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::DepthData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__entrance_detection_msgs__msg__DepthData
    std::shared_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__entrance_detection_msgs__msg__DepthData
    std::shared_ptr<entrance_detection_msgs::msg::DepthData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DepthData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->depth_image != other.depth_image) {
      return false;
    }
    if (this->color_image != other.color_image) {
      return false;
    }
    if (this->depth_scale != other.depth_scale) {
      return false;
    }
    if (this->camera_info != other.camera_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const DepthData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DepthData_

// alias to use template instance with default allocator
using DepthData =
  entrance_detection_msgs::msg::DepthData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__DEPTH_DATA__STRUCT_HPP_
