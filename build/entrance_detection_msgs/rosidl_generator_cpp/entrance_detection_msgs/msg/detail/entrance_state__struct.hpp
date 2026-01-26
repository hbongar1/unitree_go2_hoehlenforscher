// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from entrance_detection_msgs:msg/EntranceState.idl
// generated code does not contain a copyright notice

#ifndef ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__STRUCT_HPP_
#define ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__entrance_detection_msgs__msg__EntranceState __attribute__((deprecated))
#else
# define DEPRECATED__entrance_detection_msgs__msg__EntranceState __declspec(deprecated)
#endif

namespace entrance_detection_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EntranceState_
{
  using Type = EntranceState_<ContainerAllocator>;

  explicit EntranceState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->required_height = 0.0f;
      this->safety_clearance = 0.0f;
      this->action_required = false;
      this->description = "";
    }
  }

  explicit EntranceState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->required_height = 0.0f;
      this->safety_clearance = 0.0f;
      this->action_required = false;
      this->description = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _required_height_type =
    float;
  _required_height_type required_height;
  using _safety_clearance_type =
    float;
  _safety_clearance_type safety_clearance;
  using _action_required_type =
    bool;
  _action_required_type action_required;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__required_height(
    const float & _arg)
  {
    this->required_height = _arg;
    return *this;
  }
  Type & set__safety_clearance(
    const float & _arg)
  {
    this->safety_clearance = _arg;
    return *this;
  }
  Type & set__action_required(
    const bool & _arg)
  {
    this->action_required = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    entrance_detection_msgs::msg::EntranceState_<ContainerAllocator> *;
  using ConstRawPtr =
    const entrance_detection_msgs::msg::EntranceState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::EntranceState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      entrance_detection_msgs::msg::EntranceState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__entrance_detection_msgs__msg__EntranceState
    std::shared_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__entrance_detection_msgs__msg__EntranceState
    std::shared_ptr<entrance_detection_msgs::msg::EntranceState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EntranceState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->required_height != other.required_height) {
      return false;
    }
    if (this->safety_clearance != other.safety_clearance) {
      return false;
    }
    if (this->action_required != other.action_required) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const EntranceState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EntranceState_

// alias to use template instance with default allocator
using EntranceState =
  entrance_detection_msgs::msg::EntranceState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace entrance_detection_msgs

#endif  // ENTRANCE_DETECTION_MSGS__MSG__DETAIL__ENTRANCE_STATE__STRUCT_HPP_
