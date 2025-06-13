// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose.hpp"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE__STRUCT_HPP_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_msgs__msg__MarkerPose __attribute__((deprecated))
#else
# define DEPRECATED__my_msgs__msg__MarkerPose __declspec(deprecated)
#endif

namespace my_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MarkerPose_
{
  using Type = MarkerPose_<ContainerAllocator>;

  explicit MarkerPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<double, 6>::iterator, double>(this->pose_of_6d.begin(), this->pose_of_6d.end(), 0.0);
    }
  }

  explicit MarkerPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose_of_6d(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<double, 6>::iterator, double>(this->pose_of_6d.begin(), this->pose_of_6d.end(), 0.0);
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _pose_of_6d_type =
    std::array<double, 6>;
  _pose_of_6d_type pose_of_6d;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__pose_of_6d(
    const std::array<double, 6> & _arg)
  {
    this->pose_of_6d = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_msgs::msg::MarkerPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_msgs::msg::MarkerPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::MarkerPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::MarkerPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_msgs__msg__MarkerPose
    std::shared_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_msgs__msg__MarkerPose
    std::shared_ptr<my_msgs::msg::MarkerPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarkerPose_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->pose_of_6d != other.pose_of_6d) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarkerPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarkerPose_

// alias to use template instance with default allocator
using MarkerPose =
  my_msgs::msg::MarkerPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE__STRUCT_HPP_
