// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_msgs:msg/MarkerPoseArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose_array.hpp"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__STRUCT_HPP_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'markers'
#include "my_msgs/msg/detail/marker_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_msgs__msg__MarkerPoseArray __attribute__((deprecated))
#else
# define DEPRECATED__my_msgs__msg__MarkerPoseArray __declspec(deprecated)
#endif

namespace my_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MarkerPoseArray_
{
  using Type = MarkerPoseArray_<ContainerAllocator>;

  explicit MarkerPoseArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MarkerPoseArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _markers_type =
    std::vector<my_msgs::msg::MarkerPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_msgs::msg::MarkerPose_<ContainerAllocator>>>;
  _markers_type markers;

  // setters for named parameter idiom
  Type & set__markers(
    const std::vector<my_msgs::msg::MarkerPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<my_msgs::msg::MarkerPose_<ContainerAllocator>>> & _arg)
  {
    this->markers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_msgs::msg::MarkerPoseArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_msgs::msg::MarkerPoseArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::MarkerPoseArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::MarkerPoseArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_msgs__msg__MarkerPoseArray
    std::shared_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_msgs__msg__MarkerPoseArray
    std::shared_ptr<my_msgs::msg::MarkerPoseArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarkerPoseArray_ & other) const
  {
    if (this->markers != other.markers) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarkerPoseArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarkerPoseArray_

// alias to use template instance with default allocator
using MarkerPoseArray =
  my_msgs::msg::MarkerPoseArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__STRUCT_HPP_
