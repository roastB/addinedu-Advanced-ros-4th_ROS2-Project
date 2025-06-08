// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:msg/MarkerPoseArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose_array.hpp"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__BUILDER_HPP_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_msgs/msg/detail/marker_pose_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_msgs
{

namespace msg
{

namespace builder
{

class Init_MarkerPoseArray_markers
{
public:
  Init_MarkerPoseArray_markers()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_msgs::msg::MarkerPoseArray markers(::my_msgs::msg::MarkerPoseArray::_markers_type arg)
  {
    msg_.markers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::msg::MarkerPoseArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::msg::MarkerPoseArray>()
{
  return my_msgs::msg::builder::Init_MarkerPoseArray_markers();
}

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__BUILDER_HPP_
