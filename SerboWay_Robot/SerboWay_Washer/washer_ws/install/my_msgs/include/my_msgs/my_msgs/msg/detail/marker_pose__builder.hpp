// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose.hpp"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE__BUILDER_HPP_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_msgs/msg/detail/marker_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_msgs
{

namespace msg
{

namespace builder
{

class Init_MarkerPose_pose_of_6d
{
public:
  explicit Init_MarkerPose_pose_of_6d(::my_msgs::msg::MarkerPose & msg)
  : msg_(msg)
  {}
  ::my_msgs::msg::MarkerPose pose_of_6d(::my_msgs::msg::MarkerPose::_pose_of_6d_type arg)
  {
    msg_.pose_of_6d = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::msg::MarkerPose msg_;
};

class Init_MarkerPose_id
{
public:
  Init_MarkerPose_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarkerPose_pose_of_6d id(::my_msgs::msg::MarkerPose::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MarkerPose_pose_of_6d(msg_);
  }

private:
  ::my_msgs::msg::MarkerPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::msg::MarkerPose>()
{
  return my_msgs::msg::builder::Init_MarkerPose_id();
}

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE__BUILDER_HPP_
