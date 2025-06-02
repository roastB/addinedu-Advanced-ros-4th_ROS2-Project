// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose.hpp"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE__TRAITS_HPP_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_msgs/msg/detail/marker_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MarkerPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: pose_of_6d
  {
    if (msg.pose_of_6d.size() == 0) {
      out << "pose_of_6d: []";
    } else {
      out << "pose_of_6d: [";
      size_t pending_items = msg.pose_of_6d.size();
      for (auto item : msg.pose_of_6d) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MarkerPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: pose_of_6d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose_of_6d.size() == 0) {
      out << "pose_of_6d: []\n";
    } else {
      out << "pose_of_6d:\n";
      for (auto item : msg.pose_of_6d) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MarkerPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace my_msgs

namespace rosidl_generator_traits
{

[[deprecated("use my_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_msgs::msg::MarkerPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_msgs::msg::MarkerPose & msg)
{
  return my_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_msgs::msg::MarkerPose>()
{
  return "my_msgs::msg::MarkerPose";
}

template<>
inline const char * name<my_msgs::msg::MarkerPose>()
{
  return "my_msgs/msg/MarkerPose";
}

template<>
struct has_fixed_size<my_msgs::msg::MarkerPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_msgs::msg::MarkerPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_msgs::msg::MarkerPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE__TRAITS_HPP_
