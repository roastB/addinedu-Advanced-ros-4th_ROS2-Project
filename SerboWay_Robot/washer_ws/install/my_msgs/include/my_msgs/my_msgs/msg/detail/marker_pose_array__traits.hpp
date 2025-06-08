// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_msgs:msg/MarkerPoseArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose_array.hpp"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__TRAITS_HPP_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_msgs/msg/detail/marker_pose_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'markers'
#include "my_msgs/msg/detail/marker_pose__traits.hpp"

namespace my_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MarkerPoseArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: markers
  {
    if (msg.markers.size() == 0) {
      out << "markers: []";
    } else {
      out << "markers: [";
      size_t pending_items = msg.markers.size();
      for (auto item : msg.markers) {
        to_flow_style_yaml(item, out);
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
  const MarkerPoseArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: markers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.markers.size() == 0) {
      out << "markers: []\n";
    } else {
      out << "markers:\n";
      for (auto item : msg.markers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MarkerPoseArray & msg, bool use_flow_style = false)
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
  const my_msgs::msg::MarkerPoseArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_msgs::msg::MarkerPoseArray & msg)
{
  return my_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_msgs::msg::MarkerPoseArray>()
{
  return "my_msgs::msg::MarkerPoseArray";
}

template<>
inline const char * name<my_msgs::msg::MarkerPoseArray>()
{
  return "my_msgs/msg/MarkerPoseArray";
}

template<>
struct has_fixed_size<my_msgs::msg::MarkerPoseArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_msgs::msg::MarkerPoseArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_msgs::msg::MarkerPoseArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__TRAITS_HPP_
