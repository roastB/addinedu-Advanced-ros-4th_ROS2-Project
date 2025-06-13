// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/order_information.hpp"


#ifndef MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__TRAITS_HPP_
#define MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_msgs/msg/detail/order_information__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrderInformation & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: ingredients
  {
    if (msg.ingredients.size() == 0) {
      out << "ingredients: []";
    } else {
      out << "ingredients: [";
      size_t pending_items = msg.ingredients.size();
      for (auto item : msg.ingredients) {
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
  const OrderInformation & msg,
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

  // member: ingredients
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ingredients.size() == 0) {
      out << "ingredients: []\n";
    } else {
      out << "ingredients:\n";
      for (auto item : msg.ingredients) {
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

inline std::string to_yaml(const OrderInformation & msg, bool use_flow_style = false)
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
  const my_msgs::msg::OrderInformation & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_msgs::msg::OrderInformation & msg)
{
  return my_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_msgs::msg::OrderInformation>()
{
  return "my_msgs::msg::OrderInformation";
}

template<>
inline const char * name<my_msgs::msg::OrderInformation>()
{
  return "my_msgs/msg/OrderInformation";
}

template<>
struct has_fixed_size<my_msgs::msg::OrderInformation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_msgs::msg::OrderInformation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_msgs::msg::OrderInformation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__TRAITS_HPP_
