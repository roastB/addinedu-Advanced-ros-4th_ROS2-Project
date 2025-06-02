// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/order_information.hpp"


#ifndef MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__BUILDER_HPP_
#define MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_msgs/msg/detail/order_information__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_msgs
{

namespace msg
{

namespace builder
{

class Init_OrderInformation_ingredients
{
public:
  explicit Init_OrderInformation_ingredients(::my_msgs::msg::OrderInformation & msg)
  : msg_(msg)
  {}
  ::my_msgs::msg::OrderInformation ingredients(::my_msgs::msg::OrderInformation::_ingredients_type arg)
  {
    msg_.ingredients = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::msg::OrderInformation msg_;
};

class Init_OrderInformation_id
{
public:
  Init_OrderInformation_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrderInformation_ingredients id(::my_msgs::msg::OrderInformation::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_OrderInformation_ingredients(msg_);
  }

private:
  ::my_msgs::msg::OrderInformation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::msg::OrderInformation>()
{
  return my_msgs::msg::builder::Init_OrderInformation_id();
}

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__BUILDER_HPP_
