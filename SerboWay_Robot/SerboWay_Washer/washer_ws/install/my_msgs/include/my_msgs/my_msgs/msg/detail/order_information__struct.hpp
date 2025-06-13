// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/order_information.hpp"


#ifndef MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__STRUCT_HPP_
#define MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_msgs__msg__OrderInformation __attribute__((deprecated))
#else
# define DEPRECATED__my_msgs__msg__OrderInformation __declspec(deprecated)
#endif

namespace my_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OrderInformation_
{
  using Type = OrderInformation_<ContainerAllocator>;

  explicit OrderInformation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 4>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->ingredients.begin(), this->ingredients.end(), "");
    }
  }

  explicit OrderInformation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ingredients(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 4>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->ingredients.begin(), this->ingredients.end(), "");
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _ingredients_type =
    std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 4>;
  _ingredients_type ingredients;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__ingredients(
    const std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 4> & _arg)
  {
    this->ingredients = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_msgs::msg::OrderInformation_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_msgs::msg::OrderInformation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::OrderInformation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::OrderInformation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_msgs__msg__OrderInformation
    std::shared_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_msgs__msg__OrderInformation
    std::shared_ptr<my_msgs::msg::OrderInformation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OrderInformation_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->ingredients != other.ingredients) {
      return false;
    }
    return true;
  }
  bool operator!=(const OrderInformation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OrderInformation_

// alias to use template instance with default allocator
using OrderInformation =
  my_msgs::msg::OrderInformation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__STRUCT_HPP_
