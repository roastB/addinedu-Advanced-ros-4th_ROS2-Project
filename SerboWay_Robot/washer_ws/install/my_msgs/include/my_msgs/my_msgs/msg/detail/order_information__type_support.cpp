// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "my_msgs/msg/detail/order_information__functions.h"
#include "my_msgs/msg/detail/order_information__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace my_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void OrderInformation_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) my_msgs::msg::OrderInformation(_init);
}

void OrderInformation_fini_function(void * message_memory)
{
  auto typed_message = static_cast<my_msgs::msg::OrderInformation *>(message_memory);
  typed_message->~OrderInformation();
}

size_t size_function__OrderInformation__ingredients(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__OrderInformation__ingredients(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<std::string, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__OrderInformation__ingredients(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<std::string, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__OrderInformation__ingredients(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__OrderInformation__ingredients(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__OrderInformation__ingredients(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__OrderInformation__ingredients(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OrderInformation_message_member_array[2] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_msgs::msg::OrderInformation, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ingredients",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(my_msgs::msg::OrderInformation, ingredients),  // bytes offset in struct
    nullptr,  // default value
    size_function__OrderInformation__ingredients,  // size() function pointer
    get_const_function__OrderInformation__ingredients,  // get_const(index) function pointer
    get_function__OrderInformation__ingredients,  // get(index) function pointer
    fetch_function__OrderInformation__ingredients,  // fetch(index, &value) function pointer
    assign_function__OrderInformation__ingredients,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OrderInformation_message_members = {
  "my_msgs::msg",  // message namespace
  "OrderInformation",  // message name
  2,  // number of fields
  sizeof(my_msgs::msg::OrderInformation),
  false,  // has_any_key_member_
  OrderInformation_message_member_array,  // message members
  OrderInformation_init_function,  // function to initialize message memory (memory has to be allocated)
  OrderInformation_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OrderInformation_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OrderInformation_message_members,
  get_message_typesupport_handle_function,
  &my_msgs__msg__OrderInformation__get_type_hash,
  &my_msgs__msg__OrderInformation__get_type_description,
  &my_msgs__msg__OrderInformation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace my_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<my_msgs::msg::OrderInformation>()
{
  return &::my_msgs::msg::rosidl_typesupport_introspection_cpp::OrderInformation_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_msgs, msg, OrderInformation)() {
  return &::my_msgs::msg::rosidl_typesupport_introspection_cpp::OrderInformation_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
