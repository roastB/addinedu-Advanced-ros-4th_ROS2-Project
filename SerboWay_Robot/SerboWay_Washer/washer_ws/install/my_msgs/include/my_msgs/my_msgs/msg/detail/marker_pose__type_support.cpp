// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "my_msgs/msg/detail/marker_pose__functions.h"
#include "my_msgs/msg/detail/marker_pose__struct.hpp"
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

void MarkerPose_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) my_msgs::msg::MarkerPose(_init);
}

void MarkerPose_fini_function(void * message_memory)
{
  auto typed_message = static_cast<my_msgs::msg::MarkerPose *>(message_memory);
  typed_message->~MarkerPose();
}

size_t size_function__MarkerPose__pose_of_6d(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__MarkerPose__pose_of_6d(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__MarkerPose__pose_of_6d(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__MarkerPose__pose_of_6d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MarkerPose__pose_of_6d(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MarkerPose__pose_of_6d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MarkerPose__pose_of_6d(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MarkerPose_message_member_array[2] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_msgs::msg::MarkerPose, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose_of_6d",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(my_msgs::msg::MarkerPose, pose_of_6d),  // bytes offset in struct
    nullptr,  // default value
    size_function__MarkerPose__pose_of_6d,  // size() function pointer
    get_const_function__MarkerPose__pose_of_6d,  // get_const(index) function pointer
    get_function__MarkerPose__pose_of_6d,  // get(index) function pointer
    fetch_function__MarkerPose__pose_of_6d,  // fetch(index, &value) function pointer
    assign_function__MarkerPose__pose_of_6d,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MarkerPose_message_members = {
  "my_msgs::msg",  // message namespace
  "MarkerPose",  // message name
  2,  // number of fields
  sizeof(my_msgs::msg::MarkerPose),
  false,  // has_any_key_member_
  MarkerPose_message_member_array,  // message members
  MarkerPose_init_function,  // function to initialize message memory (memory has to be allocated)
  MarkerPose_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MarkerPose_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MarkerPose_message_members,
  get_message_typesupport_handle_function,
  &my_msgs__msg__MarkerPose__get_type_hash,
  &my_msgs__msg__MarkerPose__get_type_description,
  &my_msgs__msg__MarkerPose__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace my_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<my_msgs::msg::MarkerPose>()
{
  return &::my_msgs::msg::rosidl_typesupport_introspection_cpp::MarkerPose_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_msgs, msg, MarkerPose)() {
  return &::my_msgs::msg::rosidl_typesupport_introspection_cpp::MarkerPose_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
