// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_msgs/msg/detail/order_information__rosidl_typesupport_introspection_c.h"
#include "my_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_msgs/msg/detail/order_information__functions.h"
#include "my_msgs/msg/detail/order_information__struct.h"


// Include directives for member types
// Member `ingredients`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_msgs__msg__OrderInformation__init(message_memory);
}

void my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_fini_function(void * message_memory)
{
  my_msgs__msg__OrderInformation__fini(message_memory);
}

size_t my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__size_function__OrderInformation__ingredients(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__get_const_function__OrderInformation__ingredients(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String * member =
    (const rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void * my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__get_function__OrderInformation__ingredients(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String * member =
    (rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__fetch_function__OrderInformation__ingredients(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__get_const_function__OrderInformation__ingredients(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__assign_function__OrderInformation__ingredients(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__get_function__OrderInformation__ingredients(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_member_array[2] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_msgs__msg__OrderInformation, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ingredients",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(my_msgs__msg__OrderInformation, ingredients),  // bytes offset in struct
    NULL,  // default value
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__size_function__OrderInformation__ingredients,  // size() function pointer
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__get_const_function__OrderInformation__ingredients,  // get_const(index) function pointer
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__get_function__OrderInformation__ingredients,  // get(index) function pointer
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__fetch_function__OrderInformation__ingredients,  // fetch(index, &value) function pointer
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__assign_function__OrderInformation__ingredients,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_members = {
  "my_msgs__msg",  // message namespace
  "OrderInformation",  // message name
  2,  // number of fields
  sizeof(my_msgs__msg__OrderInformation),
  false,  // has_any_key_member_
  my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_member_array,  // message members
  my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_init_function,  // function to initialize message memory (memory has to be allocated)
  my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_type_support_handle = {
  0,
  &my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_members,
  get_message_typesupport_handle_function,
  &my_msgs__msg__OrderInformation__get_type_hash,
  &my_msgs__msg__OrderInformation__get_type_description,
  &my_msgs__msg__OrderInformation__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_msgs, msg, OrderInformation)() {
  if (!my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_type_support_handle.typesupport_identifier) {
    my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_msgs__msg__OrderInformation__rosidl_typesupport_introspection_c__OrderInformation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
