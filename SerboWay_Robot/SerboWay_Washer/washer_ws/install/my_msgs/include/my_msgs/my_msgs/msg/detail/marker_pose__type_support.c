// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_msgs/msg/detail/marker_pose__rosidl_typesupport_introspection_c.h"
#include "my_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_msgs/msg/detail/marker_pose__functions.h"
#include "my_msgs/msg/detail/marker_pose__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_msgs__msg__MarkerPose__init(message_memory);
}

void my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_fini_function(void * message_memory)
{
  my_msgs__msg__MarkerPose__fini(message_memory);
}

size_t my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__size_function__MarkerPose__pose_of_6d(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__get_const_function__MarkerPose__pose_of_6d(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__get_function__MarkerPose__pose_of_6d(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__fetch_function__MarkerPose__pose_of_6d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__get_const_function__MarkerPose__pose_of_6d(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__assign_function__MarkerPose__pose_of_6d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__get_function__MarkerPose__pose_of_6d(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_member_array[2] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_msgs__msg__MarkerPose, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose_of_6d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(my_msgs__msg__MarkerPose, pose_of_6d),  // bytes offset in struct
    NULL,  // default value
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__size_function__MarkerPose__pose_of_6d,  // size() function pointer
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__get_const_function__MarkerPose__pose_of_6d,  // get_const(index) function pointer
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__get_function__MarkerPose__pose_of_6d,  // get(index) function pointer
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__fetch_function__MarkerPose__pose_of_6d,  // fetch(index, &value) function pointer
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__assign_function__MarkerPose__pose_of_6d,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_members = {
  "my_msgs__msg",  // message namespace
  "MarkerPose",  // message name
  2,  // number of fields
  sizeof(my_msgs__msg__MarkerPose),
  false,  // has_any_key_member_
  my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_member_array,  // message members
  my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_init_function,  // function to initialize message memory (memory has to be allocated)
  my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_type_support_handle = {
  0,
  &my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_members,
  get_message_typesupport_handle_function,
  &my_msgs__msg__MarkerPose__get_type_hash,
  &my_msgs__msg__MarkerPose__get_type_description,
  &my_msgs__msg__MarkerPose__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_msgs, msg, MarkerPose)() {
  if (!my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_type_support_handle.typesupport_identifier) {
    my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_msgs__msg__MarkerPose__rosidl_typesupport_introspection_c__MarkerPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
