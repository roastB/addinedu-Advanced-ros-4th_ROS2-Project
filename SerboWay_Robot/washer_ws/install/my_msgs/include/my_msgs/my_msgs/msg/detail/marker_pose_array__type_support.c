// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_msgs:msg/MarkerPoseArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_msgs/msg/detail/marker_pose_array__rosidl_typesupport_introspection_c.h"
#include "my_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_msgs/msg/detail/marker_pose_array__functions.h"
#include "my_msgs/msg/detail/marker_pose_array__struct.h"


// Include directives for member types
// Member `markers`
#include "my_msgs/msg/marker_pose.h"
// Member `markers`
#include "my_msgs/msg/detail/marker_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_msgs__msg__MarkerPoseArray__init(message_memory);
}

void my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_fini_function(void * message_memory)
{
  my_msgs__msg__MarkerPoseArray__fini(message_memory);
}

size_t my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__size_function__MarkerPoseArray__markers(
  const void * untyped_member)
{
  const my_msgs__msg__MarkerPose__Sequence * member =
    (const my_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  return member->size;
}

const void * my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__get_const_function__MarkerPoseArray__markers(
  const void * untyped_member, size_t index)
{
  const my_msgs__msg__MarkerPose__Sequence * member =
    (const my_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__get_function__MarkerPoseArray__markers(
  void * untyped_member, size_t index)
{
  my_msgs__msg__MarkerPose__Sequence * member =
    (my_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  return &member->data[index];
}

void my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__fetch_function__MarkerPoseArray__markers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const my_msgs__msg__MarkerPose * item =
    ((const my_msgs__msg__MarkerPose *)
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__get_const_function__MarkerPoseArray__markers(untyped_member, index));
  my_msgs__msg__MarkerPose * value =
    (my_msgs__msg__MarkerPose *)(untyped_value);
  *value = *item;
}

void my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__assign_function__MarkerPoseArray__markers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  my_msgs__msg__MarkerPose * item =
    ((my_msgs__msg__MarkerPose *)
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__get_function__MarkerPoseArray__markers(untyped_member, index));
  const my_msgs__msg__MarkerPose * value =
    (const my_msgs__msg__MarkerPose *)(untyped_value);
  *item = *value;
}

bool my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__resize_function__MarkerPoseArray__markers(
  void * untyped_member, size_t size)
{
  my_msgs__msg__MarkerPose__Sequence * member =
    (my_msgs__msg__MarkerPose__Sequence *)(untyped_member);
  my_msgs__msg__MarkerPose__Sequence__fini(member);
  return my_msgs__msg__MarkerPose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_member_array[1] = {
  {
    "markers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_msgs__msg__MarkerPoseArray, markers),  // bytes offset in struct
    NULL,  // default value
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__size_function__MarkerPoseArray__markers,  // size() function pointer
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__get_const_function__MarkerPoseArray__markers,  // get_const(index) function pointer
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__get_function__MarkerPoseArray__markers,  // get(index) function pointer
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__fetch_function__MarkerPoseArray__markers,  // fetch(index, &value) function pointer
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__assign_function__MarkerPoseArray__markers,  // assign(index, value) function pointer
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__resize_function__MarkerPoseArray__markers  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_members = {
  "my_msgs__msg",  // message namespace
  "MarkerPoseArray",  // message name
  1,  // number of fields
  sizeof(my_msgs__msg__MarkerPoseArray),
  false,  // has_any_key_member_
  my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_member_array,  // message members
  my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_init_function,  // function to initialize message memory (memory has to be allocated)
  my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_type_support_handle = {
  0,
  &my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_members,
  get_message_typesupport_handle_function,
  &my_msgs__msg__MarkerPoseArray__get_type_hash,
  &my_msgs__msg__MarkerPoseArray__get_type_description,
  &my_msgs__msg__MarkerPoseArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_msgs, msg, MarkerPoseArray)() {
  my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_msgs, msg, MarkerPose)();
  if (!my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_type_support_handle.typesupport_identifier) {
    my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_msgs__msg__MarkerPoseArray__rosidl_typesupport_introspection_c__MarkerPoseArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
