// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:msg/MarkerPoseArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose_array.h"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__STRUCT_H_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'markers'
#include "my_msgs/msg/detail/marker_pose__struct.h"

/// Struct defined in msg/MarkerPoseArray in the package my_msgs.
typedef struct my_msgs__msg__MarkerPoseArray
{
  my_msgs__msg__MarkerPose__Sequence markers;
} my_msgs__msg__MarkerPoseArray;

// Struct for a sequence of my_msgs__msg__MarkerPoseArray.
typedef struct my_msgs__msg__MarkerPoseArray__Sequence
{
  my_msgs__msg__MarkerPoseArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__msg__MarkerPoseArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE_ARRAY__STRUCT_H_
