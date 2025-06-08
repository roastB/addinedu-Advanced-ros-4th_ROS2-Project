// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/marker_pose.h"


#ifndef MY_MSGS__MSG__DETAIL__MARKER_POSE__STRUCT_H_
#define MY_MSGS__MSG__DETAIL__MARKER_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MarkerPose in the package my_msgs.
typedef struct my_msgs__msg__MarkerPose
{
  int32_t id;
  double pose_of_6d[6];
} my_msgs__msg__MarkerPose;

// Struct for a sequence of my_msgs__msg__MarkerPose.
typedef struct my_msgs__msg__MarkerPose__Sequence
{
  my_msgs__msg__MarkerPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__msg__MarkerPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__MSG__DETAIL__MARKER_POSE__STRUCT_H_
