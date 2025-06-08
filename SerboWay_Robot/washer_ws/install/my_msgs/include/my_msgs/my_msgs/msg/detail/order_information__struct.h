// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/order_information.h"


#ifndef MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__STRUCT_H_
#define MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'ingredients'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/OrderInformation in the package my_msgs.
typedef struct my_msgs__msg__OrderInformation
{
  int32_t id;
  rosidl_runtime_c__String ingredients[4];
} my_msgs__msg__OrderInformation;

// Struct for a sequence of my_msgs__msg__OrderInformation.
typedef struct my_msgs__msg__OrderInformation__Sequence
{
  my_msgs__msg__OrderInformation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_msgs__msg__OrderInformation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_MSGS__MSG__DETAIL__ORDER_INFORMATION__STRUCT_H_
