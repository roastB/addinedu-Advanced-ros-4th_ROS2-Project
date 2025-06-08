// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_msgs:msg/OrderInformation.idl
// generated code does not contain a copyright notice

#include "my_msgs/msg/detail/order_information__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__msg__OrderInformation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6e, 0xff, 0x56, 0xe0, 0x46, 0x3f, 0x7e, 0xf9,
      0xa4, 0x38, 0x27, 0x06, 0xed, 0x9a, 0xdc, 0xad,
      0xd7, 0xe7, 0x74, 0x2f, 0x1c, 0xe2, 0xb2, 0x34,
      0x76, 0xf3, 0x72, 0xbc, 0x43, 0x01, 0xbe, 0x87,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_msgs__msg__OrderInformation__TYPE_NAME[] = "my_msgs/msg/OrderInformation";

// Define type names, field names, and default values
static char my_msgs__msg__OrderInformation__FIELD_NAME__id[] = "id";
static char my_msgs__msg__OrderInformation__FIELD_NAME__ingredients[] = "ingredients";

static rosidl_runtime_c__type_description__Field my_msgs__msg__OrderInformation__FIELDS[] = {
  {
    {my_msgs__msg__OrderInformation__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__msg__OrderInformation__FIELD_NAME__ingredients, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__msg__OrderInformation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__msg__OrderInformation__TYPE_NAME, 28, 28},
      {my_msgs__msg__OrderInformation__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 id\n"
  "string[4] ingredients";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__msg__OrderInformation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__msg__OrderInformation__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 30, 30},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__msg__OrderInformation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__msg__OrderInformation__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
