// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_msgs:msg/MarkerPose.idl
// generated code does not contain a copyright notice

#include "my_msgs/msg/detail/marker_pose__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__msg__MarkerPose__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf2, 0xeb, 0x5a, 0xc7, 0x6a, 0x49, 0xeb, 0x79,
      0x65, 0x9c, 0x84, 0x7a, 0x3c, 0x8e, 0x94, 0xd6,
      0x13, 0x6d, 0x96, 0x63, 0x55, 0x3a, 0x4c, 0xdd,
      0xf0, 0x1e, 0x9f, 0x25, 0xe1, 0x28, 0xbc, 0xf0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_msgs__msg__MarkerPose__TYPE_NAME[] = "my_msgs/msg/MarkerPose";

// Define type names, field names, and default values
static char my_msgs__msg__MarkerPose__FIELD_NAME__id[] = "id";
static char my_msgs__msg__MarkerPose__FIELD_NAME__pose_of_6d[] = "pose_of_6d";

static rosidl_runtime_c__type_description__Field my_msgs__msg__MarkerPose__FIELDS[] = {
  {
    {my_msgs__msg__MarkerPose__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_msgs__msg__MarkerPose__FIELD_NAME__pose_of_6d, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__msg__MarkerPose__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__msg__MarkerPose__TYPE_NAME, 22, 22},
      {my_msgs__msg__MarkerPose__FIELDS, 2, 2},
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
  "float64[6] pose_of_6d";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__msg__MarkerPose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__msg__MarkerPose__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 31, 31},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__msg__MarkerPose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__msg__MarkerPose__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
