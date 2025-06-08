// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_msgs:msg/MarkerPoseArray.idl
// generated code does not contain a copyright notice

#include "my_msgs/msg/detail/marker_pose_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_msgs
const rosidl_type_hash_t *
my_msgs__msg__MarkerPoseArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1e, 0xb3, 0x00, 0x6d, 0x87, 0x5e, 0x69, 0x06,
      0x45, 0x1f, 0x95, 0x17, 0x27, 0x45, 0x14, 0x0e,
      0xbb, 0xd4, 0x10, 0xd2, 0x18, 0x94, 0xa8, 0x38,
      0xea, 0x28, 0x51, 0x0a, 0x3a, 0x0a, 0xed, 0x49,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "my_msgs/msg/detail/marker_pose__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t my_msgs__msg__MarkerPose__EXPECTED_HASH = {1, {
    0xf2, 0xeb, 0x5a, 0xc7, 0x6a, 0x49, 0xeb, 0x79,
    0x65, 0x9c, 0x84, 0x7a, 0x3c, 0x8e, 0x94, 0xd6,
    0x13, 0x6d, 0x96, 0x63, 0x55, 0x3a, 0x4c, 0xdd,
    0xf0, 0x1e, 0x9f, 0x25, 0xe1, 0x28, 0xbc, 0xf0,
  }};
#endif

static char my_msgs__msg__MarkerPoseArray__TYPE_NAME[] = "my_msgs/msg/MarkerPoseArray";
static char my_msgs__msg__MarkerPose__TYPE_NAME[] = "my_msgs/msg/MarkerPose";

// Define type names, field names, and default values
static char my_msgs__msg__MarkerPoseArray__FIELD_NAME__markers[] = "markers";

static rosidl_runtime_c__type_description__Field my_msgs__msg__MarkerPoseArray__FIELDS[] = {
  {
    {my_msgs__msg__MarkerPoseArray__FIELD_NAME__markers, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {my_msgs__msg__MarkerPose__TYPE_NAME, 22, 22},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription my_msgs__msg__MarkerPoseArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {my_msgs__msg__MarkerPose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_msgs__msg__MarkerPoseArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_msgs__msg__MarkerPoseArray__TYPE_NAME, 27, 27},
      {my_msgs__msg__MarkerPoseArray__FIELDS, 1, 1},
    },
    {my_msgs__msg__MarkerPoseArray__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&my_msgs__msg__MarkerPose__EXPECTED_HASH, my_msgs__msg__MarkerPose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = my_msgs__msg__MarkerPose__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "MarkerPose[] markers";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_msgs__msg__MarkerPoseArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_msgs__msg__MarkerPoseArray__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 21, 21},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_msgs__msg__MarkerPoseArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_msgs__msg__MarkerPoseArray__get_individual_type_description_source(NULL),
    sources[1] = *my_msgs__msg__MarkerPose__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
