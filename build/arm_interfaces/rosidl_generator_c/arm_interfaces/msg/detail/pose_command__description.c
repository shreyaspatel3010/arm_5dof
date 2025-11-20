// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from arm_interfaces:msg/PoseCommand.idl
// generated code does not contain a copyright notice

#include "arm_interfaces/msg/detail/pose_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_arm_interfaces
const rosidl_type_hash_t *
arm_interfaces__msg__PoseCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0x2f, 0xaf, 0x82, 0xea, 0x0d, 0x6d, 0x07,
      0x79, 0xed, 0x96, 0xd5, 0x2c, 0xac, 0xde, 0x5e,
      0xbf, 0x84, 0x6b, 0xb4, 0xe4, 0x28, 0x8d, 0x51,
      0x78, 0xc3, 0x4f, 0x20, 0xbd, 0x28, 0xba, 0x78,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char arm_interfaces__msg__PoseCommand__TYPE_NAME[] = "arm_interfaces/msg/PoseCommand";

// Define type names, field names, and default values
static char arm_interfaces__msg__PoseCommand__FIELD_NAME__x[] = "x";
static char arm_interfaces__msg__PoseCommand__FIELD_NAME__y[] = "y";
static char arm_interfaces__msg__PoseCommand__FIELD_NAME__z[] = "z";
static char arm_interfaces__msg__PoseCommand__FIELD_NAME__roll[] = "roll";
static char arm_interfaces__msg__PoseCommand__FIELD_NAME__pitch[] = "pitch";
static char arm_interfaces__msg__PoseCommand__FIELD_NAME__yaw[] = "yaw";
static char arm_interfaces__msg__PoseCommand__FIELD_NAME__cartesian_path[] = "cartesian_path";

static rosidl_runtime_c__type_description__Field arm_interfaces__msg__PoseCommand__FIELDS[] = {
  {
    {arm_interfaces__msg__PoseCommand__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arm_interfaces__msg__PoseCommand__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arm_interfaces__msg__PoseCommand__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arm_interfaces__msg__PoseCommand__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arm_interfaces__msg__PoseCommand__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arm_interfaces__msg__PoseCommand__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arm_interfaces__msg__PoseCommand__FIELD_NAME__cartesian_path, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arm_interfaces__msg__PoseCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arm_interfaces__msg__PoseCommand__TYPE_NAME, 30, 30},
      {arm_interfaces__msg__PoseCommand__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "float64 roll\n"
  "float64 pitch\n"
  "float64 yaw\n"
  "bool cartesian_path";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
arm_interfaces__msg__PoseCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arm_interfaces__msg__PoseCommand__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 88, 88},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arm_interfaces__msg__PoseCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arm_interfaces__msg__PoseCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
