// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arm_interfaces:msg/PoseCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arm_interfaces/msg/pose_command.h"


#ifndef ARM_INTERFACES__MSG__DETAIL__POSE_COMMAND__STRUCT_H_
#define ARM_INTERFACES__MSG__DETAIL__POSE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/PoseCommand in the package arm_interfaces.
typedef struct arm_interfaces__msg__PoseCommand
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  bool cartesian_path;
} arm_interfaces__msg__PoseCommand;

// Struct for a sequence of arm_interfaces__msg__PoseCommand.
typedef struct arm_interfaces__msg__PoseCommand__Sequence
{
  arm_interfaces__msg__PoseCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arm_interfaces__msg__PoseCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARM_INTERFACES__MSG__DETAIL__POSE_COMMAND__STRUCT_H_
