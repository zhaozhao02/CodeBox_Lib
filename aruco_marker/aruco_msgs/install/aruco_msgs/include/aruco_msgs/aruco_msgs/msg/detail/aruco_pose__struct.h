// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aruco_msgs:msg/ArucoPose.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__STRUCT_H_
#define ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ArucoPose in the package aruco_msgs.
/**
  * ArucoPose.msg
 */
typedef struct aruco_msgs__msg__ArucoPose
{
  int32_t mark_id;
  double px;
  double py;
  double pz;
  double ox;
  double oy;
  double oz;
  double ow;
} aruco_msgs__msg__ArucoPose;

// Struct for a sequence of aruco_msgs__msg__ArucoPose.
typedef struct aruco_msgs__msg__ArucoPose__Sequence
{
  aruco_msgs__msg__ArucoPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aruco_msgs__msg__ArucoPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__STRUCT_H_
