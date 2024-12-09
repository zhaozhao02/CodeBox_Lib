// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aruco_msgs:msg/ArucoPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aruco_msgs/msg/detail/aruco_pose__rosidl_typesupport_introspection_c.h"
#include "aruco_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aruco_msgs/msg/detail/aruco_pose__functions.h"
#include "aruco_msgs/msg/detail/aruco_pose__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aruco_msgs__msg__ArucoPose__init(message_memory);
}

void aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_fini_function(void * message_memory)
{
  aruco_msgs__msg__ArucoPose__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_member_array[8] = {
  {
    "mark_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, mark_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "px",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, px),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "py",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, py),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, pz),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ox",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, ox),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "oy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, oy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "oz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, oz),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ow",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_msgs__msg__ArucoPose, ow),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_members = {
  "aruco_msgs__msg",  // message namespace
  "ArucoPose",  // message name
  8,  // number of fields
  sizeof(aruco_msgs__msg__ArucoPose),
  aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_member_array,  // message members
  aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_init_function,  // function to initialize message memory (memory has to be allocated)
  aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_type_support_handle = {
  0,
  &aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_msgs, msg, ArucoPose)() {
  if (!aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_type_support_handle.typesupport_identifier) {
    aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aruco_msgs__msg__ArucoPose__rosidl_typesupport_introspection_c__ArucoPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
