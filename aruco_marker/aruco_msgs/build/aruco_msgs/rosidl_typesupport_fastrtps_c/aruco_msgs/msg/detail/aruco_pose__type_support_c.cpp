// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from aruco_msgs:msg/ArucoPose.idl
// generated code does not contain a copyright notice
#include "aruco_msgs/msg/detail/aruco_pose__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "aruco_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "aruco_msgs/msg/detail/aruco_pose__struct.h"
#include "aruco_msgs/msg/detail/aruco_pose__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ArucoPose__ros_msg_type = aruco_msgs__msg__ArucoPose;

static bool _ArucoPose__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ArucoPose__ros_msg_type * ros_message = static_cast<const _ArucoPose__ros_msg_type *>(untyped_ros_message);
  // Field name: mark_id
  {
    cdr << ros_message->mark_id;
  }

  // Field name: px
  {
    cdr << ros_message->px;
  }

  // Field name: py
  {
    cdr << ros_message->py;
  }

  // Field name: pz
  {
    cdr << ros_message->pz;
  }

  // Field name: ox
  {
    cdr << ros_message->ox;
  }

  // Field name: oy
  {
    cdr << ros_message->oy;
  }

  // Field name: oz
  {
    cdr << ros_message->oz;
  }

  // Field name: ow
  {
    cdr << ros_message->ow;
  }

  return true;
}

static bool _ArucoPose__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ArucoPose__ros_msg_type * ros_message = static_cast<_ArucoPose__ros_msg_type *>(untyped_ros_message);
  // Field name: mark_id
  {
    cdr >> ros_message->mark_id;
  }

  // Field name: px
  {
    cdr >> ros_message->px;
  }

  // Field name: py
  {
    cdr >> ros_message->py;
  }

  // Field name: pz
  {
    cdr >> ros_message->pz;
  }

  // Field name: ox
  {
    cdr >> ros_message->ox;
  }

  // Field name: oy
  {
    cdr >> ros_message->oy;
  }

  // Field name: oz
  {
    cdr >> ros_message->oz;
  }

  // Field name: ow
  {
    cdr >> ros_message->ow;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_msgs
size_t get_serialized_size_aruco_msgs__msg__ArucoPose(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ArucoPose__ros_msg_type * ros_message = static_cast<const _ArucoPose__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name mark_id
  {
    size_t item_size = sizeof(ros_message->mark_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name px
  {
    size_t item_size = sizeof(ros_message->px);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name py
  {
    size_t item_size = sizeof(ros_message->py);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pz
  {
    size_t item_size = sizeof(ros_message->pz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ox
  {
    size_t item_size = sizeof(ros_message->ox);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name oy
  {
    size_t item_size = sizeof(ros_message->oy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name oz
  {
    size_t item_size = sizeof(ros_message->oz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ow
  {
    size_t item_size = sizeof(ros_message->ow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ArucoPose__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_aruco_msgs__msg__ArucoPose(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_aruco_msgs
size_t max_serialized_size_aruco_msgs__msg__ArucoPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: mark_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: px
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: py
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: pz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ox
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: oy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: oz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = aruco_msgs__msg__ArucoPose;
    is_plain =
      (
      offsetof(DataType, ow) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ArucoPose__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_aruco_msgs__msg__ArucoPose(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ArucoPose = {
  "aruco_msgs::msg",
  "ArucoPose",
  _ArucoPose__cdr_serialize,
  _ArucoPose__cdr_deserialize,
  _ArucoPose__get_serialized_size,
  _ArucoPose__max_serialized_size
};

static rosidl_message_type_support_t _ArucoPose__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ArucoPose,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, aruco_msgs, msg, ArucoPose)() {
  return &_ArucoPose__type_support;
}

#if defined(__cplusplus)
}
#endif
