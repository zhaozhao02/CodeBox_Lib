// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aruco_msgs:msg/ArucoPose.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__TRAITS_HPP_
#define ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aruco_msgs/msg/detail/aruco_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace aruco_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArucoPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: mark_id
  {
    out << "mark_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mark_id, out);
    out << ", ";
  }

  // member: px
  {
    out << "px: ";
    rosidl_generator_traits::value_to_yaml(msg.px, out);
    out << ", ";
  }

  // member: py
  {
    out << "py: ";
    rosidl_generator_traits::value_to_yaml(msg.py, out);
    out << ", ";
  }

  // member: pz
  {
    out << "pz: ";
    rosidl_generator_traits::value_to_yaml(msg.pz, out);
    out << ", ";
  }

  // member: ox
  {
    out << "ox: ";
    rosidl_generator_traits::value_to_yaml(msg.ox, out);
    out << ", ";
  }

  // member: oy
  {
    out << "oy: ";
    rosidl_generator_traits::value_to_yaml(msg.oy, out);
    out << ", ";
  }

  // member: oz
  {
    out << "oz: ";
    rosidl_generator_traits::value_to_yaml(msg.oz, out);
    out << ", ";
  }

  // member: ow
  {
    out << "ow: ";
    rosidl_generator_traits::value_to_yaml(msg.ow, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArucoPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mark_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mark_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mark_id, out);
    out << "\n";
  }

  // member: px
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "px: ";
    rosidl_generator_traits::value_to_yaml(msg.px, out);
    out << "\n";
  }

  // member: py
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "py: ";
    rosidl_generator_traits::value_to_yaml(msg.py, out);
    out << "\n";
  }

  // member: pz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pz: ";
    rosidl_generator_traits::value_to_yaml(msg.pz, out);
    out << "\n";
  }

  // member: ox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ox: ";
    rosidl_generator_traits::value_to_yaml(msg.ox, out);
    out << "\n";
  }

  // member: oy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "oy: ";
    rosidl_generator_traits::value_to_yaml(msg.oy, out);
    out << "\n";
  }

  // member: oz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "oz: ";
    rosidl_generator_traits::value_to_yaml(msg.oz, out);
    out << "\n";
  }

  // member: ow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ow: ";
    rosidl_generator_traits::value_to_yaml(msg.ow, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArucoPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace aruco_msgs

namespace rosidl_generator_traits
{

[[deprecated("use aruco_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aruco_msgs::msg::ArucoPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  aruco_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aruco_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const aruco_msgs::msg::ArucoPose & msg)
{
  return aruco_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<aruco_msgs::msg::ArucoPose>()
{
  return "aruco_msgs::msg::ArucoPose";
}

template<>
inline const char * name<aruco_msgs::msg::ArucoPose>()
{
  return "aruco_msgs/msg/ArucoPose";
}

template<>
struct has_fixed_size<aruco_msgs::msg::ArucoPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<aruco_msgs::msg::ArucoPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<aruco_msgs::msg::ArucoPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__TRAITS_HPP_
