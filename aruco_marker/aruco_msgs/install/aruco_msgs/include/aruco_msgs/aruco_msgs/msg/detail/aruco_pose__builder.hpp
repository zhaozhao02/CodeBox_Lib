// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aruco_msgs:msg/ArucoPose.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__BUILDER_HPP_
#define ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aruco_msgs/msg/detail/aruco_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aruco_msgs
{

namespace msg
{

namespace builder
{

class Init_ArucoPose_ow
{
public:
  explicit Init_ArucoPose_ow(::aruco_msgs::msg::ArucoPose & msg)
  : msg_(msg)
  {}
  ::aruco_msgs::msg::ArucoPose ow(::aruco_msgs::msg::ArucoPose::_ow_type arg)
  {
    msg_.ow = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

class Init_ArucoPose_oz
{
public:
  explicit Init_ArucoPose_oz(::aruco_msgs::msg::ArucoPose & msg)
  : msg_(msg)
  {}
  Init_ArucoPose_ow oz(::aruco_msgs::msg::ArucoPose::_oz_type arg)
  {
    msg_.oz = std::move(arg);
    return Init_ArucoPose_ow(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

class Init_ArucoPose_oy
{
public:
  explicit Init_ArucoPose_oy(::aruco_msgs::msg::ArucoPose & msg)
  : msg_(msg)
  {}
  Init_ArucoPose_oz oy(::aruco_msgs::msg::ArucoPose::_oy_type arg)
  {
    msg_.oy = std::move(arg);
    return Init_ArucoPose_oz(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

class Init_ArucoPose_ox
{
public:
  explicit Init_ArucoPose_ox(::aruco_msgs::msg::ArucoPose & msg)
  : msg_(msg)
  {}
  Init_ArucoPose_oy ox(::aruco_msgs::msg::ArucoPose::_ox_type arg)
  {
    msg_.ox = std::move(arg);
    return Init_ArucoPose_oy(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

class Init_ArucoPose_pz
{
public:
  explicit Init_ArucoPose_pz(::aruco_msgs::msg::ArucoPose & msg)
  : msg_(msg)
  {}
  Init_ArucoPose_ox pz(::aruco_msgs::msg::ArucoPose::_pz_type arg)
  {
    msg_.pz = std::move(arg);
    return Init_ArucoPose_ox(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

class Init_ArucoPose_py
{
public:
  explicit Init_ArucoPose_py(::aruco_msgs::msg::ArucoPose & msg)
  : msg_(msg)
  {}
  Init_ArucoPose_pz py(::aruco_msgs::msg::ArucoPose::_py_type arg)
  {
    msg_.py = std::move(arg);
    return Init_ArucoPose_pz(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

class Init_ArucoPose_px
{
public:
  explicit Init_ArucoPose_px(::aruco_msgs::msg::ArucoPose & msg)
  : msg_(msg)
  {}
  Init_ArucoPose_py px(::aruco_msgs::msg::ArucoPose::_px_type arg)
  {
    msg_.px = std::move(arg);
    return Init_ArucoPose_py(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

class Init_ArucoPose_mark_id
{
public:
  Init_ArucoPose_mark_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArucoPose_px mark_id(::aruco_msgs::msg::ArucoPose::_mark_id_type arg)
  {
    msg_.mark_id = std::move(arg);
    return Init_ArucoPose_px(msg_);
  }

private:
  ::aruco_msgs::msg::ArucoPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aruco_msgs::msg::ArucoPose>()
{
  return aruco_msgs::msg::builder::Init_ArucoPose_mark_id();
}

}  // namespace aruco_msgs

#endif  // ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__BUILDER_HPP_
