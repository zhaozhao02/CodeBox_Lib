// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from aruco_msgs:msg/ArucoPose.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__STRUCT_HPP_
#define ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__aruco_msgs__msg__ArucoPose __attribute__((deprecated))
#else
# define DEPRECATED__aruco_msgs__msg__ArucoPose __declspec(deprecated)
#endif

namespace aruco_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArucoPose_
{
  using Type = ArucoPose_<ContainerAllocator>;

  explicit ArucoPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mark_id = 0l;
      this->px = 0.0;
      this->py = 0.0;
      this->pz = 0.0;
      this->ox = 0.0;
      this->oy = 0.0;
      this->oz = 0.0;
      this->ow = 0.0;
    }
  }

  explicit ArucoPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mark_id = 0l;
      this->px = 0.0;
      this->py = 0.0;
      this->pz = 0.0;
      this->ox = 0.0;
      this->oy = 0.0;
      this->oz = 0.0;
      this->ow = 0.0;
    }
  }

  // field types and members
  using _mark_id_type =
    int32_t;
  _mark_id_type mark_id;
  using _px_type =
    double;
  _px_type px;
  using _py_type =
    double;
  _py_type py;
  using _pz_type =
    double;
  _pz_type pz;
  using _ox_type =
    double;
  _ox_type ox;
  using _oy_type =
    double;
  _oy_type oy;
  using _oz_type =
    double;
  _oz_type oz;
  using _ow_type =
    double;
  _ow_type ow;

  // setters for named parameter idiom
  Type & set__mark_id(
    const int32_t & _arg)
  {
    this->mark_id = _arg;
    return *this;
  }
  Type & set__px(
    const double & _arg)
  {
    this->px = _arg;
    return *this;
  }
  Type & set__py(
    const double & _arg)
  {
    this->py = _arg;
    return *this;
  }
  Type & set__pz(
    const double & _arg)
  {
    this->pz = _arg;
    return *this;
  }
  Type & set__ox(
    const double & _arg)
  {
    this->ox = _arg;
    return *this;
  }
  Type & set__oy(
    const double & _arg)
  {
    this->oy = _arg;
    return *this;
  }
  Type & set__oz(
    const double & _arg)
  {
    this->oz = _arg;
    return *this;
  }
  Type & set__ow(
    const double & _arg)
  {
    this->ow = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aruco_msgs::msg::ArucoPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const aruco_msgs::msg::ArucoPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aruco_msgs::msg::ArucoPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aruco_msgs::msg::ArucoPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aruco_msgs__msg__ArucoPose
    std::shared_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aruco_msgs__msg__ArucoPose
    std::shared_ptr<aruco_msgs::msg::ArucoPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArucoPose_ & other) const
  {
    if (this->mark_id != other.mark_id) {
      return false;
    }
    if (this->px != other.px) {
      return false;
    }
    if (this->py != other.py) {
      return false;
    }
    if (this->pz != other.pz) {
      return false;
    }
    if (this->ox != other.ox) {
      return false;
    }
    if (this->oy != other.oy) {
      return false;
    }
    if (this->oz != other.oz) {
      return false;
    }
    if (this->ow != other.ow) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArucoPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArucoPose_

// alias to use template instance with default allocator
using ArucoPose =
  aruco_msgs::msg::ArucoPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace aruco_msgs

#endif  // ARUCO_MSGS__MSG__DETAIL__ARUCO_POSE__STRUCT_HPP_
