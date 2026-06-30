// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_msgs:msg/LidarSweep.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__STRUCT_HPP_
#define LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_msgs__msg__LidarSweep __attribute__((deprecated))
#else
# define DEPRECATED__lidar_msgs__msg__LidarSweep __declspec(deprecated)
#endif

namespace lidar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LidarSweep_
{
  using Type = LidarSweep_<ContainerAllocator>;

  explicit LidarSweep_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit LidarSweep_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _angles_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _angles_type angles;
  using _distances_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _distances_type distances;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__angles(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->angles = _arg;
    return *this;
  }
  Type & set__distances(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->distances = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_msgs::msg::LidarSweep_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_msgs::msg::LidarSweep_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::LidarSweep_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::LidarSweep_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_msgs__msg__LidarSweep
    std::shared_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_msgs__msg__LidarSweep
    std::shared_ptr<lidar_msgs::msg::LidarSweep_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarSweep_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->angles != other.angles) {
      return false;
    }
    if (this->distances != other.distances) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarSweep_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarSweep_

// alias to use template instance with default allocator
using LidarSweep =
  lidar_msgs::msg::LidarSweep_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__STRUCT_HPP_
