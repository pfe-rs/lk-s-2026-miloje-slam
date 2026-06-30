// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_msgs:msg/LidarSweep.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__BUILDER_HPP_
#define LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_msgs/msg/detail/lidar_sweep__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_msgs
{

namespace msg
{

namespace builder
{

class Init_LidarSweep_distances
{
public:
  explicit Init_LidarSweep_distances(::lidar_msgs::msg::LidarSweep & msg)
  : msg_(msg)
  {}
  ::lidar_msgs::msg::LidarSweep distances(::lidar_msgs::msg::LidarSweep::_distances_type arg)
  {
    msg_.distances = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_msgs::msg::LidarSweep msg_;
};

class Init_LidarSweep_angles
{
public:
  explicit Init_LidarSweep_angles(::lidar_msgs::msg::LidarSweep & msg)
  : msg_(msg)
  {}
  Init_LidarSweep_distances angles(::lidar_msgs::msg::LidarSweep::_angles_type arg)
  {
    msg_.angles = std::move(arg);
    return Init_LidarSweep_distances(msg_);
  }

private:
  ::lidar_msgs::msg::LidarSweep msg_;
};

class Init_LidarSweep_header
{
public:
  Init_LidarSweep_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarSweep_angles header(::lidar_msgs::msg::LidarSweep::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LidarSweep_angles(msg_);
  }

private:
  ::lidar_msgs::msg::LidarSweep msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_msgs::msg::LidarSweep>()
{
  return lidar_msgs::msg::builder::Init_LidarSweep_header();
}

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__BUILDER_HPP_
