#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <optional>
#include <string>

class FrameTree
{
public:
  // Constructor: Initializes the TF listener and buffer using a ROS2 node
  explicit FrameTree(rclcpp::Node * node);

  // Returns the transformation matrix between two frames at a specified time point
  std::optional<Eigen::Matrix4d> getTransform(
    const std::string & from_frame, const std::string & to_frame,
    const std::chrono::time_point<std::chrono::system_clock> & time_point);
  std::optional<Eigen::Matrix4d> getTransform(
    const std::string & from_frame, const std::string & to_frame,
    const std::chrono::time_point<std::chrono::system_clock> & time_point,
    const std::chrono::duration<double> & timeout);

private:
  rclcpp::Node * node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
