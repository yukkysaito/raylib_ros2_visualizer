#pragma once

#include <rclcpp/rclcpp.hpp>

class RaylibViz : public rclcpp::Node
{
public:
  explicit RaylibViz(const rclcpp::NodeOptions & options);
  ~RaylibViz();
};
