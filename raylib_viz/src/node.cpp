#include "node.hpp"

#include "topic_plugin/topic_plugin.hpp"
#include "window.hpp"

RaylibViz::RaylibViz(const rclcpp::NodeOptions & options) : rclcpp::Node("raylib_viz", options)
{
  Window window(this);

  window.run([=]() { return WindowShouldClose() ? rclcpp::shutdown() : rclcpp::ok(); });
}

RaylibViz::~RaylibViz()
{
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(RaylibViz)
