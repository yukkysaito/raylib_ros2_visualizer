#pragma once

#include "../frame_tree.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

class TopicPluginInterface
{
protected:
  rclcpp::Node * node_;
  std::shared_ptr<FrameTree> frame_tree_;

public:
  TopicPluginInterface(rclcpp::Node * node, const std::shared_ptr<FrameTree> frame_tree)
  : node_(node), frame_tree_(frame_tree)
  {
  }
  virtual ~TopicPluginInterface() {}
  virtual void visualize3D() = 0;
  virtual void init(){};
  virtual void preprocess(){};
  virtual void visualizeOverlay2D(){};
};
