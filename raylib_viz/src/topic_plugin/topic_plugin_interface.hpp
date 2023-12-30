#pragma once

#include "../frame_tree.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

class TopicPluginInterface
{
protected:
  rclcpp::Node * node_;
  std::shared_ptr<FrameTree> frame_tree_;
  std::string base_frame_;

public:
  TopicPluginInterface(
    rclcpp::Node * node, const std::shared_ptr<FrameTree> frame_tree,
    const std::string & base_frame)
  : node_(node), frame_tree_(frame_tree), base_frame_(base_frame)
  {
  }
  virtual ~TopicPluginInterface() {}
  virtual void visualize3D() = 0;
  virtual void init(){};
  virtual void preprocess(){};
  virtual void visualizeOverlay2D(){};
};
