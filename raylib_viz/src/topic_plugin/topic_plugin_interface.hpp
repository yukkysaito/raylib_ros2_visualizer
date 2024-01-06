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
  std::string topic_name_;

public:
  TopicPluginInterface(
    rclcpp::Node * node, const std::shared_ptr<FrameTree> frame_tree,
    const std::string & base_frame, const std::string & topic_name)
  : node_(node), frame_tree_(frame_tree), base_frame_(base_frame), topic_name_(topic_name)
  {
  }
  virtual ~TopicPluginInterface() {}
  virtual void visualize3D() = 0;
  virtual void init(){};
  virtual void preprocess(){};
  virtual void visualizeOverlay2D(){};
};
