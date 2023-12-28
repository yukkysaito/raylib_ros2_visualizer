#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

class TopicPluginInterface
{
protected:
  rclcpp::Node * node_;

public:
  TopicPluginInterface(rclcpp::Node * node) : node_(node) {}
  virtual ~TopicPluginInterface() {}
  virtual void visualize3D() = 0;
  virtual void init(){};
  virtual void preprocess(){};
  virtual void visualizeOverlay2D(){};
};
