#pragma once
#include "camera_player.hpp"
#include "frame_tree.hpp"
#include "raylib.h"
#include "topic_plugin/topic_plugin_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

class Viewer3D
{
private:
  rclcpp::Node * node_;
  std::shared_ptr<FrameTree> frame_tree_;
  CameraPlayer camera_player_;
  std::vector<std::unique_ptr<TopicPluginInterface>> topic_plugins_;
  template <typename T>
  std::unique_ptr<T> createPlugin();
  std::string base_frame_ = "map";

public:
  Viewer3D(rclcpp::Node * node);
  ~Viewer3D();
  void visualize();
  void setBaseFrame(const std::string & base_frame);
  std::string getBaseFrame();
};
