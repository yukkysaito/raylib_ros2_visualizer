#include "viewer3d.hpp"

#include "raymath.h"
#include "topic_plugin/topic_plugin.hpp"

#include <iostream>
Viewer3D::Viewer3D(rclcpp::Node * node)
: node_(node), frame_tree_(std::make_shared<FrameTree>(node)), camera_player_(frame_tree_)
{
  topic_plugins_.push_back(createPlugin<PointCloudPlugin>());
  topic_plugins_.push_back(createPlugin<ObjectsPlugin>());
}

Viewer3D::~Viewer3D()
{
}

void Viewer3D::visualize()
{
  camera_player_.updateCamera();

  for (const auto & topic_plugin : topic_plugins_) {
    topic_plugin->preprocess();
  }

  BeginDrawing();
  ClearBackground(RAYWHITE);

  BeginMode3D(camera_player_.getCamera());

  for (const auto & topic_plugin : topic_plugins_) {
    topic_plugin->visualize3D();
  }

  DrawGrid(10, 10.0f);
  EndMode3D();

  for (const auto & topic_plugin : topic_plugins_) {
    topic_plugin->visualizeOverlay2D();
  }

  // camera_player_.drawCameraInfo();
  DrawFPS(10, 10);
  EndDrawing();
}

template <typename T>
std::unique_ptr<T> Viewer3D::createPlugin()
{
  auto plugin = std::make_unique<T>(node_, frame_tree_);
  plugin->init();
  return plugin;
}
