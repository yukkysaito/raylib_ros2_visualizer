#include "viewer3d.hpp"

#include "raymath.h"
#include "topic_plugin/topic_plugin.hpp"

#include <iostream>
Viewer3D::Viewer3D(rclcpp::Node * node)
: node_(node),
  frame_tree_(std::make_shared<FrameTree>(node)),
  camera_player_(frame_tree_, "base_link", base_frame_),
  grid_(std::make_unique<CartesianGrid>(frame_tree_, camera_player_.getViewerFrame(), base_frame_))
{
  topic_plugins_.push_back(
    createPlugin<PointCloudPlugin>("/perception/obstacle_segmentation/pointcloud"));
  topic_plugins_.push_back(createPlugin<ObjectsPlugin>("/perception/object_recognition/objects"));
  topic_plugins_.push_back(
    createPlugin<TrajectoryPlugin>("/planning/scenario_planning/trajectory"));
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

  grid_->drawGrid();

  EndMode3D();

  for (const auto & topic_plugin : topic_plugins_) {
    topic_plugin->visualizeOverlay2D();
  }

  // camera_player_.drawCameraInfo();
  DrawFPS(10, 10);
  EndDrawing();
}

template <typename T>
std::unique_ptr<T> Viewer3D::createPlugin(const std::string & topic_name)
{
  auto plugin = std::make_unique<T>(node_, frame_tree_, getBaseFrame(), topic_name);
  plugin->init();
  return plugin;
}

void Viewer3D::setBaseFrame(const std::string & base_frame)
{
  base_frame_ = base_frame;
}

std::string Viewer3D::getBaseFrame()
{
  return base_frame_;
}
