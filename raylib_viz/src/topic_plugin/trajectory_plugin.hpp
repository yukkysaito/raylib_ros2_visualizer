#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "topic_plugin_interface.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <iostream>

class TrajectoryPlugin : public TopicPluginInterface
{
public:
  TrajectoryPlugin(
    rclcpp::Node * node, const std::shared_ptr<FrameTree> frame_tree,
    const std::string & base_frame)
  : TopicPluginInterface(node, frame_tree, base_frame)
  {
    trajectory_mesh_ = std::make_unique<Mesh>();
    initTrajectoryMesh(trajectory_mesh_);
    trajectory_material_ = LoadMaterialDefault();
    trajectory_material_.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
  }
  ~TrajectoryPlugin()
  {
    UnloadMaterial(point_material_);
    UnloadMesh(*point_mesh_);
  }

  void init() override
  {
    subscription_ = node_->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "/planning/trajectory", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&TrajectoryPlugin::onTrajectory, this, std::placeholders::_1));
  }

  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    std::cout << "TrajectoryPlugin::onTrajectory()" << std::endl;
    buffer_.addData(msg);
  }

  void preprocess() override
  {
    const auto message = buffer_.getDataByTimestamp(
      std::chrono::system_clock::time_point(std::chrono::seconds(0)), false);
    if (message) {
      if (message->timestamp == uploaded_mesh_timestamp_) {
        return;
      }
      auto start = std::chrono::high_resolution_clock::now();

      if (uploaded_mesh_) UnloadMesh(*trajectory_mesh_);
      initTrajectoryMesh(trajectory_mesh_);
      generateTrajectoryMesh(trajectory_mesh_, message->data);
      UploadMesh(point_mesh_.get(), true);
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      std::cout << "PointCloud processing time: " << duration.count() << " ms" << std::endl;

      uploaded_mesh_ = true;
      uploaded_mesh_timestamp_ = message->timestamp;
    }
  }

  void visualize3D() override
  {
    if (uploaded_mesh_) {
      DrawMesh(*trajectory_mesh_, point_material_, MatrixTranslate(0, 0, 0));
    }
  }

  void visualizeOverlay2D() override {}

private:
  MessageBuffer<autoware_auto_planning_msgs::msg::Trajectory::SharedPtr> buffer_;
  rclcpp::Subscription<sautoware_auto_planning_msgs::msg::Trajectory>::SharedPtr subscription_;
  std::chrono::system_clock::time_point uploaded_mesh_timestamp_;
  std::unique_ptr<Mesh> point_mesh_;
  Material trajectory_material_;
  bool uploaded_mesh_ = false;
  void initTrajectoryMesh(const std::unique_ptr<Mesh> & mesh)
  {
    mesh->triangleCount = 0;
    mesh->texcoords = NULL;
    mesh->texcoords2 = NULL;
    mesh->normals = NULL;
    mesh->tangents = NULL;
    mesh->colors = NULL;
    mesh->indices = NULL;
    mesh->animVertices = NULL;
    mesh->animNormals = NULL;
    mesh->boneIds = NULL;
    mesh->boneWeights = NULL;
    mesh->vaoId = 0;
    mesh->vboId = NULL;
    mesh->vertexCount = 0;
    mesh->vertices = NULL;
  }

  void generatePointCloudMesh(
    const std::unique_ptr<Mesh> & mesh, const sensor_msgs::msg::PointCloud2::SharedPtr data)
  {
    mesh->vertices = (float *)RL_MALLOC(points_count * 3 * sizeof(float));
  }
};
