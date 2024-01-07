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
    const std::string & base_frame, const std::string & topic_name)
  : TopicPluginInterface(node, frame_tree, base_frame, topic_name)
  {
    mesh_ = std::make_unique<Mesh>();
    initMesh(*mesh_);
    material_ = LoadMaterialDefault();
    material_.maps[MATERIAL_MAP_DIFFUSE].color = {0, 121, 241, 255};
  }
  ~TrajectoryPlugin()
  {
    UnloadMaterial(material_);
    UnloadMesh(*mesh_);
  }

  void init() override
  {
    subscription_ = node_->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      topic_name_, rclcpp::SensorDataQoS().keep_last(1),
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

      if (uploaded_mesh_) UnloadMesh(*mesh_);
      initMesh(*mesh_);
      generateTrajectoryMesh(mesh_, message->data);
      UploadMesh(mesh_.get(), true);
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      std::cout << "Trajectory processing time: " << duration.count() << " ms" << std::endl;

      uploaded_mesh_ = true;
      uploaded_mesh_timestamp_ = message->timestamp;
    }
  }

  void visualize3D() override
  {
    if (uploaded_mesh_) {
      DrawMesh(*mesh_, material_, MatrixIdentity());
    }
  }

  void visualizeOverlay2D() override {}

private:
  MessageBuffer<autoware_auto_planning_msgs::msg::Trajectory::SharedPtr> buffer_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr subscription_;
  std::chrono::system_clock::time_point uploaded_mesh_timestamp_;
  std::unique_ptr<Mesh> mesh_;
  Material material_;
  bool uploaded_mesh_ = false;
  const float width = 2.0;

  void calculatePointOffset(
    const geometry_msgs::msg::Pose & pose, float width_offset, Vector3 & point)
  {
    const auto & position = pose.position;
    const auto & orientation = pose.orientation;
    Eigen::Quaternionf quat(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Vector3f local_offset, rotated_offset;
    local_offset << 0, width_offset, 0;
    rotated_offset = quat * local_offset;
    point = convertFromROS<Vector3>(
      static_cast<float>(position.x) + rotated_offset.x(),
      static_cast<float>(position.y) + rotated_offset.y(),
      static_cast<float>(position.z) + rotated_offset.z());
  }

  void generateTrajectoryMesh(
    const std::unique_ptr<Mesh> & mesh,
    const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr data)
  {
    if (data->points.size() < 2) {
      std::cerr << "TrajectoryPlugin::generateTrajectoryMesh() data->points.size() < 2"
                << std::endl;
      return;
    }

    std::vector<Vector3> vertices, normals;
    Vector3 prev_right_point, prev_left_point;

    // Calculate offsets for the first point
    calculatePointOffset(data->points.at(0).pose, -(width / 2.0), prev_right_point);
    calculatePointOffset(data->points.at(0).pose, (width / 2.0), prev_left_point);

    for (size_t i = 1; i < data->points.size(); ++i) {
      Vector3 traj_right_point, traj_left_point;
      calculatePointOffset(data->points.at(i).pose, -(width / 2.0), traj_right_point);
      calculatePointOffset(data->points.at(i).pose, (width / 2.0), traj_left_point);

      // Add vertices and normals
      vertices.insert(vertices.end(), {prev_left_point, prev_right_point, traj_right_point});
      normals.insert(normals.end(), 3, Vector3{0.0f, 1.0f, 0.0f});
      vertices.insert(vertices.end(), {traj_right_point, traj_left_point, prev_left_point});
      normals.insert(normals.end(), 3, Vector3{0.0f, 1.0f, 0.0f});

      prev_left_point = traj_left_point;
      prev_right_point = traj_right_point;
    }

    // Populate mesh data
    populateMeshData(mesh, vertices, normals);
  }

  void populateMeshData(
    const std::unique_ptr<Mesh> & mesh, const std::vector<Vector3> & vertices,
    const std::vector<Vector3> & normals)
  {
    const auto vertex_count = vertices.size();
    const auto normal_count = normals.size();
    mesh->vertexCount = vertex_count;
    mesh->triangleCount = vertex_count / 3;
    mesh->vertices = (float *)RL_MALLOC(vertex_count * 3 * sizeof(float));
    mesh->normals = (float *)RL_MALLOC(normal_count * 3 * sizeof(float));

    for (size_t i = 0; i < vertex_count; ++i) {
      mesh->vertices[i * 3] = vertices[i].x;
      mesh->vertices[i * 3 + 1] = vertices[i].y;
      mesh->vertices[i * 3 + 2] = vertices[i].z;
    }
    for (size_t i = 0; i < normal_count; ++i) {
      mesh->normals[i * 3] = normals[i].x;
      mesh->normals[i * 3 + 1] = normals[i].y;
      mesh->normals[i * 3 + 2] = normals[i].z;
    }
  }
};
