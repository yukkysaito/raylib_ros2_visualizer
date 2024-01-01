#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "topic_plugin_interface.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <GL/gl.h>
#include <raymath.h>

#include <iostream>

#define RESET_EVERY_FRAME 1

class PointCloudPlugin : public TopicPluginInterface
{
public:
  PointCloudPlugin(
    rclcpp::Node * node, const std::shared_ptr<FrameTree> frame_tree,
    const std::string & base_frame)
  : TopicPluginInterface(node, frame_tree, base_frame)
  {
    point_mesh_ = std::make_unique<Mesh>();
    initPointCloudMesh(point_mesh_);
    point_material_ = LoadMaterialDefault();
    point_material_.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
  }
  ~PointCloudPlugin()
  {
    UnloadMaterial(point_material_);
    UnloadMesh(*point_mesh_);
  }

  void init() override
  {
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/perception/obstacle_segmentation/pointcloud", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PointCloudPlugin::onPointCloud, this, std::placeholders::_1));
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::cout << "PointCloudPlugin::onPointCloud()" << std::endl;
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

#if RESET_EVERY_FRAME
      if (uploaded_mesh_) UnloadMesh(*point_mesh_);
      initPointCloudMesh(point_mesh_);
      generatePointCloudMesh(point_mesh_, message->data);
      UploadMesh(point_mesh_.get(), true);
#else
      const int points_count = message->data->width * message->data->height;
      if (!uploaded_mesh_) {
        generatePointCloudMesh(point_mesh_, message->data);
        UploadMesh(point_mesh_.get(), true);
      } else if (points_count_max_ < points_count) {
        std::cout << "GPU memory is re-allocated. Pointcloud max size change from"
                  << points_count_max_ << "to" << points_count << std::endl;
        UnloadMesh(*point_mesh_);
        points_count_max_ = points_count;
        initPointCloudMesh(point_mesh_);
        UploadMesh(point_mesh_.get(), true);
      } else {
        updatePointCloudMesh(point_mesh_, message->data);
      }
#endif
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      std::cout << "PointCloud processing time: " << duration.count() << " ms" << std::endl;

      uploaded_mesh_ = true;
      uploaded_mesh_timestamp_ = message->timestamp;
    }
  }

  void visualize3D() override
  {
    rlEnablePointMode();
    glPointSize(3.0f);
    if (uploaded_mesh_) {
      const auto transform_opt = frame_tree_->getTransform(
        "map", "base_link", uploaded_mesh_timestamp_, std::chrono::duration<double>(0.0));

      if (transform_opt) {
        Eigen::Matrix4d eigen_matrix = convertFromROS(transform_opt.value());
        Matrix matrix = convertFromEigenMatrix(eigen_matrix);

        DrawMesh(*point_mesh_, point_material_, matrix);
      }
    }
    rlDisableWireMode();
  }

  void visualizeOverlay2D() override {}

private:
  MessageBuffer<sensor_msgs::msg::PointCloud2::SharedPtr> buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::chrono::system_clock::time_point uploaded_mesh_timestamp_;
  std::unique_ptr<Mesh> point_mesh_;
  Material point_material_;
  bool uploaded_mesh_ = false;
  int points_count_max_ = 1000000;
  void initPointCloudMesh(const std::unique_ptr<Mesh> & mesh)
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
#if !RESET_EVERY_FRAME
    mesh->vertices = (float *)RL_MALLOC(points_count_max_ * 3 * sizeof(float));

    mesh->vertexCount = points_count_max_;
    for (int i = 0; i < points_count_max_ * 3; i += 3) {
      mesh->vertices[i] = 0;
      mesh->vertices[i + 1] = 0;
      mesh->vertices[i + 2] = 0;
    }
#endif
  }

  void generatePointCloudMesh(
    const std::unique_ptr<Mesh> & mesh, const sensor_msgs::msg::PointCloud2::SharedPtr data)
  {
    int points_count = data->width * data->height;
#if RESET_EVERY_FRAME
    mesh->vertices = (float *)RL_MALLOC(points_count * 3 * sizeof(float));
#endif
    mesh->vertexCount = points_count;

    // create vertices
    sensor_msgs::PointCloud2Iterator<float> iter_x(*data, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*data, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*data, "z");
    for (int i = 0; i < points_count * 3; i += 3, ++iter_x, ++iter_y, ++iter_z) {
      mesh->vertices[i] = *iter_y;
      mesh->vertices[i + 1] = *iter_z;
      mesh->vertices[i + 2] = *iter_x;
    }
  }

  void updatePointCloudMesh(
    const std::unique_ptr<Mesh> & mesh, const sensor_msgs::msg::PointCloud2::SharedPtr data)
  {
    int points_count = data->width * data->height;
    mesh->vertexCount = points_count;

    sensor_msgs::PointCloud2Iterator<float> iter_x(*data, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*data, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*data, "z");
    for (int i = 0; i < points_count * 3; i += 3, ++iter_x, ++iter_y, ++iter_z) {
      mesh->vertices[i] = *iter_y;
      mesh->vertices[i + 1] = *iter_z;
      mesh->vertices[i + 2] = *iter_x;
    }
    UpdateMeshBuffer(*mesh, 0, mesh->vertices, points_count * 3 * sizeof(float), 0);
  }
  // Eigen::Quaterniond rotateAxes(const Eigen::Quaterniond & quaternion)
  // {
  //   Eigen::Quaterniond rotated_quaternion;
  //   rotated_quaternion.x() = quaternion.y();
  //   rotated_quaternion.y() = quaternion.z();
  //   rotated_quaternion.z() = quaternion.x();
  //   rotated_quaternion.w() = quaternion.w();

  //   return rotated_quaternion;
  // }
};
