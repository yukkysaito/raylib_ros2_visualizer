#pragma once

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "topic_plugin_interface.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <iostream>

class ObjectsPlugin : public TopicPluginInterface
{
public:
  ObjectsPlugin(
    rclcpp::Node * node, const std::shared_ptr<FrameTree> frame_tree,
    const std::string & base_frame, const std::string & topic_name)
  : TopicPluginInterface(node, frame_tree, base_frame, topic_name)
  {
    mesh_ = std::make_unique<Mesh>();
    initMesh(*mesh_);
    material_ = LoadMaterialDefault();
    material_.maps[MATERIAL_MAP_DIFFUSE].color = {0, 131, 241, 200};
  }

  ~ObjectsPlugin()
  {
    UnloadMaterial(material_);
    UnloadMesh(*mesh_);
  }

  void init() override
  {
    subscription_ =
      node_->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
        topic_name_, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ObjectsPlugin::onObjects, this, std::placeholders::_1));
  }

  void onObjects(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
  {
    std::cout << "ObjectPlugin::onObjects()" << std::endl;
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
      generateObjectMesh(mesh_, message->data);
      UploadMesh(mesh_.get(), true);
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      std::cout << "Object processing time: " << duration.count() << " ms" << std::endl;

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

private:
  MessageBuffer<autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr> buffer_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    subscription_;
  std::chrono::system_clock::time_point uploaded_mesh_timestamp_;
  std::unique_ptr<Mesh> mesh_;
  Material material_;
  bool uploaded_mesh_ = false;

  void generateObjectMesh(
    const std::unique_ptr<Mesh> & mesh,
    const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr data)
  {

    auto trans = frame_tree_->getTransform(
        "map", "base_link",
        std::chrono::system_clock::time_point(std::chrono::nanoseconds(
            rclcpp::Time(data->header.stamp).nanoseconds())));
    if (!trans) {
      std::cout << "aaaaaaaaaaaaa - 0.1 fail" << std::endl;
    }

    auto transform_point = [&trans](const geometry_msgs::msg::Point &p_in)
        -> geometry_msgs::msg::Point {
      auto p_out = p_in;
      Eigen::Vector4d point_in(p_in.x, p_in.y, p_in.z, 1.0f);
      Eigen::Vector4d point_trans = trans.value().inverse() * point_in;
      p_out.x = point_trans.x();
      p_out.y = point_trans.y();
      p_out.z = point_trans.z();
      return p_out;
    };

    int vertex_count = 0;
    int normal_count = 0;
    std::vector<Vector3> all_vertices, all_normals;
    std::vector<unsigned char> all_indices;
    for (const auto & object : data->objects) {
      const auto & shape = object.shape;
      const auto &pos =
          trans ? transform_point(object.kinematics.initial_pose_with_covariance
                                      .pose.position)
                : object.kinematics.initial_pose_with_covariance.pose.position;
      const auto & quat = object.kinematics.initial_pose_with_covariance.pose.orientation;
      Eigen::Translation3f translation =
        convertFromROS<Eigen::Translation3f>(pos.x, pos.y, pos.z).cast<float>();
      Eigen::Quaternionf quaternion = convertFromROS(quat.x, quat.y, quat.z, quat.w).cast<float>();

      std::vector<Vector3> vertices, normals;
      std::vector<unsigned short> indices;

      if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
        const auto dimensions =
          convertFromROS<Vector3>(shape.dimensions.x, shape.dimensions.y, shape.dimensions.z);
        generateBoundingBox3D(
          dimensions.x, dimensions.y, dimensions.z, translation, quaternion, vertices, normals,
          indices);
      } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
        const auto dimensions =
          convertFromROS<Vector3>(shape.dimensions.x, shape.dimensions.y, shape.dimensions.z);
        generateCylinder3D(
          dimensions.x / 2, dimensions.y, translation, quaternion, vertices, normals, indices);
      } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
        std::vector<Vector2> polygon_2d;
        for (const auto & point : shape.footprint.points) {
          const auto vector = convertFromROS<Vector3>(point.x, point.y, point.z);
          polygon_2d.push_back({vector.x, vector.z});
        }
        generatePolygon3D(
          polygon_2d, shape.dimensions.z, translation, quaternion, vertices, normals, indices);
      }
      vertex_count += vertices.size();
      normal_count += normals.size();
      all_vertices.insert(all_vertices.end(), vertices.begin(), vertices.end());
      all_normals.insert(all_normals.end(), normals.begin(), normals.end());
      all_indices.insert(all_indices.end(), indices.begin(), indices.end());
    }

    mesh->vertexCount = vertex_count;
    mesh->triangleCount = vertex_count / 3;
    mesh->vertices = (float *)RL_MALLOC(vertex_count * sizeof(float) * 3);
    mesh->normals = (float *)RL_MALLOC(normal_count * sizeof(float) * 3);

    for (int i = 0; i < vertex_count; ++i) {
      mesh->vertices[i * 3] = all_vertices[i].x;
      mesh->vertices[i * 3 + 1] = all_vertices[i].y;
      mesh->vertices[i * 3 + 2] = all_vertices[i].z;
    }
    for (int i = 0; i < normal_count; ++i) {
      mesh->normals[i * 3] = all_normals[i].x;
      mesh->normals[i * 3 + 1] = all_normals[i].y;
      mesh->normals[i * 3 + 2] = all_normals[i].z;
    }
  }
};
