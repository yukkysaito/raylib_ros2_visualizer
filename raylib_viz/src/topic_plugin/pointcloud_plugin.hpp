#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "topic_plugin_interface.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <GL/gl.h>

#include <iostream>

class PointCloudPlugin : public TopicPluginInterface
{
public:
  PointCloudPlugin(
    rclcpp::Node * node, const std::shared_ptr<FrameTree> frame_tree,
    const std::string & base_frame)
  : TopicPluginInterface(node, frame_tree, base_frame)
  {
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

  void preprocess() override {}

  void visualize3D() override
  {
    const auto message = buffer_.getDataByTimestamp(
      std::chrono::system_clock::time_point(std::chrono::seconds(0)), false);
    if (message) {
      // if (message->timestamp == prev_timestamp_) {
      // std::cout << "PointCloudPlugin::visualize() : same timestamp" << std::endl;
      // return;
      // }
      const auto & data = message->data;
      sensor_msgs::PointCloud2Iterator<float> iter_x(*data, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*data, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*data, "z");

      // TODO : ここで描画する
#if 1
      rlBegin(RL_LINES);
      rlColor4ub(/*r*/ 255, /*g*/ 0, /*b*/ 0, /*a*/ 255);

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        rlVertex3f((*iter_y), (*iter_z), (*iter_x));
        rlVertex3f((*iter_y), (*iter_z + 0.05f), (*iter_x));
      }
      rlEnd();
      prev_timestamp_ = message->timestamp;

#elif 0
      // glEnable(GL_POINT_SMOOTH);
      // glPointSize(10.0f);  // ポイントのサイズを設定

      rlEnablePointMode();
      rlBegin(RL_TRIANGLES);
      rlColor4ub(/*r*/ 255, /*g*/ 0, /*b*/ 0, /*a*/ 255);
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        rlVertex3f((*iter_y), (*iter_z), (*iter_x));
      }
      rlEnd();
//      rlDisableWireMode();
#endif
    }
  }

  void visualizeOverlay2D() override {}

private:
  MessageBuffer<sensor_msgs::msg::PointCloud2::SharedPtr> buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::chrono::system_clock::time_point prev_timestamp_;
  std::unique_ptr<Mesh> pointMesh_;
  std::unique_ptr<Material> material_;
};
