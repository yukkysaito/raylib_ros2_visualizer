#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "topic_plugin_interface.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <iostream>

class PointCloudPlugin : public TopicPluginInterface
{
public:
  PointCloudPlugin(rclcpp::Node * node) : TopicPluginInterface(node) {}

  void init() override
  {
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/top/pointcloud", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PointCloudPlugin::onPointCloud, this, std::placeholders::_1));
    offscreen_texture_ = LoadRenderTexture(800, 450);
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::cout << "PointCloudPlugin::onPointCloud()" << std::endl;
    buffer_.addData(msg);
  }

  void preprocess() override
  {
#if 0
    const auto message = buffer_.getDataByTimestamp(
      std::chrono::system_clock::time_point(std::chrono::seconds(0)), false);
    if (message) {
      if (message->timestamp == prev_timestamp_) {
        //        std::cout << "PointCloudPlugin::visualize() : same timestamp" << std::endl;
        return;
      }
      Camera3D camera_;
      camera_.position = Vector3{0.0f, 5.0f, -10.0f};  // Camera position
      camera_.target = Vector3{0.0f, 0.0f, 0.0f};      // Camera looking at point
      camera_.up = Vector3{0.0f, 1.0f, 0.0f};          // Camera up vector (rotation towards target)
      camera_.fovy = 45.0f;                            // Camera field-of-view Y
      camera_.projection = CAMERA_PERSPECTIVE;         // Camera mode type

      const auto & data = message->data;
      sensor_msgs::PointCloud2Iterator<float> iter_x(*data, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*data, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*data, "z");

      BeginTextureMode(offscreen_texture_);
      BeginMode3D(camera_);
      ClearBackground(WHITE);
      rlBegin(RL_LINES);
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        rlVertex3f((*iter_y), -(*iter_z), (*iter_x));
        rlVertex3f((*iter_y), -(*iter_z + 0.05f), (*iter_x));
      }
      rlEnd();
      EndMode3D();
      EndTextureMode();
      prev_timestamp_ = message->timestamp;
    }
#endif
  }

  void visualize3D() override
  {
#if 1
    const auto message = buffer_.getDataByTimestamp(
      std::chrono::system_clock::time_point(std::chrono::seconds(0)), false);
    if (message) {
      if (message->timestamp == prev_timestamp_) {
        //        std::cout << "PointCloudPlugin::visualize() : same timestamp" << std::endl;
        // return;
      }
      const auto & data = message->data;
      sensor_msgs::PointCloud2Iterator<float> iter_x(*data, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*data, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*data, "z");
      //   for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      //     //        DrawSphere(Vector3{(*iter_y), -(*iter_z), (*iter_x)}, 0.05, RED);
      //     DrawPoint3D(Vector3{(*iter_y), -(*iter_z), (*iter_x)}, RED);
      //   }
      rlBegin(RL_LINES);
      rlColor4ub(/*r*/ 255, /*g*/ 0, /*b*/ 0, /*a*/ 125);

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        rlVertex3f((*iter_y), -(*iter_z), (*iter_x));
        rlVertex3f((*iter_y), -(*iter_z + 0.01f), (*iter_x));
      }
      rlEnd();
      prev_timestamp_ = message->timestamp;
    }
#endif
  }

  void visualizeOverlay2D() override
  {
#if 0
    DrawTexture(
      offscreen_texture_.texture, 0, 0, WHITE);  // オフスクリーンバッファの内容を画面に表示
#endif
  }

private:
  MessageBuffer<sensor_msgs::msg::PointCloud2::SharedPtr> buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::chrono::system_clock::time_point prev_timestamp_;
  RenderTexture2D offscreen_texture_;
};
