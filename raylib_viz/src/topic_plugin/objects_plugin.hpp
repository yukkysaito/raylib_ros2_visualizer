#pragma once

#include "raylib.h"
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
    const std::string & base_frame)
  : TopicPluginInterface(node, frame_tree, base_frame)
  {
  }

  void init() override
  {
    subscription_ =
      node_->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
        "/perception/object_recognition/objects", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ObjectsPlugin::onObjects, this, std::placeholders::_1));
  }

  void onObjects(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
  {
    std::cout << "ObjectPlugin::onObjects()" << std::endl;
    buffer_.addData(msg);
  }

  void visualize3D() override
  {
    const auto message = buffer_.getDataByTimestamp(
      std::chrono::system_clock::time_point(std::chrono::seconds(0)), false);
    if (message) {
      const auto & data = message->data;
      for (const auto & object : data->objects) {
        const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
        DrawSphere(
          convertFromROS<Vector3>(pose.position.x, pose.position.y, pose.position.z), 1.0,
          Color{255, 0, 0, 255});
      }
    }
  }

private:
  MessageBuffer<autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr> buffer_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    subscription_;
  //   std::chrono::system_clock::time_point prev_timestamp_;
};
