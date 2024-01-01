#include "frame_tree.hpp"

FrameTree::FrameTree(rclcpp::Node * node)
: node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
{
  // Node-specific configuration can be done here if necessary
}

std::optional<Eigen::Matrix4d> FrameTree::getTransform(
  const std::string & from_frame, const std::string & to_frame,
  const std::chrono::time_point<std::chrono::system_clock> & time_point)
{
  return getTransform(from_frame, to_frame, time_point, std::chrono::duration<double>(0.0));
}

std::optional<Eigen::Matrix4d> FrameTree::getTransform(
  const std::string & from_frame, const std::string & to_frame,
  const std::chrono::time_point<std::chrono::system_clock> & time_point,
  const std::chrono::duration<double> & timeout)
{
  try {
    auto tf_time =
      tf2::timeFromSec(std::chrono::duration<double>(time_point.time_since_epoch()).count());
    auto timeout_duration = tf2::durationFromSec(timeout.count());
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped = tf_buffer_.lookupTransform(from_frame, to_frame, tf_time, timeout_duration);

    return tf2::transformToEigen(transformStamped.transform).matrix();
  } catch (const tf2::ExtrapolationException &) {
    geometry_msgs::msg::TransformStamped transformStamped =
      tf_buffer_.lookupTransform(from_frame, to_frame, tf2::TimePointZero);
    return tf2::transformToEigen(transformStamped.transform).matrix();

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("FrameTree"), ex.what());
    return std::nullopt;
  }
}
