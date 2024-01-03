#pragma once

#include "raylib.h"

#include <Eigen/Dense>

#include <algorithm>
#include <optional>

/**
 * @brief Convert coordinates from ROS format to another format.
 *
 * @tparam T The target type for the converted coordinates.
 * @param x The x coordinate in ROS format.
 * @param y The y coordinate in ROS format.
 * @param z The z coordinate in ROS format.
 * @return T Converted coordinates in the target format.
 */
template <typename T>
static inline T convertFromROS(const float & x, const float & y, const float & z)
{
  return T{y, z, x};
}

template <typename T>
static inline T convertFromROS(const Vector3 & vec)
{
  return convertFromROS<T>(vec.x, vec.y, vec.z);
}

template <typename T>
static inline T convertFromROS(const Eigen::Vector3d & vec)
{
  return convertFromROS<T>(vec.x(), vec.y(), vec.z());
}

template <typename T>
static inline T convertFromROS(const Eigen::Vector3f & vec)
{
  Eigen::Vector3f casted_vec = vec.cast<float>();
  return convertFromROS<T>(casted_vec.x(), casted_vec.y(), casted_vec.z());
}

Eigen::Quaternionf convertFromROS(
  const float & x, const float & y, const float & z, const float & w);

Eigen::Quaterniond convertFromROS(
  const double & x, const double & y, const double & z, const double & w);

template <typename T>
static inline T convertToROS(const float & x, const float & y, const float & z)
{
  return T{z, x, y};
}

template <typename T>
static inline T convertToROS(const Vector3 & vec)
{
  return convertToROS<T>(vec.x, vec.y, vec.z);
}

template <typename T>
static inline T convertToROS(const Eigen::Vector3d & vec)
{
  Eigen::Vector3f casted_vec = vec.cast<float>();
  return convertToROS<T>(casted_vec.x(), casted_vec.y(), casted_vec.z());
}

template <typename T>
static inline T convertToROS(const Eigen::Vector3f & vec)
{
  return convertToROS<T>(vec.x(), vec.y(), vec.z());
}

Eigen::Matrix4f convertFromROS(
  const float & x, const float & y, const float & z, const float & quat_x, const float & quat_y,
  const float & quat_z, const float & quat_w);

Eigen::Matrix4f convertFromROS(const Eigen::Matrix4f & ros_matrix);

Eigen::Matrix4d convertFromROS(
  const double & x, const double & y, const double & z, const double & quat_x,
  const double & quat_y, const double & quat_z, const double & quat_w);

Eigen::Matrix4d convertFromROS(const Eigen::Matrix4d & ros_matrix);

Matrix convertFromEigenMatrix(const Eigen::Matrix4d & eigen_matrix);

Matrix convertFromEigenMatrix(const Eigen::Matrix4f & eigen_matrix);
