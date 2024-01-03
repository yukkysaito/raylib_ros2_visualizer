#pragma once

#include <Eigen/Dense>

Eigen::Vector3d transformVector(
  const Eigen::Matrix4d & transformMatrix, double x, double y, double z);

Eigen::Vector3d transformVector(
  const Eigen::Matrix4d & transformMatrix, const Eigen::Vector3d & vec);
