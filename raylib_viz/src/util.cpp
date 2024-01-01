#include "util.hpp"

Eigen::Vector3d transformVector(
  const Eigen::Matrix4d & transformMatrix, double x, double y, double z)
{
  Eigen::Vector4d vec(x, y, z, 1.0);
  Eigen::Vector4d transformed_vec = transformMatrix * vec;
  return Eigen::Vector3d(transformed_vec.x(), transformed_vec.y(), transformed_vec.z());
}

Eigen::Vector3d transformVector(
  const Eigen::Matrix4d & transformMatrix, const Eigen::Vector3d & vec)
{
  return transformVector(transformMatrix, vec.x(), vec.y(), vec.z());
}

Matrix convertFromEigenMatrix(const Eigen::Matrix4d & eigen_matrix)
{
  Eigen::Matrix4f casted_matrix = eigen_matrix.cast<float>();
  return convertFromEigenMatrix(casted_matrix);
  //   return convertFromEigenMatrix(eigen_matrix.cast<float>());
}

Matrix convertFromEigenMatrix(const Eigen::Matrix4f & eigen_matrix)
{
  Matrix matrix = {eigen_matrix(0, 0), eigen_matrix(0, 1), eigen_matrix(0, 2), eigen_matrix(0, 3),
                   eigen_matrix(1, 0), eigen_matrix(1, 1), eigen_matrix(1, 2), eigen_matrix(1, 3),
                   eigen_matrix(2, 0), eigen_matrix(2, 1), eigen_matrix(2, 2), eigen_matrix(2, 3),
                   eigen_matrix(3, 0), eigen_matrix(3, 1), eigen_matrix(3, 2), eigen_matrix(3, 3)};
  return matrix;
}

Eigen::Matrix4d convertFromROS(const Eigen::Matrix4d & ros_matrix)
{
  Eigen::Quaterniond ros_quaternion(ros_matrix.block<3, 3>(0, 0));
  Eigen::Quaterniond quaternion;
  quaternion.x() = ros_quaternion.y();
  quaternion.y() = ros_quaternion.z();
  quaternion.z() = ros_quaternion.x();
  quaternion.w() = ros_quaternion.w();

  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

  Eigen::Matrix4d affine_matrix = Eigen::Matrix4d::Zero();
  affine_matrix.block<3, 3>(0, 0) = rotation_matrix;

  affine_matrix(0, 3) = ros_matrix(1, 3);
  affine_matrix(1, 3) = ros_matrix(2, 3);
  affine_matrix(2, 3) = ros_matrix(0, 3);

  affine_matrix.row(3) << 0, 0, 0, 1;
  return affine_matrix;
}
