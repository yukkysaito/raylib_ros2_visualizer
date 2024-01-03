#include "util.hpp"

#include <iostream>

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
Eigen::Quaternionf convertFromROS(
  const float & x, const float & y, const float & z, const float & w)
{
  return Eigen::Quaternionf(w, y, z, x);
}

Eigen::Quaterniond convertFromROS(
  const double & x, const double & y, const double & z, const double & w)
{
  return Eigen::Quaterniond(w, y, z, x);
}

Eigen::Matrix4d convertFromROS(const Eigen::Matrix4d & ros_matrix)
{
  Eigen::Quaterniond ros_quaternion(ros_matrix.block<3, 3>(0, 0));

  return convertFromROS(
    ros_matrix(0, 3), ros_matrix(1, 3), ros_matrix(2, 3), ros_quaternion.x(), ros_quaternion.y(),
    ros_quaternion.z(), ros_quaternion.w());
}

Eigen::Matrix4d convertFromROS(
  const double & x, const double & y, const double & z, const double & quat_x,
  const double & quat_y, const double & quat_z, const double & quat_w)
{
  Eigen::Quaterniond quaternion = convertFromROS(quat_x, quat_y, quat_z, quat_w);

  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

  Eigen::Matrix4d affine_matrix = Eigen::Matrix4d::Zero();
  affine_matrix.block<3, 3>(0, 0) = rotation_matrix;

  affine_matrix(0, 3) = y;
  affine_matrix(1, 3) = z;
  affine_matrix(2, 3) = x;

  affine_matrix.row(3) << 0, 0, 0, 1;
  return affine_matrix;
}

Eigen::Matrix4f convertFromROS(const Eigen::Matrix4f & ros_matrix)
{
  Eigen::Quaternionf ros_quaternion(ros_matrix.block<3, 3>(0, 0));

  return convertFromROS(
    ros_matrix(0, 3), ros_matrix(1, 3), ros_matrix(2, 3), ros_quaternion.x(), ros_quaternion.y(),
    ros_quaternion.z(), ros_quaternion.w());
}

Eigen::Matrix4f convertFromROS(
  const float & x, const float & y, const float & z, const float & quat_x, const float & quat_y,
  const float & quat_z, const float & quat_w)
{
  Eigen::Quaternionf quaternion = convertFromROS(quat_x, quat_y, quat_z, quat_w);

  Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();

  Eigen::Matrix4f affine_matrix = Eigen::Matrix4f::Zero();
  affine_matrix.block<3, 3>(0, 0) = rotation_matrix;

  affine_matrix(0, 3) = y;
  affine_matrix(1, 3) = z;
  affine_matrix(2, 3) = x;

  affine_matrix.row(3) << 0, 0, 0, 1;
  return affine_matrix;
}

void initMesh(const std::unique_ptr<Mesh> & mesh)
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
}

void generateBoundingBox3D(
  float length, float width, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices)
{
  // Eigenを使用して変換行列を計算
  //   Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();
  //   Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
  //   transform.block<3, 3>(0, 0) = rotation_matrix;
  //   transform(0, 3) = translation.x();
  //   transform(1, 3) = translation.y();
  //   transform(2, 3) = translation.z();
  //   transform.row(3) << 0, 0, 0, 1;

  Eigen::Matrix4f transform = (translation * quaternion).matrix();

  // バウンディングボックスのローカル頂点
  std::vector<Eigen::Vector4f> local_vertices = {
    {length / 2, width / 2, height / 2, 1},    {length / 2, -width / 2, height / 2, 1},
    {-length / 2, -width / 2, height / 2, 1},  {-length / 2, width / 2, height / 2, 1},
    {length / 2, width / 2, -height / 2, 1},   {length / 2, -width / 2, -height / 2, 1},
    {-length / 2, -width / 2, -height / 2, 1}, {-length / 2, width / 2, -height / 2, 1}};

  // 各面の頂点インデックス
  std::vector<unsigned short> local_indices = {
    0, 1, 2, 2, 3, 0,  // 上面
    4, 5, 6, 6, 7, 4,  // 下面
    0, 4, 7, 7, 3, 0,  // 前面
    1, 5, 6, 6, 2, 1,  // 後面
    0, 4, 5, 5, 1, 0,  // 右面
    3, 7, 6, 6, 2, 3   // 左面
  };

  // 各面の法線ベクトル
  std::vector<Eigen::Vector3f> face_normals = {
    {0, 0, 1},   // 上面
    {0, 0, -1},  // 下面
    {0, 1, 0},   // 前面
    {0, -1, 0},  // 後面
    {1, 0, 0},   // 右面
    {-1, 0, 0}   // 左面
  };

  vertices.clear();
  normals.clear();
  // indices.clear();  // インデックスが必要な場合にコメントアウトを解除

  //   変換行列を使用して頂点と法線を変換し、配列に追加
  for (size_t i = 0; i < local_indices.size(); i += 3) {
    Eigen::Vector4f transformed_vertex1 = transform * local_vertices[local_indices[i]];
    Eigen::Vector4f transformed_vertex2 = transform * local_vertices[local_indices[i + 1]];
    Eigen::Vector4f transformed_vertex3 = transform * local_vertices[local_indices[i + 2]];

    vertices.push_back({transformed_vertex1[0], transformed_vertex1[1], transformed_vertex1[2]});
    vertices.push_back({transformed_vertex2[0], transformed_vertex2[1], transformed_vertex2[2]});
    vertices.push_back({transformed_vertex3[0], transformed_vertex3[1], transformed_vertex3[2]});

    // 法線の計算
    Eigen::Vector3f normal = face_normals[i / 6];
    normal = quaternion * normal;  // 法線を回転
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});

    // インデックスの追加
    indices.push_back(local_indices[i]);
    indices.push_back(local_indices[i + 1]);
    indices.push_back(local_indices[i + 2]);
  }

  //   {
  //     size_t i = 0;
  //     Eigen::Vector4f transformed_vertex1 = transform * local_vertices[local_indices[i]];
  //     Eigen::Vector4f transformed_vertex2 = transform * local_vertices[local_indices[i + 1]];
  //     Eigen::Vector4f transformed_vertex3 = transform * local_vertices[local_indices[i + 2]];
  //     std::cout << "translation: " << translation.x() << ", " << translation.y() << ", "
  //               << translation.z() << std::endl;
  //     std::cout << "quaternion: " << quaternion.x() << ", " << quaternion.y() << ", "
  //               << quaternion.z() << ", " << quaternion.w() << std::endl;
  //     std::cout << "local_indices[i]: " << local_indices[i] << std::endl;
  //     std::cout << "local_indices[i + 1]: " << local_indices[i + 1] << std::endl;
  //     std::cout << "local_indices[i + 2]: " << local_indices[i + 2] << std::endl;
  //     std::cout << "local_vertices[local_indices[i]]: " << local_vertices[local_indices[i]]
  //               << std::endl;
  //     std::cout << "local_vertices[local_indices[i + 1]]: " << local_vertices[local_indices[i +
  //     1]]
  //               << std::endl;
  //     std::cout << "local_vertices[local_indices[i + 2]]: " << local_vertices[local_indices[i +
  //     2]]
  //               << std::endl;
  //     std::cout << "transformed_vertex1: " << transformed_vertex1 << std::endl;
  //     std::cout << "transformed_vertex2: " << transformed_vertex2 << std::endl;
  //     std::cout << "transformed_vertex3: " << transformed_vertex3 << std::endl;
  //     std::cout << "transform: " << transform << std::endl;
  //     std::cout << "------------------------------------" << std::endl;
  //     vertices.push_back({transformed_vertex1[0], transformed_vertex1[1],
  //     transformed_vertex1[2]}); vertices.push_back({transformed_vertex2[0],
  //     transformed_vertex2[1], transformed_vertex2[2]});
  //     vertices.push_back({transformed_vertex3[0], transformed_vertex3[1],
  //     transformed_vertex3[2]});

  //     // 法線の計算
  //     Eigen::Vector3f normal = face_normals[i / 6];
  //     normal = quaternion * normal;  // 法線を回転
  //     normals.push_back({normal[0], normal[1], normal[2]});
  //     normals.push_back({normal[0], normal[1], normal[2]});
  //     normals.push_back({normal[0], normal[1], normal[2]});

  //     // インデックスの追加
  //     indices.push_back(local_indices[i]);
  //     indices.push_back(local_indices[i + 1]);
  //     indices.push_back(local_indices[i + 2]);
  //   }
}
