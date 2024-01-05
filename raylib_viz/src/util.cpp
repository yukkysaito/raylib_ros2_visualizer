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
  float width, float height, float length, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices)
{
  // Combine translation and rotation to form the transformation matrix
  Eigen::Matrix4f transform = (translation * quaternion).matrix();

  // Define the local vertices of the bounding box
  std::vector<Eigen::Vector4f> local_vertices = {
    {width / 2, height / 2, length / 2, 1},   {width / 2, height / 2, -length / 2, 1},
    {-width / 2, height / 2, length / 2, 1},  {-width / 2, height / 2, -length / 2, 1},
    {width / 2, -height / 2, length / 2, 1},  {width / 2, -height / 2, -length / 2, 1},
    {-width / 2, -height / 2, length / 2, 1}, {-width / 2, -height / 2, -length / 2, 1}};

  // Define vertex indices for each face of the box
  std::vector<unsigned short> local_indices = {
    0, 1, 2, 2, 1, 3,  // Top face
    4, 6, 5, 6, 7, 5,  // Bottom face
    0, 2, 4, 4, 2, 6,  // Front face
    1, 5, 3, 5, 7, 3,  // Back face
    2, 3, 6, 6, 3, 7,  // Right face
    0, 4, 1, 4, 5, 1   // Left face
  };

  // Define normal vectors for each face
  std::vector<Eigen::Vector3f> face_normals = {
    {0, 1, 0},   // Top face
    {0, -1, 0},  // Bottom face
    {0, 0, 1},   // Front face
    {0, 0, -1},  // Back face
    {-1, 0, 0},  // Right face
    {1, 0, 0}    // Left face
  };

  vertices.clear();
  normals.clear();
  // Uncomment the following line if indices need to be cleared
  // indices.clear();

  // Transform vertices and normals using the transformation matrix and add them to the arrays
  for (size_t i = 0; i < local_indices.size(); i += 3) {
    Eigen::Vector4f transformed_vertex1 = transform * local_vertices[local_indices[i]];
    Eigen::Vector4f transformed_vertex2 = transform * local_vertices[local_indices[i + 1]];
    Eigen::Vector4f transformed_vertex3 = transform * local_vertices[local_indices[i + 2]];

    vertices.push_back({transformed_vertex1[0], transformed_vertex1[1], transformed_vertex1[2]});
    vertices.push_back({transformed_vertex2[0], transformed_vertex2[1], transformed_vertex2[2]});
    vertices.push_back({transformed_vertex3[0], transformed_vertex3[1], transformed_vertex3[2]});

    // Calculate and rotate normals
    Eigen::Vector3f normal = face_normals[i / 6];
    normal = quaternion * normal;  // Rotate the normal
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});

    // Add indices
    indices.push_back(local_indices[i]);
    indices.push_back(local_indices[i + 1]);
    indices.push_back(local_indices[i + 2]);
  }
}
