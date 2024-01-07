#include "util.hpp"

#include <iostream>
#include <optional>

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

void initMesh(Mesh & mesh)
{
  mesh.triangleCount = 0;
  mesh.texcoords = NULL;
  mesh.texcoords2 = NULL;
  mesh.normals = NULL;
  mesh.tangents = NULL;
  mesh.colors = NULL;
  mesh.indices = NULL;
  mesh.animVertices = NULL;
  mesh.animNormals = NULL;
  mesh.boneIds = NULL;
  mesh.boneWeights = NULL;
  mesh.vaoId = 0;
  mesh.vboId = NULL;
  mesh.vertexCount = 0;
  mesh.vertices = NULL;
}

void generateBoundingBox3D(
  float width, float height, float length, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices)
{
#if 0
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
  indices.clear();

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

#else
  std::vector<Vector2> polygon_2d = {
    {width / 2, length / 2},
    {width / 2, -length / 2},
    {-width / 2, -length / 2},
    {-width / 2, length / 2}};
  generatePolygon3D(polygon_2d, height, translation, quaternion, vertices, normals, indices);
#endif
}

Eigen::Vector3f calcCrossProduct(const Eigen::Vector3f & a, const Eigen::Vector3f & b)
{
  return Eigen::Vector3f(
    a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(), a.x() * b.y() - a.y() * b.x());
}

std::vector<Vector2> inverseClockWise(const std::vector<Vector2> & polygon_2d)
{
  // Create a copy of the original polygon
  std::vector<Vector2> inversed_polygon = polygon_2d;

  // Reverse the order of vertices
  std::reverse(inversed_polygon.begin(), inversed_polygon.end());

  return inversed_polygon;
}

bool isClockWise(const std::vector<Vector2> & polygon_2d)
{
  const int n = polygon_2d.size();
  const double x_offset = polygon_2d.at(0).x;
  const double y_offset = polygon_2d.at(0).y;
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon_2d.size(); ++i) {
    sum += (polygon_2d.at(i).x - x_offset) * (polygon_2d.at((i + 1) % n).y - y_offset) -
           (polygon_2d.at(i).y - y_offset) * (polygon_2d.at((i + 1) % n).x - x_offset);
  }

  return sum < 0.0;
}

void generateCylinder3D(
  float radius, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices)
{
  constexpr int n = 12;
  std::vector<Vector2> polygon_2d;
  for (int i = 0; i < n; ++i) {
    Vector2 point;

    point.x = std::cos(
                (static_cast<float>(n - i) / static_cast<float>(n)) * 2.0 * M_PI +
                M_PI / static_cast<float>(n)) *
              radius;
    point.y = std::sin(
                (static_cast<float>(n - i) / static_cast<float>(n)) * 2.0 * M_PI +
                M_PI / static_cast<float>(n)) *
              radius;
    polygon_2d.push_back(point);
  }
  generatePolygon3D(polygon_2d, height, translation, quaternion, vertices, normals, indices);
}

void generatePolygon3D(
  const std::vector<Vector2> & polygon_2d, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices)
{
  // Combine translation and rotation to form the transformation matrix
  Eigen::Matrix4f transform = (translation * quaternion).matrix();

  vertices.clear();
  normals.clear();
  indices.clear();

  // check clockwise
  if (!isClockWise(polygon_2d)) {
    std::cerr << "Polygon is not clockwise" << std::endl;
    return;
  }

  // Top
  std::optional<Eigen::Vector3f> top_normal = std::nullopt;
  for (size_t i = 2; i < polygon_2d.size(); ++i) {
    std::vector<Eigen::Vector4f> local_vertices{
      {polygon_2d.at(0).x, height / 2, polygon_2d.at(0).y, 1},
      {polygon_2d.at(i - 1).x, height / 2, polygon_2d.at(i - 1).y, 1},
      {polygon_2d.at(i).x, height / 2, polygon_2d.at(i).y, 1}};

    Eigen::Vector4f transformed_vertex1, transformed_vertex2, transformed_vertex3;
    transformed_vertex1 = transform * local_vertices[0];
    transformed_vertex2 = transform * local_vertices[1];
    transformed_vertex3 = transform * local_vertices[2];

    vertices.push_back({transformed_vertex1[0], transformed_vertex1[1], transformed_vertex1[2]});
    vertices.push_back({transformed_vertex2[0], transformed_vertex2[1], transformed_vertex2[2]});
    vertices.push_back({transformed_vertex3[0], transformed_vertex3[1], transformed_vertex3[2]});

    if (!top_normal.has_value()) {
      top_normal = calcCrossProduct(
        transformed_vertex2.head<3>() - transformed_vertex1.head<3>(),
        transformed_vertex3.head<3>() - transformed_vertex1.head<3>());
    }
    const auto & normal = top_normal.value();
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
  }

  // Side
  for (size_t i = 0; i < polygon_2d.size(); ++i) {
    size_t j = (i + 1) % polygon_2d.size();

    std::vector<Eigen::Vector4f> local_vertices{
      {polygon_2d.at(i).x, height / 2, polygon_2d.at(i).y, 1},
      {polygon_2d.at(i).x, -height / 2, polygon_2d.at(i).y, 1},
      {polygon_2d.at(j).x, -height / 2, polygon_2d.at(j).y, 1},
      {polygon_2d.at(j).x, height / 2, polygon_2d.at(j).y, 1}};

    Eigen::Vector4f transformed_vertex1, transformed_vertex2, transformed_vertex3,
      transformed_vertex4;
    transformed_vertex1 = transform * local_vertices[0];
    transformed_vertex2 = transform * local_vertices[1];
    transformed_vertex3 = transform * local_vertices[2];
    transformed_vertex4 = transform * local_vertices[3];

    vertices.push_back({transformed_vertex1[0], transformed_vertex1[1], transformed_vertex1[2]});
    vertices.push_back({transformed_vertex2[0], transformed_vertex2[1], transformed_vertex2[2]});
    vertices.push_back({transformed_vertex3[0], transformed_vertex3[1], transformed_vertex3[2]});
    vertices.push_back({transformed_vertex1[0], transformed_vertex1[1], transformed_vertex1[2]});
    vertices.push_back({transformed_vertex3[0], transformed_vertex3[1], transformed_vertex3[2]});
    vertices.push_back({transformed_vertex4[0], transformed_vertex4[1], transformed_vertex4[2]});

    const Eigen::Vector3f normal = calcCrossProduct(
      transformed_vertex2.head<3>() - transformed_vertex1.head<3>(),
      transformed_vertex3.head<3>() - transformed_vertex1.head<3>());
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
  }

  // Bottom
  Eigen::Vector3f bottom_normal = -top_normal.value();
  for (size_t i = 2; i < polygon_2d.size(); ++i) {
    std::vector<Eigen::Vector4f> local_vertices{
      {polygon_2d.at(0).x, -height / 2, polygon_2d.at(0).y, 1},
      {polygon_2d.at(i).x, -height / 2, polygon_2d.at(i).y, 1},
      {polygon_2d.at(i - 1).x, -height / 2, polygon_2d.at(i - 1).y, 1}};

    Eigen::Vector4f transformed_vertex1, transformed_vertex2, transformed_vertex3;
    transformed_vertex1 = transform * local_vertices[0];
    transformed_vertex2 = transform * local_vertices[1];
    transformed_vertex3 = transform * local_vertices[2];

    vertices.push_back({transformed_vertex1[0], transformed_vertex1[1], transformed_vertex1[2]});
    vertices.push_back({transformed_vertex2[0], transformed_vertex2[1], transformed_vertex2[2]});
    vertices.push_back({transformed_vertex3[0], transformed_vertex3[1], transformed_vertex3[2]});

    const auto & normal = bottom_normal;
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
    normals.push_back({normal[0], normal[1], normal[2]});
  }
}
