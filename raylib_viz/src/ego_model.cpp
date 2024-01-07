#include "ego_model.hpp"

#include "rlgl.h"
#include "util.hpp"

EgoModel::EgoModel(
  const std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
  const std::string & base_frame)
: frame_tree_(frame_tree), viewer_frame_(viewer_frame), base_frame_(base_frame)
{
  initMesh(mesh_);
  material_ = LoadMaterialDefault();
  material_.maps[MATERIAL_MAP_DIFFUSE].color = {255, 0, 0, 255};

  const float width = 2.0f;
  const float length = 5.0f;
  const float height = 2.0f;
  const float rear_overhang = 1.0f;
  std::vector<Vector3> vertices, normals;
  std::vector<unsigned short> indices;
  Eigen::Translation3f translation(0.0f, width / 2, length / 2 - rear_overhang);
  Eigen::Quaternionf quaternion(1.0f, 0.0f, 0.0f, 0.0f);
  generateBoundingBox3D(width, height, length, translation, quaternion, vertices, normals, indices);

  const int vertex_count = vertices.size();
  const int normal_count = normals.size();
  mesh_.vertexCount = vertex_count;
  mesh_.triangleCount = vertex_count / 3;
  mesh_.vertices = (float *)RL_MALLOC(vertex_count * sizeof(float) * 3);
  mesh_.normals = (float *)RL_MALLOC(normal_count * sizeof(float) * 3);

  for (int i = 0; i < vertex_count; ++i) {
    mesh_.vertices[i * 3] = vertices[i].x;
    mesh_.vertices[i * 3 + 1] = vertices[i].y;
    mesh_.vertices[i * 3 + 2] = vertices[i].z;
  }
  for (int i = 0; i < normal_count; ++i) {
    mesh_.normals[i * 3] = normals[i].x;
    mesh_.normals[i * 3 + 1] = normals[i].y;
    mesh_.normals[i * 3 + 2] = normals[i].z;
  }
  UploadMesh(&mesh_, false);
}

void EgoModel::drawModel()
{
  const auto transform_opt = frame_tree_->getTransform(
    base_frame_, viewer_frame_, std::chrono::system_clock::time_point(std::chrono::seconds(0)),
    std::chrono::duration<double>(0.0));
  if (!transform_opt) {
    std::cerr << "Failed to get transform from map to base_link" << std::endl;
    return;
  }

  const auto eigen_matrix = convertFromROS(transform_opt.value());
  Matrix matrix = convertFromEigenMatrix(eigen_matrix);

  DrawMesh(mesh_, material_, matrix);
}
