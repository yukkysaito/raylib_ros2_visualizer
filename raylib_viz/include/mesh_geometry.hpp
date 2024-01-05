#pragma once

#include "raylib.h"

#include <Eigen/Dense>

#include <memory>
#include <vector>

void initMesh(const std::unique_ptr<Mesh> & mesh);

void generateBoundingBox3D(
  float width, float height, float length, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices);

void generateCylinder3D(
  float radius, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices);

void generatePolygon3D(
  const std::vector<Vector2> & polygon_2d, float height, const Eigen::Translation3f & translation,
  const Eigen::Quaternionf & quaternion, std::vector<Vector3> & vertices,
  std::vector<Vector3> & normals, std::vector<unsigned short> & indices);
