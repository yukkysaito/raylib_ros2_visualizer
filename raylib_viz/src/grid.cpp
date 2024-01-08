/**********************************************************************************************
 *
 *   rmodels - Basic functions to draw 3d shapes and load and draw 3d models
 *
 *   CONFIGURATION:
 *       #define SUPPORT_MODULE_RMODELS
 *           rmodels module is included in the build
 *
 *       #define SUPPORT_FILEFORMAT_OBJ
 *       #define SUPPORT_FILEFORMAT_MTL
 *       #define SUPPORT_FILEFORMAT_IQM
 *       #define SUPPORT_FILEFORMAT_GLTF
 *       #define SUPPORT_FILEFORMAT_VOX
 *       #define SUPPORT_FILEFORMAT_M3D
 *           Selected desired fileformats to be supported for model data loading.
 *
 *       #define SUPPORT_MESH_GENERATION
 *           Support procedural mesh generation functions, uses external par_shapes.h library
 *           NOTE: Some generated meshes DO NOT include generated texture coordinates
 *
 *
 *   LICENSE: zlib/libpng
 *
 *   Copyright (c) 2013-2023 Ramon Santamaria (@raysan5)
 *
 *   This software is provided "as-is", without any express or implied warranty. In no event
 *   will the authors be held liable for any damages arising from the use of this software.
 *
 *   Permission is granted to anyone to use this software for any purpose, including commercial
 *   applications, and to alter it and redistribute it freely, subject to the following
 *restrictions:
 *
 *     1. The origin of this software must not be misrepresented; you must not claim that you
 *     wrote the original software. If you use this software in a product, an acknowledgment
 *     in the product documentation would be appreciated but is not required.
 *
 *     2. Altered source versions must be plainly marked as such, and must not be misrepresented
 *     as being the original software.
 *
 *     3. This notice may not be removed or altered from any source distribution.
 *
 **********************************************************************************************/

#include "grid.hpp"

#include "rlgl.h"
#include "util.hpp"

CartesianGrid::CartesianGrid(
  const std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
  const std::string & base_frame)
: GridInterface(frame_tree, viewer_frame, base_frame)
{
}
void CartesianGrid::drawGrid()
{
  const auto transform_opt = frame_tree_->getTransform(
    base_frame_, viewer_frame_, std::chrono::system_clock::time_point(std::chrono::seconds(0)),
    std::chrono::duration<double>(0.0));
  if (!transform_opt) {
    std::cerr << "Failed to get transform from map to base_link" << std::endl;
    return;
  }

  const auto eigen_matrix = convertFromROS(transform_opt.value());

  const auto & slices = slices_;
  const auto & spacing = spacing_;
  int half_slices = slices / 2;

  BeginShaderMode(shader_);

  rlBegin(RL_LINES);
  for (int i = -half_slices; i <= half_slices; i++) {
    // maybe this is not necessary because the shader is already set
    if (i == 0) {
      rlColor3f(0.5f, 0.5f, 0.5f);
      rlColor3f(0.5f, 0.5f, 0.5f);
      rlColor3f(0.5f, 0.5f, 0.5f);
      rlColor3f(0.5f, 0.5f, 0.5f);
    } else {
      rlColor3f(0.75f, 0.75f, 0.75f);
      rlColor3f(0.75f, 0.75f, 0.75f);
      rlColor3f(0.75f, 0.75f, 0.75f);
      rlColor3f(0.75f, 0.75f, 0.75f);
    }
    Eigen::Vector4d p1, p2;
    p1 << (double)i * spacing, 0.0f, (double)-half_slices * spacing, 1.0f;
    p2 << (double)i * spacing, 0.0f, (double)half_slices * spacing, 1.0f;
    p1 = eigen_matrix * p1;
    p2 = eigen_matrix * p2;
    rlVertex3f((float)p1[0], (float)p1[1], (float)p1[2]);
    rlVertex3f((float)p2[0], (float)p2[1], (float)p2[2]);

    p1 << (double)-half_slices * spacing, 0.0f, (double)i * spacing, 1.0f;
    p2 << (double)half_slices * spacing, 0.0f, (double)i * spacing, 1.0f;
    p1 = eigen_matrix * p1;
    p2 = eigen_matrix * p2;
    rlVertex3f((float)p1[0], (float)p1[1], (float)p1[2]);
    rlVertex3f((float)p2[0], (float)p2[1], (float)p2[2]);
  }
  rlEnd();
  EndShaderMode();
}
