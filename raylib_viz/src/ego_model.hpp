#pragma once
#include "frame_tree.hpp"
#include "raylib.h"

#include <memory>
#include <string>
#include <vector>

class EgoModel
{
private:
  std::shared_ptr<FrameTree> frame_tree_;
  std::string viewer_frame_;
  std::string base_frame_;
  Model model_;
  Texture2D texture_;
  Material material_;
  Mesh mesh_;

public:
  EgoModel(
    const std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
    const std::string & base_frame);
  ~EgoModel()
  {
    UnloadTexture(texture_);  // Unload texture
    UnloadModel(model_);      // Unload model
    UnloadMaterial(material_);
    UnloadMesh(mesh_);
  }

  void drawModel();
};
