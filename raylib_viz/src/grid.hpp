#pragma once
#include "frame_tree.hpp"
#include "raylib.h"
#include "util.hpp"

#include <memory>
#include <string>
#include <vector>

class GridInterface
{
protected:
  std::shared_ptr<FrameTree> frame_tree_;
  std::string viewer_frame_;
  std::string base_frame_;
  Shader shader_;

public:
  GridInterface(
    const std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
    const std::string & base_frame)
  : frame_tree_(frame_tree), viewer_frame_(viewer_frame), base_frame_(base_frame)
  {
    shader_ = LoadShader(0, std::string(getMyPackagePath() + "/resource/shader/grid.fs").c_str());
  }
  virtual ~GridInterface() { UnloadShader(shader_); }
  virtual void drawGrid() = 0;
};

class CartesianGrid : public GridInterface
{
private:
  int slices_ = 100;
  float spacing_ = 1.0f;

public:
  CartesianGrid(
    const std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
    const std::string & base_frame);
  void drawGrid() override;
};
