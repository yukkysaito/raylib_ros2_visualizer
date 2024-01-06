#pragma once
#include "frame_tree.hpp"
#include "raylib.h"

#include <memory>
#include <string>
#include <vector>

class GridInterface
{
protected:
  std::shared_ptr<FrameTree> frame_tree_;
  std::string viewer_frame_;
  std::string base_frame_;

public:
  GridInterface(
    const std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
    const std::string & base_frame)
  : frame_tree_(frame_tree), viewer_frame_(viewer_frame), base_frame_(base_frame)
  {
  }
  virtual ~GridInterface() {}
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
