#include "window.hpp"

#include "viewer3d.hpp"

Window::Window(rclcpp::Node * node) : node_(node)
{
  SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
  // SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
  InitWindow(window_width_, window_height_, "raylib viz");
  SetTargetFPS(fps_);
}

Window::~Window()
{
  CloseWindow();
}

void Window::run(std::function<bool()> continueLoop)
{
  Viewer3D viewer3d(node_);

  while (continueLoop()) {
    rclcpp::spin_some(node_->get_node_base_interface());

    viewer3d.visualize();
  }
}
