#include "window.hpp"

#include "viewer3d.hpp"

Window::Window(rclcpp::Node * node) : node_(node)
{
  SetConfigFlags(FLAG_WINDOW_RESIZABLE);
  InitWindow(window_width_, window_height_, "raylib visualizer");
}

Window::~Window()
{
  CloseWindow();
}

void Window::run(std::function<bool()> continueLoop)
{
  Viewer3D viewer3d(node_);

  SetTargetFPS(fps_);  // Set our game to run at 60 frames-per-second
  ClearBackground(RAYWHITE);

  while (continueLoop())  // Detect window close button or ESC key
  {
    BeginDrawing();
    viewer3d.visualize();
    EndDrawing();
  }
}
