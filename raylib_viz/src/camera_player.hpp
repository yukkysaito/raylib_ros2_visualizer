#pragma once
#include "frame_tree.hpp"
#include "raylib.h"

#include <memory>
#include <string>
#include <vector>

class CameraPlayer
{
private:
  struct CameraInfo
  {
    std::string viewer_frame = "base_link";
    Camera3D camera;
    int camera_mode = CAMERA_THIRD_PERSON;
    CameraInfo()
    {
      camera.position = Vector3{0.0f, 5.0f, -10.0f};  // Camera position
      camera.target = Vector3{0.0f, 0.0f, 0.0f};      // Camera looking at point
      camera.up = Vector3{0.0f, 1.0f, 0.0f};          // Camera up vector (rotation towards target)
      camera.fovy = 45.0f;                            // Camera field-of-view Y
      camera.projection = CAMERA_PERSPECTIVE;         // Camera mode type
    }
  };
  CameraInfo camera_info_;

  std::shared_ptr<FrameTree> frame_tree_;

  float mouse_movement_sensitivity_ = 0.1f;  // Mouse sensitivity for shift movement
  float mouse_rotation_sensitivity_ = 0.5f;  // Mouse sensitivity for rotation
  float wheel_sensitivity_ = 1.0f;           // Wheel sensitivity
  float move_speed_ = 5.0f;                  // Constant movement speed
  Vector3 current_rotation_ = {0.0f, 0.0f, 0.0f};
  Vector3 target_rotation_ = {0.0f, 0.0f, 0.0f};
  Vector3 current_movement_ = {0.0f, 0.0f, 0.0f};
  Vector3 target_movement_ = {0.0f, 0.0f, 0.0f};
  float current_zoom_ = 0.0f;
  float target_zoom_ = 0.0f;

public:
  explicit CameraPlayer(std::shared_ptr<FrameTree> frame_tree);
  void updateCamera();
  void drawCameraInfo();
  void setViewerFrame(const std::string & viewer_frame);
  Camera3D getCamera() const { return camera_info_.camera; }
};
