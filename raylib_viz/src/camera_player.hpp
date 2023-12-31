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
    Camera3D camera;
    int camera_mode = CAMERA_THIRD_PERSON;
    Vector3 camera_initial_offset = {0.0f, 5.0f, -10.0f};
    Vector3 camera_initial_origin = {0.0f, 0.0f, 0.0f};
    Vector3 camera_initial_up = {0.0f, 1.0f, 0.0f};
    CameraInfo()
    {
      camera.position = camera_initial_offset;  // Camera position
      camera.target = camera_initial_origin;    // Camera looking at point
      camera.up = camera_initial_up;            // Camera up vector (rotation towards target)
      camera.fovy = 45.0f;                      // Camera field-of-view Y
      camera.projection = CAMERA_PERSPECTIVE;   // Camera mode type
    }
  };
  CameraInfo camera_info_;

  std::shared_ptr<FrameTree> frame_tree_;
  std::string viewer_frame_;
  std::string base_frame_;

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
  explicit CameraPlayer(
    std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
    const std::string & base_frame);
  void updateCamera();
  void drawCameraInfo();
  void setViewerFrame(const std::string & viewer_frame);
  std::string getViewerFrame();
  Camera3D getCamera() const { return camera_info_.camera; }
};
