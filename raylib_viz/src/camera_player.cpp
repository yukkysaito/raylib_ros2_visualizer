/*******************************************************************************************
 *
 *   raylib [core] example - Initialize 3d camera mode
 *
 *   Example originally created with raylib 1.0, last time updated with raylib 1.0
 *
 *   Example licensed under an unmodified zlib/libpng license, which is an OSI-certified,
 *   BSD-like license that allows static linking with closed source software
 *
 *   Copyright (c) 2014-2023 Ramon Santamaria (@raysan5)
 *
 ********************************************************************************************/
#include "camera_player.hpp"

#include "raymath.h"
#include "util.hpp"

CameraPlayer::CameraPlayer(
  std::shared_ptr<FrameTree> frame_tree, const std::string & viewer_frame,
  const std::string & base_frame)
: frame_tree_(frame_tree), viewer_frame_(viewer_frame), base_frame_(base_frame)
{
}

void CameraPlayer::setViewerFrame(const std::string & viewer_frame)
{
  viewer_frame_ = viewer_frame;
}

std::string CameraPlayer::getViewerFrame()
{
  return viewer_frame_;
}

void CameraPlayer::updateCamera()
{
  auto & camera_mode = camera_info_.camera_mode;
  auto & camera = camera_info_.camera;
  auto & camera_initial_offset = camera_info_.camera_initial_offset;

  // Handle camera mode changes
  if (IsKeyPressed(KEY_ONE)) {
    camera_mode = CAMERA_FREE;
    camera.up = Vector3{0.0f, 1.0f, 0.0f};  // Reset roll
  } else if (IsKeyPressed(KEY_TWO)) {
    camera_mode = CAMERA_FIRST_PERSON;
    camera.up = Vector3{0.0f, 1.0f, 0.0f};  // Reset roll
  } else if (IsKeyPressed(KEY_THREE)) {
    camera_mode = CAMERA_THIRD_PERSON;
    camera.up = Vector3{0.0f, 1.0f, 0.0f};  // Reset roll
  } else if (IsKeyPressed(KEY_FOUR)) {
    camera_mode = CAMERA_ORBITAL;
    camera.up = Vector3{0.0f, 1.0f, 0.0f};  // Reset roll
  }

  // Switch camera projection
  if (IsKeyPressed(KEY_P)) {
    if (camera.projection == CAMERA_PERSPECTIVE) {
      // Create isometric view
      camera_mode = CAMERA_THIRD_PERSON;
      camera = CameraInfo().camera;
      camera.position = camera_initial_offset;
      camera.up = Vector3{0.0f, 0.0f, 1.0f};
      camera.projection = CAMERA_ORTHOGRAPHIC;
    } else if (camera.projection == CAMERA_ORTHOGRAPHIC) {
      // Reset to default view
      camera = CameraInfo().camera;
      camera_mode = CAMERA_THIRD_PERSON;
      camera.position = camera_initial_offset;
      camera.up = Vector3{0.0f, 1.0f, 0.0f};
      camera.projection = CAMERA_PERSPECTIVE;
    }
  }

  // Mouse movement handling for camera rotation and zoom
  float delta_time = GetFrameTime();  // Time per frame
  Vector2 mouse_delta = GetMouseDelta();

  // Accumulate the difference moved while right-clicking
  if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
    target_movement_.x += mouse_delta.y * mouse_movement_sensitivity_;
    target_movement_.y += -mouse_delta.x * mouse_movement_sensitivity_;
  }

  // Accumulate the difference moved while left-clicking
  if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    target_rotation_.x += mouse_delta.x * mouse_rotation_sensitivity_;
    target_rotation_.y += mouse_delta.y * mouse_rotation_sensitivity_;
  }

  // Set the target value for zoom with mouse wheel
  float mouse_wheel_move = GetMouseWheelMove();
  target_zoom_ += -mouse_wheel_move * wheel_sensitivity_;

  // Move the current value closer to the target value (using constant speed)
  Vector3 rotation_delta = {
    (target_rotation_.x - current_rotation_.x) * move_speed_ * delta_time,
    (target_rotation_.y - current_rotation_.y) * move_speed_ * delta_time, 0.0f};
  Vector3 movement_delta = {
    (target_movement_.x - current_movement_.x) * move_speed_ * delta_time,
    (target_movement_.y - current_movement_.y) * move_speed_ * delta_time, 0.0f};
  float zoom_delta = (target_zoom_ - current_zoom_) * move_speed_ * delta_time;

  // Get the transform from the map frame to the viewer frame
  auto ros_transform = frame_tree_->getTransform(
    base_frame_, viewer_frame_, std::chrono::system_clock::time_point(std::chrono::seconds(0)),
    std::chrono::duration<double>(0.0));
  if (ros_transform) {
    const auto target_vec = camera_info_.camera_initial_origin;
    const auto position_vec = camera_info_.camera_initial_offset;
    const auto ros_target_vec = convertToROS<Eigen::Vector3d>(target_vec);
    const auto ros_position_vec = convertToROS<Eigen::Vector3d>(position_vec);

    const auto transformed_ros_target_vec = transformVector(*ros_transform, ros_target_vec);
    const auto transformed_ros_position_vec = transformVector(*ros_transform, ros_position_vec);
    const auto transformed_target_vec = convertFromROS<Vector3>(transformed_ros_target_vec);
    const auto transformed_position_vec = convertFromROS<Vector3>(transformed_ros_position_vec);

    camera.position = Vector3{
      transformed_position_vec.x + current_movement_.x,
      transformed_position_vec.y + current_movement_.y, transformed_position_vec.z};
    camera.target = transformed_target_vec;

    UpdateCameraPro(&camera, current_movement_, current_rotation_, current_zoom_);
  }

  // Update the camera
  UpdateCameraPro(&camera, movement_delta, rotation_delta, zoom_delta);

  current_rotation_.x += rotation_delta.x;
  current_rotation_.y += rotation_delta.y;
  current_movement_.x += movement_delta.x;
  current_movement_.y += movement_delta.y;
  current_zoom_ += zoom_delta;
}

void CameraPlayer::drawCameraInfo()
{
  auto & camera_mode = camera_info_.camera_mode;
  auto & camera = camera_info_.camera;
  // Draw info boxes
  DrawRectangle(5, 5, 330, 100, Fade(SKYBLUE, 0.5f));
  DrawRectangleLines(5, 5, 330, 100, BLUE);

  DrawText("Camera controls:", 15, 15, 10, BLACK);
  DrawText("- Look around: arrow keys or mouse", 15, 45, 10, BLACK);
  DrawText("- Camera mode keys: 1, 2, 3, 4", 15, 60, 10, BLACK);
  DrawText("- Zoom keys: num-plus, num-minus or mouse scroll", 15, 75, 10, BLACK);
  DrawText("- Camera projection key: P", 15, 90, 10, BLACK);

  DrawRectangle(600, 5, 195, 100, Fade(SKYBLUE, 0.5f));
  DrawRectangleLines(600, 5, 195, 100, BLUE);

  DrawText("Camera status:", 610, 15, 10, BLACK);
  DrawText(
    TextFormat(
      "- Mode: %s", (camera_mode == CAMERA_FREE)           ? "FREE"
                    : (camera_mode == CAMERA_FIRST_PERSON) ? "FIRST_PERSON"
                    : (camera_mode == CAMERA_THIRD_PERSON) ? "THIRD_PERSON"
                    : (camera_mode == CAMERA_ORBITAL)      ? "ORBITAL"
                                                           : "CUSTOM"),
    610, 30, 10, BLACK);
  DrawText(
    TextFormat(
      "- Projection: %s", (camera.projection == CAMERA_PERSPECTIVE)    ? "PERSPECTIVE"
                          : (camera.projection == CAMERA_ORTHOGRAPHIC) ? "ORTHOGRAPHIC"
                                                                       : "CUSTOM"),
    610, 45, 10, BLACK);
  DrawText(
    TextFormat(
      "- Position: (%06.3f, %06.3f, %06.3f)", camera.position.x, camera.position.y,
      camera.position.z),
    610, 60, 10, BLACK);
  DrawText(
    TextFormat(
      "- Target: (%06.3f, %06.3f, %06.3f)", camera.target.x, camera.target.y, camera.target.z),
    610, 75, 10, BLACK);
  DrawText(
    TextFormat("- Up: (%06.3f, %06.3f, %06.3f)", camera.up.x, camera.up.y, camera.up.z), 610, 90,
    10, BLACK);
}
