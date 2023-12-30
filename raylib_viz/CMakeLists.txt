cmake_minimum_required(VERSION 3.14)
project(raylib_viz)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(raylib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(
  include
  SYSTEM
  ${raylib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
)

ament_auto_add_library(raylib_viz SHARED
  src/node.cpp
  src/window.cpp
  src/viewer3d.cpp
  src/camera_player.cpp
  src/frame_tree.cpp
)

target_link_libraries(raylib_viz
  ${raylib_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

rclcpp_components_register_node(raylib_viz
  PLUGIN "RaylibViz"
  EXECUTABLE raylib_viz_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
