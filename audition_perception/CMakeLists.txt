cmake_minimum_required(VERSION 3.8)
project(audition_perception VERSION 0.1.0)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(audition_msgs REQUIRED)

add_library(audition_perception_components SHARED
  src/spatial_perception_node.cpp
  src/acoustic_scene_node.cpp
  src/semantic_perception_node.cpp)
target_compile_features(audition_perception_components PUBLIC cxx_std_17)
target_include_directories(audition_perception_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(audition_perception_components
  rclcpp rclcpp_components audition_msgs)
rclcpp_components_register_nodes(audition_perception_components
  "audition_perception::SpatialPerceptionNode"
  "audition_perception::AcousticSceneNode"
  "audition_perception::SemanticPerceptionNode")

install(TARGETS audition_perception_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
