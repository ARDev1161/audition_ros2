cmake_minimum_required(VERSION 3.8)
project(audition_perception VERSION 0.1.0)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(audition_msgs REQUIRED)

option(AUDITION_ROS2_WITH_ODAS "Build native libaudition ODAS spatial adapter" OFF)
if(AUDITION_ROS2_WITH_ODAS)
  find_package(libaudition 1 CONFIG REQUIRED)
  if(NOT TARGET audition::backend_odas)
    message(FATAL_ERROR
      "libaudition was installed without its ODAS backend. Rebuild upstream with "
      "-DLIBAUDITION_WITH_ODAS=ON")
  endif()
endif()

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
if(AUDITION_ROS2_WITH_ODAS)
  target_link_libraries(audition_perception_components
    audition::backend_odas audition::core)
  target_compile_definitions(audition_perception_components PRIVATE
    AUDITION_ROS2_WITH_ODAS=1)
endif()
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
