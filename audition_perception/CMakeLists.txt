cmake_minimum_required(VERSION 3.8)
project(audition_perception VERSION 0.1.0)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(audition_msgs REQUIRED)

option(AUDITION_ROS2_WITH_SEMANTIC "Enable libaudition semantic runtime" OFF)
option(AUDITION_ROS2_WITH_YAMNET "Enable native YAMNet ONNX classifier" OFF)
if(AUDITION_ROS2_WITH_YAMNET AND NOT AUDITION_ROS2_WITH_SEMANTIC)
  message(FATAL_ERROR "YAMNet requires AUDITION_ROS2_WITH_SEMANTIC=ON")
endif()
option(AUDITION_ROS2_WITH_ODAS "Build native libaudition ODAS spatial adapter" OFF)
if(AUDITION_ROS2_WITH_ODAS OR AUDITION_ROS2_WITH_SEMANTIC)
  find_package(libaudition 1 CONFIG REQUIRED)
endif()
if(AUDITION_ROS2_WITH_ODAS)
  if(NOT TARGET audition::backend_odas)
    message(FATAL_ERROR
      "libaudition was installed without its ODAS backend. Rebuild upstream with "
      "-DLIBAUDITION_WITH_ODAS=ON")
  endif()
endif()

add_library(audition_perception_components SHARED
  src/spatial_perception_node.cpp
  src/source_lifecycle.cpp
  src/event_ledger.cpp
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
if(AUDITION_ROS2_WITH_SEMANTIC)
  if(NOT TARGET audition::pipeline)
    message(FATAL_ERROR "libaudition pipeline target is required for semantic runtime")
  endif()
  target_link_libraries(audition_perception_components audition::pipeline)
  target_compile_definitions(audition_perception_components PRIVATE
    AUDITION_ROS2_WITH_SEMANTIC=1)
endif()
if(AUDITION_ROS2_WITH_YAMNET)
  if(NOT TARGET audition::backend_yamnet)
    message(FATAL_ERROR "libaudition YAMNet backend not installed; enable LIBAUDITION_WITH_YAMNET")
  endif()
  target_link_libraries(audition_perception_components audition::backend_yamnet)
  target_compile_definitions(audition_perception_components PRIVATE
    AUDITION_ROS2_WITH_YAMNET=1)
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
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(event_ledger_test test/test_event_ledger.cpp)
  target_compile_features(event_ledger_test PUBLIC cxx_std_17)
  target_include_directories(event_ledger_test PRIVATE include)
  target_link_libraries(event_ledger_test audition_perception_components)
  ament_target_dependencies(event_ledger_test audition_msgs)
  ament_add_gtest(source_lifecycle_test test/test_source_lifecycle.cpp)
  target_compile_features(source_lifecycle_test PUBLIC cxx_std_17)
  target_include_directories(source_lifecycle_test PRIVATE include)
  target_link_libraries(source_lifecycle_test audition_perception_components)
  ament_target_dependencies(source_lifecycle_test audition_msgs)
endif()
ament_package()
