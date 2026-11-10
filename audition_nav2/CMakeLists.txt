cmake_minimum_required(VERSION 3.8)
project(audition_nav2)
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d QUIET)
find_package(pluginlib QUIET)
find_package(rclcpp QUIET)
# Native ODAS/YAMNet/Speech CI images deliberately omit Nav2. Keep the
# ROS-independent admission/ownership contracts available without requiring
# those unrelated jobs to install the navigation stack.
if(nav2_costmap_2d_FOUND AND pluginlib_FOUND AND rclcpp_FOUND)
  add_library(acoustic_costmap_layer SHARED src/acoustic_costmap_layer.cpp)
  target_compile_features(acoustic_costmap_layer PUBLIC cxx_std_17)
  target_include_directories(acoustic_costmap_layer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(acoustic_costmap_layer nav2_costmap_2d pluginlib rclcpp)
  pluginlib_export_plugin_description_file(nav2_costmap_2d acoustic_layer_plugin.xml)
  install(TARGETS acoustic_costmap_layer DESTINATION lib)
  install(DIRECTORY include/ DESTINATION include)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_acoustic_costmap_plugin
      test/test_acoustic_costmap_plugin.cpp)
    if(TARGET test_acoustic_costmap_plugin)
      target_compile_features(test_acoustic_costmap_plugin PUBLIC cxx_std_17)
      target_include_directories(test_acoustic_costmap_plugin PRIVATE include)
      target_link_libraries(test_acoustic_costmap_plugin acoustic_costmap_layer)
      ament_target_dependencies(test_acoustic_costmap_plugin
        nav2_costmap_2d pluginlib rclcpp)
    endif()
  endif()
else()
  message(STATUS "audition_nav2: Nav2/pluginlib/rclcpp absent; optional costmap plugin not built")
endif()
# The merge contract is ROS-independent: exercise it even in CI images
# without Nav2. The standalone assert-based executable returns nonzero on
# contract violations, and is registered with CTest.
if(BUILD_TESTING)
  enable_testing()
  add_executable(test_acoustic_master_merge test/test_acoustic_master_merge.cpp)
  target_compile_features(test_acoustic_master_merge PRIVATE cxx_std_17)
  target_include_directories(test_acoustic_master_merge PRIVATE include)
  add_test(NAME acoustic_master_merge_contract COMMAND test_acoustic_master_merge)
endif()
ament_package()
