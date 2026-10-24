cmake_minimum_required(VERSION 3.8)
project(audition_capture VERSION 0.1.0)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(audition_msgs REQUIRED)

add_library(audition_capture_component SHARED
  src/wav_reader.cpp
  src/wav_capture_node.cpp)
target_compile_features(audition_capture_component PUBLIC cxx_std_17)
target_include_directories(audition_capture_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(audition_capture_component rclcpp rclcpp_components audition_msgs)
rclcpp_components_register_nodes(audition_capture_component "audition_capture::WavCaptureNode")

add_executable(wav_capture src/wav_capture_main.cpp)
target_compile_features(wav_capture PUBLIC cxx_std_17)
target_link_libraries(wav_capture audition_capture_component)
ament_target_dependencies(wav_capture rclcpp)

install(TARGETS audition_capture_component wav_capture
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wav_reader_test test/test_wav_reader.cpp)
  target_compile_features(wav_reader_test PUBLIC cxx_std_17)
  target_include_directories(wav_reader_test PRIVATE include)
  target_link_libraries(wav_reader_test audition_capture_component)
endif()
ament_package()
