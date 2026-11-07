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

option(AUDITION_ROS2_WITH_ALSA "Build live ALSA capture backend" OFF)
if(AUDITION_ROS2_WITH_ALSA)
  find_package(ALSA REQUIRED)
  add_library(audition_alsa_reader STATIC src/alsa_pcm_reader.cpp)
  target_compile_features(audition_alsa_reader PUBLIC cxx_std_17)
  target_include_directories(audition_alsa_reader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ALSA_INCLUDE_DIRS})
  target_link_libraries(audition_alsa_reader ${ALSA_LIBRARIES})
  add_executable(alsa_capture src/alsa_capture_main.cpp)
  target_compile_features(alsa_capture PUBLIC cxx_std_17)
  target_link_libraries(alsa_capture audition_alsa_reader)
  ament_target_dependencies(alsa_capture rclcpp audition_msgs)
  install(TARGETS alsa_capture RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS audition_capture_component wav_capture
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(capture_continuity_test test/test_capture_continuity.cpp)
  target_compile_features(capture_continuity_test PUBLIC cxx_std_17)
  target_include_directories(capture_continuity_test PRIVATE include)
  ament_add_gtest(wav_reader_test test/test_wav_reader.cpp)
  target_compile_features(wav_reader_test PUBLIC cxx_std_17)
  target_include_directories(wav_reader_test PRIVATE include)
  target_link_libraries(wav_reader_test audition_capture_component)
  if(AUDITION_ROS2_WITH_ALSA)
    ament_add_gtest(alsa_pcm_reader_test test/test_alsa_pcm_reader.cpp)
    target_compile_features(alsa_pcm_reader_test PUBLIC cxx_std_17)
    target_include_directories(alsa_pcm_reader_test PRIVATE include)
    target_link_libraries(alsa_pcm_reader_test audition_alsa_reader)
  endif()
endif()
ament_package()
