cmake_minimum_required(VERSION 3.8)
project(audition_msgs VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioFrame.msg"
  "msg/AcousticSource.msg"
  "msg/AcousticEvent.msg"
  "msg/AcousticScene.msg"
  "msg/LocalizedSource.msg"
  "msg/Transcript.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
