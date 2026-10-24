#include "audition_capture/wav_capture_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<audition_capture::WavCaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
