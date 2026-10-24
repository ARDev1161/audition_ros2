#pragma once

#include <audition_msgs/msg/audio_frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include "audition_capture/wav_reader.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace audition_capture {

class WavCaptureNode final : public rclcpp::Node {
public:
  explicit WavCaptureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishNext();

  std::unique_ptr<WavReader> reader_;
  rclcpp::Publisher<audition_msgs::msg::AudioFrame>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
  std::uint32_t frames_per_message_{160};
  std::uint64_t sequence_{0};
  bool loop_{false};
};

}  // namespace audition_capture
