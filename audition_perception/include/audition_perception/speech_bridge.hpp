#pragma once
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/transcript.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

namespace audition_perception {
class SpeechBridge final {
public:
  explicit SpeechBridge(rclcpp::Node & node);
  ~SpeechBridge();
  bool enabled() const noexcept;
  void onAudio(const audition_msgs::msg::SourceAudioFrame & frame);
  void onSources(const audition_msgs::msg::AcousticSourceArray & snapshot);
  std::vector<audition_msgs::msg::Transcript> drain();
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace audition_perception
