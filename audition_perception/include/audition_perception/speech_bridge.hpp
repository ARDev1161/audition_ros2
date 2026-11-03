#pragma once
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/transcript.hpp>
#include <rclcpp/rclcpp.hpp>
#include "audition_perception/inference_workload_stats.hpp"
#include <memory>
#include <cstdint>
#include <vector>

namespace audition_perception {
struct SpeechDrainResult {
  std::vector<audition_msgs::msg::Transcript> transcripts;
  std::uint64_t completed_segments{0};
  std::uint64_t empty_segments{0};
  std::uint64_t discarded_segments{0};
};
class SpeechBridge final {
public:
  explicit SpeechBridge(rclcpp::Node & node);
  ~SpeechBridge();
  bool enabled() const noexcept;
  void onAudio(const audition_msgs::msg::SourceAudioFrame & frame);
  void onSources(const audition_msgs::msg::AcousticSourceArray & snapshot);
  void resetStream();
  SpeechDrainResult drain();
  SpeechWorkloadSnapshot workload() const;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace audition_perception
