#pragma once
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <audition_msgs/msg/semantic_progress.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/sound_classification.hpp>
#include <audition_msgs/msg/acoustic_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <audition_msgs/msg/transcript.hpp>
#include <audition_msgs/srv/get_capabilities.hpp>
#include <audition_msgs/srv/get_semantic_status.hpp>
#include <audition_msgs/srv/reload_semantic_workers.hpp>
#include "audition_perception/semantic_epoch_gate.hpp"
#include "audition_perception/speech_bridge.hpp"
#include "audition_perception/inference_completion_stats.hpp"
#include "audition_perception/inference_workload_stats.hpp"
#include <memory>
#include <string>

namespace audition_perception {
class SemanticPerceptionNode final : public rclcpp::Node {
public:
  explicit SemanticPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SemanticPerceptionNode() override;
private:
  class Impl;
  void onSourceAudio(const audition_msgs::msg::SourceAudioFrame & msg);
  void onSources(const audition_msgs::msg::AcousticSourceArray & msg);
  void pollCompleted();
  SemanticEpochGate gate_;
  InferenceCompletionStats inference_stats_;
  std::unique_ptr<Impl> impl_;
  std::unique_ptr<SpeechBridge> speech_;
  rclcpp::Subscription<audition_msgs::msg::SourceAudioFrame>::SharedPtr sub_;
  rclcpp::Subscription<audition_msgs::msg::AcousticSourceArray>::SharedPtr sources_sub_;
  rclcpp::Publisher<audition_msgs::msg::SoundClassification>::SharedPtr class_pub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticEvent>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr workload_timer_;
  rclcpp::Publisher<audition_msgs::msg::Transcript>::SharedPtr transcript_pub_;
  rclcpp::Publisher<audition_msgs::msg::SemanticProgress>::SharedPtr progress_pub_;
  rclcpp::Service<audition_msgs::srv::GetCapabilities>::SharedPtr capabilities_srv_;
  rclcpp::Service<audition_msgs::srv::GetSemanticStatus>::SharedPtr semantic_status_srv_;
  rclcpp::Service<audition_msgs::srv::ReloadSemanticWorkers>::SharedPtr reload_srv_;
  std::uint64_t reload_epoch_{0};
  std::uint64_t reload_successes_{0};
  std::uint64_t reload_failures_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  std::string profile_name_;
  std::string profile_spatial_backend_;
};
}  // namespace audition_perception
