#pragma once
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/sound_classification.hpp>
#include <audition_msgs/msg/acoustic_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

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
  std::unique_ptr<Impl> impl_;
  rclcpp::Subscription<audition_msgs::msg::SourceAudioFrame>::SharedPtr sub_;
  rclcpp::Subscription<audition_msgs::msg::AcousticSourceArray>::SharedPtr sources_sub_;
  rclcpp::Publisher<audition_msgs::msg::SoundClassification>::SharedPtr class_pub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticEvent>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace audition_perception
