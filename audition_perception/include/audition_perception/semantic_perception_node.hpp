#pragma once
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <rclcpp/rclcpp.hpp>

namespace audition_perception {
class SemanticPerceptionNode final : public rclcpp::Node {
public:
  explicit SemanticPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  void onSourceAudio(const audition_msgs::msg::SourceAudioFrame & msg);
  rclcpp::Subscription<audition_msgs::msg::SourceAudioFrame>::SharedPtr sub_;
};
}  // namespace audition_perception
