#pragma once

#include "audition_perception/mono_router.hpp"
#include <audition_msgs/msg/audio_frame.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace audition_perception {

// Explicit no-ODAS capture -> semantic path. Never claims spatial tracking.
class MonoSourceNode final : public rclcpp::Node {
public:
  explicit MonoSourceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  void onAudio(const audition_msgs::msg::AudioFrame & frame);
  MonoRouter router_;
  rclcpp::Subscription<audition_msgs::msg::AudioFrame>::SharedPtr audio_sub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticSourceArray>::SharedPtr sources_pub_;
  rclcpp::Publisher<audition_msgs::msg::SourceAudioFrame>::SharedPtr mono_pub_;
};

}  // namespace audition_perception
