#pragma once

#include <audition_msgs/msg/audio_frame.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace audition_perception {

// Lives in an isolated component_container when the native ODAS backend is used.
// Native ODAS processing may be expensive and contains upstream fatal exit paths.
class SpatialPerceptionNode final : public rclcpp::Node {
public:
  explicit SpatialPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SpatialPerceptionNode() override;

private:
  class Impl;
  void onAudio(const audition_msgs::msg::AudioFrame & msg);

  std::unique_ptr<Impl> impl_;
  rclcpp::Subscription<audition_msgs::msg::AudioFrame>::SharedPtr audio_sub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticSourceArray>::SharedPtr sources_pub_;
  rclcpp::Publisher<audition_msgs::msg::SourceAudioFrame>::SharedPtr separated_pub_;
  std::string backend_;
};
}  // namespace audition_perception
