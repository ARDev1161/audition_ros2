#pragma once
#include <audition_msgs/msg/audio_frame.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>

namespace audition_perception {
class SpatialPerceptionNode final : public rclcpp::Node {
public:
  explicit SpatialPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  void onAudio(const audition_msgs::msg::AudioFrame & msg);
  rclcpp::Subscription<audition_msgs::msg::AudioFrame>::SharedPtr audio_sub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticSourceArray>::SharedPtr sources_pub_;
  std::uint64_t generation_{0};
};
}  // namespace audition_perception
