#include "audition_perception/mono_source_node.hpp"

#include <exception>
#include <utility>
#include <rclcpp_components/register_node_macro.hpp>

namespace audition_perception {

MonoSourceNode::MonoSourceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mono_source", options) {
  sources_pub_ = create_publisher<audition_msgs::msg::AcousticSourceArray>(
    "internal/spatial_sources", rclcpp::QoS(10).reliable());
  mono_pub_ = create_publisher<audition_msgs::msg::SourceAudioFrame>(
    "internal/source_audio", rclcpp::SensorDataQoS());
  audio_sub_ = create_subscription<audition_msgs::msg::AudioFrame>(
    "audio/raw", rclcpp::SensorDataQoS(),
    [this](const audition_msgs::msg::AudioFrame & frame) { onAudio(frame); });
  RCLCPP_INFO(get_logger(), "Mono semantic route: 16 kHz, one capture channel, no DOA");
}

void MonoSourceNode::onAudio(const audition_msgs::msg::AudioFrame & frame) {
  try {
    auto result = router_.process(frame);
    // Publish the canonical-source admission snapshot before its audio. DDS
    // cross-topic order is not guaranteed, so subsequent frames also refresh it.
    sources_pub_->publish(std::move(result.sources));
    mono_pub_->publish(std::move(result.audio));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Rejected mono capture frame: %s", e.what());
  }
}

}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::MonoSourceNode)
