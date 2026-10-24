#include "audition_perception/spatial_perception_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace audition_perception {
SpatialPerceptionNode::SpatialPerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("spatial_perception", options)
{
  const auto backend = declare_parameter<std::string>("backend", "disabled");
  if (backend != "disabled") {
    throw std::invalid_argument(
      "Spatial backend is not implemented yet. Set backend=disabled; ODAS integration is pending.");
  }
  audio_sub_ = create_subscription<audition_msgs::msg::AudioFrame>(
    "audio/raw", rclcpp::SensorDataQoS(),
    [this](const audition_msgs::msg::AudioFrame & msg) { onAudio(msg); });
  sources_pub_ = create_publisher<audition_msgs::msg::AcousticSourceArray>(
    "internal/spatial_sources", rclcpp::QoS(10).reliable());
  RCLCPP_WARN(get_logger(),
    "Spatial backend disabled: validates PCM and publishes EMPTY source snapshots; NO DOA or separation.");
}

void SpatialPerceptionNode::onAudio(const audition_msgs::msg::AudioFrame & msg) {
  const auto count = static_cast<std::uint64_t>(msg.frame_count) * msg.channel_count;
  if (msg.sample_rate_hz == 0 || msg.frame_count == 0 || msg.channel_count == 0 ||
      msg.channel_indices.size() != msg.channel_count ||
      msg.layout > audition_msgs::msg::AudioFrame::PLANAR ||
      count != msg.pcm.size() ||
      !std::all_of(msg.pcm.begin(), msg.pcm.end(), [](float v) {return std::isfinite(v) && v >= -1.0f && v <= 1.0f;})) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid AudioFrame");
    return;
  }
  audition_msgs::msg::AcousticSourceArray out;
  out.header = msg.header;
  out.generation = generation_;
  // No fabricated source IDs, directions or audio streams while ODAS is absent.
  sources_pub_->publish(out);
}
}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::SpatialPerceptionNode)
