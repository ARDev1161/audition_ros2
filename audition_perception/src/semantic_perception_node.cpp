#include "audition_perception/semantic_perception_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace audition_perception {
SemanticPerceptionNode::SemanticPerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("semantic_perception", options)
{
  sub_ = create_subscription<audition_msgs::msg::SourceAudioFrame>(
    "internal/source_audio", rclcpp::SensorDataQoS(),
    [this](const audition_msgs::msg::SourceAudioFrame & msg) { onSourceAudio(msg); });
  RCLCPP_WARN(get_logger(), "Semantic inference is not implemented: no classifiers/ASR are run.");
}
void SemanticPerceptionNode::onSourceAudio(
  const audition_msgs::msg::SourceAudioFrame & msg)
{
  if (msg.sample_rate_hz == 0 || msg.frame_count == 0 ||
      msg.source_id.empty() || msg.pcm.size() != msg.frame_count) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected invalid SourceAudioFrame");
  }
  // Intentionally emit no fake semantic events or transcripts.
}
}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::SemanticPerceptionNode)
