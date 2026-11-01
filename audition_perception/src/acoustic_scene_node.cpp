#include "audition_perception/acoustic_scene_node.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace audition_perception {
AcousticSceneNode::AcousticSceneNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("acoustic_scene", options)
{
  const auto lost_ms = declare_parameter<int>("source_lost_after_ms", 300);
  const auto end_ms = declare_parameter<int>("source_end_after_ms", 1500);
  const auto capacity = declare_parameter<int>("max_sources", 16);
  if (capacity <= 0) throw std::invalid_argument("max_sources must be positive");
  lifecycle_ = std::make_unique<SourceLifecycle>(
    std::chrono::milliseconds(lost_ms), std::chrono::milliseconds(end_ms),
    static_cast<std::size_t>(capacity));

  source_sub_ = create_subscription<audition_msgs::msg::AcousticSourceArray>(
    "internal/spatial_sources", rclcpp::QoS(10).reliable(),
    [this](const audition_msgs::msg::AcousticSourceArray & msg) { onSources(msg); });
  source_pub_ = create_publisher<audition_msgs::msg::AcousticSourceArray>(
    "sources", rclcpp::QoS(10).reliable());
  scene_pub_ = create_publisher<audition_msgs::msg::AcousticScene>(
    "scene", rclcpp::QoS(10).reliable());
  timer_ = create_wall_timer(std::chrono::milliseconds(100),
    [this]() { onTimer(); });
}

void AcousticSceneNode::publish(SourceLifecycle::Update update) {
  if (!update.changed) return;
  // /scene contains only ACTIVE and LOST sources; /sources additionally
  // carries one-shot ENDED notifications. Both share the same generation.
  source_pub_->publish(std::move(update.sources));
  scene_pub_->publish(std::move(update.scene));
}

void AcousticSceneNode::onSources(
  const audition_msgs::msg::AcousticSourceArray & msg)
{
  try {
    frame_id_ = msg.header.frame_id;
    publish(lifecycle_->observe(msg, SourceLifecycle::Clock::now()));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Invalid spatial source snapshot: %s", e.what());
  }
}

void AcousticSceneNode::onTimer() {
  if (frame_id_.empty()) return;
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = frame_id_;
  publish(lifecycle_->tick(SourceLifecycle::Clock::now(), header));
}
}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::AcousticSceneNode)
