#include "audition_perception/acoustic_scene_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace audition_perception {
AcousticSceneNode::AcousticSceneNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("acoustic_scene", options)
{
  source_sub_ = create_subscription<audition_msgs::msg::AcousticSourceArray>(
    "internal/spatial_sources", rclcpp::QoS(10).reliable(),
    [this](const audition_msgs::msg::AcousticSourceArray & msg) { onSources(msg); });
  source_pub_ = create_publisher<audition_msgs::msg::AcousticSourceArray>(
    "sources", rclcpp::QoS(10).reliable());
  scene_pub_ = create_publisher<audition_msgs::msg::AcousticScene>(
    "scene", rclcpp::QoS(10).reliable());
}
void AcousticSceneNode::onSources(const audition_msgs::msg::AcousticSourceArray & msg) {
  // M1 scaffold: pass through spatial observations. Semantic reconciliation,
  // per-source TTL and event retirement are not implemented in this version.
  audition_msgs::msg::AcousticScene scene;
  scene.header = msg.header;
  scene.generation = msg.generation;
  scene.sources = msg.sources;
  source_pub_->publish(msg);
  scene_pub_->publish(scene);
}
}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::AcousticSceneNode)
