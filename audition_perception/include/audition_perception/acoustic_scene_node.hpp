#pragma once
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/acoustic_scene.hpp>
#include <rclcpp/rclcpp.hpp>

namespace audition_perception {
class AcousticSceneNode final : public rclcpp::Node {
public:
  explicit AcousticSceneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  void onSources(const audition_msgs::msg::AcousticSourceArray & msg);
  rclcpp::Subscription<audition_msgs::msg::AcousticSourceArray>::SharedPtr source_sub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticSourceArray>::SharedPtr source_pub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticScene>::SharedPtr scene_pub_;
};
}  // namespace audition_perception
