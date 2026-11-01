#pragma once

#include <audition_msgs/msg/acoustic_scene.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "audition_perception/source_lifecycle.hpp"

namespace audition_perception {
class AcousticSceneNode final : public rclcpp::Node {
public:
  explicit AcousticSceneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  void onSources(const audition_msgs::msg::AcousticSourceArray & msg);
  void onTimer();
  void publish(SourceLifecycle::Update update);

  std::unique_ptr<SourceLifecycle> lifecycle_;
  rclcpp::Subscription<audition_msgs::msg::AcousticSourceArray>::SharedPtr source_sub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticSourceArray>::SharedPtr source_pub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticScene>::SharedPtr scene_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
};
}  // namespace audition_perception
