#pragma once

#include <audition_msgs/msg/acoustic_scene.hpp>
#include <audition_msgs/msg/acoustic_event.hpp>
#include <audition_msgs/msg/sound_classification.hpp>
#include <audition_msgs/msg/transcript.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "audition_perception/source_lifecycle.hpp"
#include "audition_perception/event_ledger.hpp"

namespace audition_perception {
class AcousticSceneNode final : public rclcpp::Node {
public:
  explicit AcousticSceneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  void onSources(const audition_msgs::msg::AcousticSourceArray & msg);
  void onTimer();
  void onEvent(const audition_msgs::msg::AcousticEvent & msg);
  void onClassification(const audition_msgs::msg::SoundClassification & msg);
  void onTranscript(const audition_msgs::msg::Transcript & msg);
  void publish(SourceLifecycle::Update update);

  std::unique_ptr<SourceLifecycle> lifecycle_;
  std::unique_ptr<EventLedger> event_ledger_;
  std::uint64_t stream_generation_{0};
  bool have_stream_{false};
  std::uint64_t canonical_sequence_{0};
  rclcpp::Subscription<audition_msgs::msg::AcousticSourceArray>::SharedPtr source_sub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticSourceArray>::SharedPtr source_pub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticScene>::SharedPtr scene_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<audition_msgs::msg::AcousticEvent>::SharedPtr event_sub_;
  rclcpp::Subscription<audition_msgs::msg::SoundClassification>::SharedPtr class_sub_;
  rclcpp::Publisher<audition_msgs::msg::AcousticEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<audition_msgs::msg::SoundClassification>::SharedPtr class_pub_;
  rclcpp::Subscription<audition_msgs::msg::Transcript>::SharedPtr transcript_sub_;
  rclcpp::Publisher<audition_msgs::msg::Transcript>::SharedPtr transcript_pub_;
  std::string frame_id_;
  std::string scene_session_id_;
};
}  // namespace audition_perception
