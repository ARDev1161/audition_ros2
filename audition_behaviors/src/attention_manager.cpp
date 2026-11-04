#include "audition_behaviors/attention_policy.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace audition_behaviors {
class AttentionManager final : public rclcpp::Node {
public:
  AttentionManager() : Node("attention_manager"),
    policy_(configure()) {
    publisher_ = create_publisher<audition_msgs::msg::AttentionTarget>(
      "attention/target", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<audition_msgs::msg::AcousticScene>(
      "scene", rclcpp::QoS(10).reliable(),
      [this](const audition_msgs::msg::AcousticScene & scene) {
        auto target = policy_.observe(scene, AttentionPolicy::Clock::now());
        publisher_->publish(target);
        invalid_published_ = !target.valid;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      auto target = policy_.expire(AttentionPolicy::Clock::now());
      if (!target.valid && !invalid_published_) {
        publisher_->publish(target);
        invalid_published_ = true;
      } else if (target.valid) {
        invalid_published_ = false;
      }
    });
  }
private:
  AttentionPolicy::Config configure() {
    AttentionPolicy::Config config;
    config.minimum_activity = declare_parameter<double>("minimum_activity", 0.3);
    config.switch_margin = declare_parameter<double>("switch_margin", 0.15);
    config.scene_timeout = std::chrono::milliseconds(
      declare_parameter<int>("scene_timeout_ms", 1500));
    const int confirmations = declare_parameter<int>("switch_confirmations", 1);
    if (confirmations <= 0 || confirmations > 100) {
      throw std::invalid_argument("switch_confirmations must be between 1 and 100");
    }
    config.switch_confirmations = static_cast<std::uint32_t>(confirmations);
    config.switch_confirmation_window = std::chrono::milliseconds(
      declare_parameter<int>("switch_confirmation_window_ms", 1000));
    return config;
  }
  AttentionPolicy policy_;
  bool invalid_published_ = false;
  rclcpp::Subscription<audition_msgs::msg::AcousticScene>::SharedPtr subscription_;
  rclcpp::Publisher<audition_msgs::msg::AttentionTarget>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace audition_behaviors

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<audition_behaviors::AttentionManager>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("attention_manager"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
