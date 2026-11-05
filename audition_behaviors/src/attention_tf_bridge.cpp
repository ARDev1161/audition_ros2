#include "audition_behaviors/attention_tf_geometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace audition_behaviors {
class AttentionTfBridge final : public rclcpp::Node {
public:
  AttentionTfBridge() : Node("attention_tf_bridge"),
    target_frame_(declare_parameter<std::string>("target_frame", "base_link")),
    maximum_age_ms_(declare_parameter<int>("maximum_age_ms", 500)),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    if (target_frame_.empty() || target_frame_[0] == '/' || maximum_age_ms_ <= 0) {
      throw std::invalid_argument("target_frame must be relative and maximum_age_ms positive");
    }
    publisher_ = create_publisher<audition_msgs::msg::AttentionTarget>(
      "attention/target_base", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<audition_msgs::msg::AttentionTarget>(
      "attention/target", rclcpp::QoS(10).reliable(),
      [this](const audition_msgs::msg::AttentionTarget & input) { transform(input); });
  }
private:
  void transform(const audition_msgs::msg::AttentionTarget & input) {
    auto output = input;
    output.valid = false;
    output.direction_unit.x = 0.0;
    output.direction_unit.y = 0.0;
    output.direction_unit.z = 0.0;
    output.header.frame_id = target_frame_;
    std::string reason;
    if (!valid_attention_observation(input, now().nanoseconds(), maximum_age_ms_, reason)) {
      output.reason = reason;
      publisher_->publish(output);
      return;
    }
    const auto & v = input.direction_unit;
    try {
      // Look up the transform at the *observation timestamp*, never latest.
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_, input.header.frame_id, input.header.stamp,
        rclcpp::Duration::from_seconds(0.0));
      const auto & q = tf.transform.rotation;
      if (!rotate_attention_direction(v, q, output.direction_unit)) {
        output.direction_unit = geometry_msgs::msg::Vector3();
        output.reason = "invalid_tf_rotation";
      } else {
        output.valid = true;
        output.reason = "tf_transformed";
      }
    } catch (const tf2::TransformException &) {
      output.reason = "tf_unavailable";
    }
    publisher_->publish(output);
  }
  std::string target_frame_;
  int maximum_age_ms_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<audition_msgs::msg::AttentionTarget>::SharedPtr subscription_;
  rclcpp::Publisher<audition_msgs::msg::AttentionTarget>::SharedPtr publisher_;
};
}  // namespace audition_behaviors

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<audition_behaviors::AttentionTfBridge>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("attention_tf_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
