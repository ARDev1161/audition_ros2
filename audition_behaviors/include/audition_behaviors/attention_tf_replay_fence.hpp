#pragma once
#include <audition_msgs/msg/attention_target.hpp>
#include <cstdint>
#include <string>

namespace audition_behaviors {
// Accepted transformed observations only: a failed TF lookup can be retried,
// but an already published bearing must never be re-armed by replay.
class AttentionTfReplayFence {
public:
  bool admissible(const audition_msgs::msg::AttentionTarget & input,
                  std::string & reason) const {
    if (!accepted_) return true;
    if (input.scene_generation < scene_generation_) {
      reason = "tf_regressed_generation";
      return false;
    }
    if (input.scene_generation == scene_generation_) {
      if (input.header.frame_id != frame_) {
        reason = "tf_frame_changed_without_generation";
        return false;
      }
      const int64_t stamp = stamp_ns(input);
      if (stamp <= stamp_ns_) {
        reason = "tf_replayed_observation";
        return false;
      }
    }
    return true;
  }
  void accept(const audition_msgs::msg::AttentionTarget & input) {
    accepted_ = true;
    scene_generation_ = input.scene_generation;
    frame_ = input.header.frame_id;
    stamp_ns_ = stamp_ns(input);
  }
private:
  static int64_t stamp_ns(const audition_msgs::msg::AttentionTarget & input) {
    return static_cast<int64_t>(input.header.stamp.sec) * 1000000000LL +
      input.header.stamp.nanosec;
  }
  bool accepted_ = false;
  uint64_t scene_generation_ = 0;
  int64_t stamp_ns_ = 0;
  std::string frame_;
};
}  // namespace audition_behaviors
