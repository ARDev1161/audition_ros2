#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <stdexcept>
#include <string>
#include <utility>

#include <audition_msgs/msg/acoustic_scene.hpp>
#include <audition_msgs/msg/attention_target.hpp>

namespace audition_behaviors {

class AttentionPolicy {
public:
  using Clock = std::chrono::steady_clock;
  struct Config {
    double minimum_activity = 0.3;
    double switch_margin = 0.15;
    std::chrono::milliseconds scene_timeout{1500};
  };

  explicit AttentionPolicy(Config config) : config_(config) {
    if (!std::isfinite(config_.minimum_activity) ||
        config_.minimum_activity < 0 || config_.minimum_activity > 1 ||
        !std::isfinite(config_.switch_margin) ||
        config_.switch_margin < 0 || config_.switch_margin > 1 ||
        config_.scene_timeout.count() <= 0)
      throw std::invalid_argument("invalid attention policy configuration");
  }

  audition_msgs::msg::AttentionTarget observe(
    const audition_msgs::msg::AcousticScene & scene, Clock::time_point now)
  {
    // A reset/invalid scene must never leave the previous bearing actionable.
    if (scene.header.frame_id.empty() || scene.generation == 0) {
      reset();
      return invalid(scene, "invalid_scene");
    }
    // A delayed old generation must never roll the policy back to a prior stream.
    if (have_scene_ && scene.generation < generation_) {
      return invalid(scene, "regressed_generation");
    }
    if (have_scene_ && scene.generation == generation_ &&
        scene.header.frame_id == frame_id_ &&
        (scene.header.stamp.sec < last_stamp_.sec ||
         (scene.header.stamp.sec == last_stamp_.sec &&
          scene.header.stamp.nanosec < last_stamp_.nanosec))) {
      return invalid(scene, "regressed_scene_stamp");
    }
    if (have_scene_ && (scene.generation != generation_ ||
        scene.header.frame_id != frame_id_)) {
      selected_id_.clear();
      selected_generation_ = 0;
    }
    have_scene_ = true;
    generation_ = scene.generation;
    frame_id_ = scene.header.frame_id;
    last_observation_ = now;
    last_stamp_ = scene.header.stamp;

    const audition_msgs::msg::AcousticSource * best = nullptr;
    const audition_msgs::msg::AcousticSource * incumbent = nullptr;
    std::unordered_set<std::string> seen_ids;
    std::unordered_set<std::string> ambiguous_ids;
    for (const auto & source : scene.sources) {
      if (!seen_ids.insert(source.source_id).second) ambiguous_ids.insert(source.source_id);
    }
    for (const auto & source : scene.sources) {
      if (ambiguous_ids.count(source.source_id) || !eligible(source, scene)) continue;
      if (source.source_id == selected_id_ &&
          source.source_generation == selected_generation_) incumbent = &source;
      if (!best || source.activity_score > best->activity_score ||
          (source.activity_score == best->activity_score &&
           source.source_id < best->source_id)) best = &source;
    }
    if (!best) {
      selected_id_.clear();
      return invalid(scene, "no_eligible_bearing");
    }
    const auto * chosen = incumbent &&
      best->activity_score < incumbent->activity_score + config_.switch_margin
      ? incumbent : best;
    selected_id_ = chosen->source_id;
    selected_generation_ = chosen->source_generation;
    audition_msgs::msg::AttentionTarget out;
    out.header = scene.header;
    out.valid = true;
    out.source_id = chosen->source_id;
    out.source_generation = chosen->source_generation;
    out.scene_generation = scene.generation;
    out.direction_unit = chosen->direction_unit;
    out.priority = chosen->activity_score;
    out.reason = chosen == incumbent ? "retained" : "highest_activity";
    last_target_ = out;
    return out;
  }

  audition_msgs::msg::AttentionTarget expire(Clock::time_point now) {
    if (!have_scene_) return invalid({}, "no_scene");
    if (now < last_observation_ ||
        now - last_observation_ > config_.scene_timeout) {
      auto stale = invalid({}, "scene_timeout");
      stale.header.frame_id = frame_id_;
      stale.scene_generation = generation_;
      reset();
      return stale;
    }
    return last_target_;
  }

  void reset() {
    have_scene_ = false;
    selected_id_.clear();
    selected_generation_ = 0;
    generation_ = 0;
    frame_id_.clear();
    last_target_ = {};
    last_stamp_ = {};
  }

private:
  bool eligible(const audition_msgs::msg::AcousticSource & source,
                const audition_msgs::msg::AcousticScene & scene) const {
    if (source.source_id.empty() ||
        source.source_generation != scene.generation ||
        source.header.frame_id != scene.header.frame_id ||
        source.state != audition_msgs::msg::AcousticSource::ACTIVE ||
        !source.has_direction ||
        !std::isfinite(source.activity_score) ||
        source.activity_score < config_.minimum_activity ||
        source.activity_score > 1.0f) return false;
    const auto & d = source.direction_unit;
    if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z)) return false;
    const double norm = d.x * d.x + d.y * d.y + d.z * d.z;
    return std::abs(norm - 1.0) <= 0.05;
  }

  static audition_msgs::msg::AttentionTarget invalid(
    const audition_msgs::msg::AcousticScene & scene, std::string reason) {
    audition_msgs::msg::AttentionTarget out;
    out.header = scene.header;
    out.scene_generation = scene.generation;
    out.valid = false;
    out.reason = std::move(reason);
    return out;
  }

  Config config_;
  bool have_scene_ = false;
  std::uint64_t generation_ = 0;
  std::uint64_t selected_generation_ = 0;
  std::string frame_id_;
  std::string selected_id_;
  Clock::time_point last_observation_{};
  builtin_interfaces::msg::Time last_stamp_{};
  audition_msgs::msg::AttentionTarget last_target_{};
};

}  // namespace audition_behaviors
