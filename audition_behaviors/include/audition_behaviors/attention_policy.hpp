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
    // Require repeated accepted snapshots before changing to a new source.
    std::uint32_t switch_confirmations = 1;
    std::chrono::milliseconds switch_confirmation_window{1000};
    // Zero disables the optional DOA uncertainty gate.
    double maximum_angular_variance_rad2 = 0.0;
    // Zero disables per-source last_seen age validation against the scene stamp.
    std::chrono::milliseconds maximum_source_age{0};
    // Zero disables angular jump rejection for the currently selected source.
    double maximum_direction_jump_rad = 0.0;
    // Reject equal scene stamps to prevent duplicate snapshots counting as evidence.
    bool require_advancing_scene_stamp = false;
  };

  explicit AttentionPolicy(Config config) : config_(config) {
    if (!std::isfinite(config_.minimum_activity) ||
        config_.minimum_activity < 0 || config_.minimum_activity > 1 ||
        !std::isfinite(config_.switch_margin) ||
        config_.switch_margin < 0 || config_.switch_margin > 1 ||
        config_.scene_timeout.count() <= 0 ||
        config_.switch_confirmations == 0 ||
        config_.switch_confirmations > 100 ||
        config_.switch_confirmation_window.count() <= 0 ||
        !std::isfinite(config_.maximum_angular_variance_rad2) ||
        config_.maximum_angular_variance_rad2 < 0 ||
        config_.maximum_source_age.count() < 0 ||
        !std::isfinite(config_.maximum_direction_jump_rad) ||
        config_.maximum_direction_jump_rad < 0.0 ||
        config_.maximum_direction_jump_rad > 3.14159265358979323846)
      throw std::invalid_argument("invalid attention policy configuration");
  }

  audition_msgs::msg::AttentionTarget observe(
    const audition_msgs::msg::AcousticScene & scene, Clock::time_point now)
  {
    // Timer callbacks are not guaranteed to run before a new scene arrives.
    // A stalled executor must not preserve an expired incumbent or its
    // partially confirmed challenger. Keep the accepted stream watermark.
    if (have_scene_ &&
        (now < last_observation_ ||
         now - last_observation_ > config_.scene_timeout)) {
      invalidate_target();
    }
    // A reset/invalid scene must never leave the previous bearing actionable.
    if (scene.header.frame_id.empty() || scene.generation == 0) {
      // Invalidate the advisory, but keep the accepted generation/stamp fence.
      // A malformed snapshot must not make a delayed older stream admissible.
      invalidate_target();
      return invalid(scene, "invalid_scene");
    }
    // A delayed old generation must never roll the policy back to a prior stream.
    if (have_scene_ && scene.generation < generation_) {
      return invalid(scene, "regressed_generation");
    }
    if (config_.require_advancing_scene_stamp &&
        (scene.header.stamp.sec < 0 || scene.header.stamp.nanosec >= 1000000000u ||
         (scene.header.stamp.sec == 0 && scene.header.stamp.nanosec == 0))) {
      return invalid(scene, "invalid_scene_stamp");
    }
    if (have_scene_ && scene.generation == generation_ &&
        scene.header.frame_id == frame_id_ &&
        (scene.header.stamp.sec < last_stamp_.sec ||
         (scene.header.stamp.sec == last_stamp_.sec &&
          (scene.header.stamp.nanosec < last_stamp_.nanosec ||
           (config_.require_advancing_scene_stamp &&
            scene.header.stamp.nanosec == last_stamp_.nanosec))))) {
      return invalid(scene, "regressed_scene_stamp");
    }
    if (have_scene_ && (scene.generation != generation_ ||
        scene.header.frame_id != frame_id_)) {
      selected_id_.clear();
      selected_generation_ = 0;
      clear_pending();
      last_target_ = {};
      quarantined_ids_.clear();
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
    // A discontinuous bearing cannot become actionable again merely because
    // invalidation cleared the incumbent. Keep the identity fenced until it
    // disappears from a subsequent accepted snapshot or the stream changes.
    // Multiple identities may jump in successive accepted scenes. Retain all
    // quarantines while their identities remain visible in this generation.
    // The set is bounded by the identities present in the latest scene.
    for (auto it = quarantined_ids_.begin(); it != quarantined_ids_.end();) {
      bool present = false;
      for (const auto & source : scene.sources) {
        if (source.source_id == *it && source.source_generation == scene.generation) {
          present = true;
          break;
        }
      }
      if (!present) it = quarantined_ids_.erase(it);
      else ++it;
    }
    for (const auto & source : scene.sources) {
      if (quarantined_ids_.count(source.source_id)) continue;
      // Ambiguous or foreign-frame observations cannot establish identity
      // continuity, nor can they poison a valid incumbent's quarantine.
      if (ambiguous_ids.count(source.source_id) ||
          source.source_generation != scene.generation ||
          source.header.frame_id != scene.header.frame_id) continue;
      // Do not lose the reference bearing when the selected source temporarily
      // reports invalid direction evidence. Otherwise it could reappear at an
      // arbitrary angle and bypass the jump gate on fresh acquisition.
      if (config_.maximum_direction_jump_rad > 0.0 &&
          source.source_id == selected_id_ &&
          source.source_generation == selected_generation_ &&
          last_target_.valid &&
          (!source.has_direction ||
           !std::isfinite(source.direction_unit.x) ||
           !std::isfinite(source.direction_unit.y) ||
           !std::isfinite(source.direction_unit.z) ||
           std::abs(source.direction_unit.x * source.direction_unit.x +
                    source.direction_unit.y * source.direction_unit.y +
                    source.direction_unit.z * source.direction_unit.z - 1.0) > 0.05 ||
           (config_.maximum_angular_variance_rad2 > 0.0 &&
            (!source.has_angular_variance ||
             !std::isfinite(source.angular_variance_rad2) ||
             source.angular_variance_rad2 < 0.0 ||
             source.angular_variance_rad2 > config_.maximum_angular_variance_rad2)))) {
        quarantined_ids_.insert(source.source_id);
        continue;
      }
      if (ambiguous_ids.count(source.source_id) || !eligible(source, scene)) continue;
      if (config_.maximum_direction_jump_rad > 0.0 &&
          source.source_id == selected_id_ &&
          source.source_generation == selected_generation_ &&
          last_target_.valid && !direction_consistent(source)) {
        quarantined_ids_.insert(source.source_id);
        continue;
      }
      if (source.source_id == selected_id_ &&
          source.source_generation == selected_generation_) incumbent = &source;
      if (!best || source.activity_score > best->activity_score ||
          (source.activity_score == best->activity_score &&
           source.source_id < best->source_id)) best = &source;
    }
    if (!best) {
      invalidate_target();
      return invalid(scene, "no_eligible_bearing");
    }
    const bool challenger = incumbent && best != incumbent &&
      best->activity_score >= incumbent->activity_score + config_.switch_margin;
    const auto * chosen = incumbent ? incumbent : best;
    if (challenger) {
      // A candidate must win consecutive accepted scenes within a bounded window.
      // Regressed scenes return before reaching this state machine.
      if (pending_id_ != best->source_id ||
          pending_generation_ != best->source_generation ||
          now < pending_since_ ||
          now - pending_since_ > config_.switch_confirmation_window) {
        pending_id_ = best->source_id;
        pending_generation_ = best->source_generation;
        pending_since_ = now;
        pending_count_ = 1;
      } else if (pending_count_ < config_.switch_confirmations) {
        ++pending_count_;
      }
      if (pending_count_ >= config_.switch_confirmations) {
        chosen = best;
        clear_pending();
      }
    } else {
      clear_pending();
      if (!incumbent) chosen = best;
    }
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
      // Preserve generation and last accepted stamp across timeout. Otherwise
      // a delayed old scene could re-arm the advisory after the timer expires.
      invalidate_target();
      return stale;
    }
    return last_target_;
  }

  void reset() {
    have_scene_ = false;
    invalidate_target();
    generation_ = 0;
    frame_id_.clear();
    last_target_ = {};
    last_stamp_ = {};
    quarantined_ids_.clear();
  }

private:
  void invalidate_target() {
    selected_id_.clear();
    selected_generation_ = 0;
    clear_pending();
    last_target_ = {};
  }

  bool direction_consistent(const audition_msgs::msg::AcousticSource & source) const {
    const auto & a = source.direction_unit;
    const auto & b = last_target_.direction_unit;
    // Both vectors were already validated as approximately unit length.
    const double dot = a.x * b.x + a.y * b.y + a.z * b.z;
    const double an = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    const double bn = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
    const double cosine = std::max(-1.0, std::min(1.0, dot / (an * bn)));
    return std::acos(cosine) <= config_.maximum_direction_jump_rad;
  }

  void clear_pending() {
    pending_id_.clear();
    pending_generation_ = 0;
    pending_count_ = 0;
    pending_since_ = {};
  }

  bool fresh_source(const audition_msgs::msg::AcousticSource & source,
                    const audition_msgs::msg::AcousticScene & scene) const {
    // Both stamps are generated by the same scene producer, never compared to wall time.
    const auto & a = scene.header.stamp;
    const auto & b = source.last_seen;
    if (a.sec < 0 || b.sec < 0 || a.nanosec >= 1000000000u ||
        b.nanosec >= 1000000000u) return false;
    const std::int64_t scene_ns = static_cast<std::int64_t>(a.sec) * 1000000000LL + a.nanosec;
    const std::int64_t seen_ns = static_cast<std::int64_t>(b.sec) * 1000000000LL + b.nanosec;
    const std::int64_t delta_ns = scene_ns - seen_ns;
    return delta_ns >= 0 &&
      delta_ns <= config_.maximum_source_age.count() * 1000000LL;
  }

  bool eligible(const audition_msgs::msg::AcousticSource & source,
                const audition_msgs::msg::AcousticScene & scene) const {
    if (config_.maximum_source_age.count() > 0 &&
        !fresh_source(source, scene)) return false;
    if (source.source_id.empty() ||
        source.source_generation != scene.generation ||
        source.header.frame_id != scene.header.frame_id ||
        source.state != audition_msgs::msg::AcousticSource::ACTIVE ||
        !source.has_direction ||
        !std::isfinite(source.activity_score) ||
        source.activity_score < config_.minimum_activity ||
        source.activity_score > 1.0f) return false;
    if (config_.maximum_angular_variance_rad2 > 0.0 &&
        (!source.has_angular_variance ||
         !std::isfinite(source.angular_variance_rad2) ||
         source.angular_variance_rad2 < 0.0 ||
         source.angular_variance_rad2 > config_.maximum_angular_variance_rad2)) return false;
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
  std::string pending_id_;
  // All entries belong to the currently accepted stream generation/frame.
  std::unordered_set<std::string> quarantined_ids_;
  std::uint64_t pending_generation_ = 0;
  std::uint32_t pending_count_ = 0;
  Clock::time_point pending_since_{};
  Clock::time_point last_observation_{};
  builtin_interfaces::msg::Time last_stamp_{};
  audition_msgs::msg::AttentionTarget last_target_{};
};

}  // namespace audition_behaviors
