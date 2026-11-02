#pragma once

#include <audition_msgs/msg/audio_frame.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace audition_perception {

// Bounded, model-independent ODAS input staging. All state changes occur only
// after validating a complete frame; discontinuities create a new source epoch.
// A rejected frame never contributes PCM or advances the capture clock.
class OdasIngress final {
public:
  struct Hop {
    std::vector<float> pcm;
    std::int64_t capture_ns{0};
    std::uint64_t sequence{0};
    std::uint32_t channels{0};
  };
  struct Batch {
    std::vector<Hop> hops;
    std::uint64_t generation{1};
    bool reset{false};
  };
  struct Stats {
    std::uint64_t accepted_frames{0};
    std::uint64_t rejected_frames{0};
    std::uint64_t overflow_drops{0};
    std::uint64_t discontinuity_resets{0};
    std::uint64_t backend_resets{0};
    std::uint64_t processed_hops{0};
  };

  OdasIngress(std::uint32_t sample_rate, std::uint32_t hop_frames,
              std::string frame_id, std::uint32_t min_channels,
              std::uint32_t max_pending_hops = 8);
  Batch push(const audition_msgs::msg::AudioFrame & frame);
  void backendFailure() noexcept;
  std::uint64_t generation() const noexcept { return generation_; }
  std::uint32_t bufferedFrames() const noexcept;
  const Stats & stats() const noexcept { return stats_; }

private:
  void reset() noexcept;
  void validate(const audition_msgs::msg::AudioFrame & frame) const;
  std::uint32_t rate_, hop_, min_channels_, max_pending_hops_;
  std::string frame_id_;
  std::vector<float> pending_;
  std::uint32_t pending_channels_{0};
  std::int64_t pending_stamp_ns_{0}, expected_next_ns_{0}, last_stamp_ns_{0};
  std::uint64_t last_sequence_{0}, hop_sequence_{0}, generation_{1};
  bool have_previous_{false};
  Stats stats_;
};

}  // namespace audition_perception
