#pragma once

#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/audio_frame.hpp>
#include <audition_msgs/msg/source_audio_frame.hpp>
#include "audition_perception/capture_session_gate.hpp"

#include <cstdint>
#include <string>

namespace audition_perception {

// A single capture stream is one *non-spatial* source, not a speaker/track.
// No direction, distance, activity detection or channel separation is inferred.
class MonoRouter final {
public:
  struct Output {
    audition_msgs::msg::AcousticSourceArray sources;
    audition_msgs::msg::SourceAudioFrame audio;
  };

  MonoRouter();
  Output process(const audition_msgs::msg::AudioFrame & frame);
  std::uint64_t generation() const noexcept { return generation_; }

private:
  CaptureSessionGate capture_sessions_;
  std::string source_id_;
  std::string frame_id_;
  builtin_interfaces::msg::Time first_seen_;
  std::uint64_t generation_{1};
  std::uint64_t last_sequence_{0};
  std::int64_t last_stamp_ns_{0};
  std::int64_t expected_next_ns_{0};
  bool started_{false};
};

}  // namespace audition_perception
