#pragma once

#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <audition_msgs/srv/get_semantic_status.hpp>

#include <cstdint>
#include <set>
#include <string>

namespace audition_perception {

// An authoritative fence from the canonical /sources stream to asynchronous
// semantic workers. It never guesses identity from a PCM packet alone.
class SemanticEpochGate final {
public:
  enum class SnapshotResult { Rejected, Accepted, NewGeneration };

  SnapshotResult observe(const audition_msgs::msg::AcousticSourceArray & snapshot);
  bool accept(const audition_msgs::msg::SourceAudioFrame & audio);
  void rejectInvalidAudio() noexcept { ++dropped_audio_frames_; }
  void status(audition_msgs::srv::GetSemanticStatus::Response & out) const;

private:
  bool have_stream_{false};
  std::uint64_t generation_{0};
  std::string frame_id_;
  std::set<std::string> admitted_;
  std::uint64_t accepted_audio_frames_{0};
  std::uint64_t dropped_audio_frames_{0};
  std::uint64_t rejected_source_snapshots_{0};
  std::uint64_t stream_resets_{0};
};

}  // namespace audition_perception
