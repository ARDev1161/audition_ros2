#pragma once

#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <audition_msgs/srv/get_semantic_status.hpp>

#include <cstdint>
#include <map>
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
  // On successful worker replacement, fence all PCM until a strictly newer
  // canonical snapshot arrives. The upstream generation is NOT fabricated.
  void beginReload() noexcept;
  bool reloadPending() const noexcept { return reload_pending_; }
  void rejectInvalidAudio() noexcept { ++dropped_audio_frames_; }
  void status(audition_msgs::srv::GetSemanticStatus::Response & out) const;

private:
  bool reload_pending_{false};
  bool reload_audio_fence_{false};
  std::uint64_t reload_fence_generation_{0};
  std::int64_t last_snapshot_stamp_ns_{0};
  std::int64_t reload_cutoff_ns_{0};
  bool have_stream_{false};
  std::uint64_t generation_{0};
  std::uint64_t last_snapshot_sequence_{0};
  std::string scene_session_id_;
  std::string spatial_session_id_;
  // Retain every accepted incarnation: a delayed old session can never
  // become authoritative again. Fail closed after 64 scene incarnations.
  std::set<std::string> seen_scene_sessions_;
  std::uint64_t scene_session_resets_{0};
  std::string frame_id_;
  std::set<std::string> admitted_;
  struct AudioCursor {
    std::uint64_t sequence{0};
    std::int64_t capture_ns{0};
  };
  std::map<std::string, AudioCursor> audio_cursors_;
  std::uint64_t out_of_order_audio_frames_{0};
  std::uint64_t accepted_audio_frames_{0};
  std::uint64_t dropped_audio_frames_{0};
  std::uint64_t rejected_source_snapshots_{0};
  std::uint64_t stream_resets_{0};
};

}  // namespace audition_perception
