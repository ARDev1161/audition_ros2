#include "audition_perception/semantic_epoch_gate.hpp"

#include <audition_msgs/msg/acoustic_source.hpp>
#include <set>
#include <string>
#include <utility>

namespace audition_perception {
namespace {
std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

void SemanticEpochGate::beginReload() noexcept {
  admitted_.clear();
  reload_pending_ = have_stream_;
  reload_audio_fence_ = have_stream_;
  reload_fence_generation_ = generation_;
  reload_cutoff_ns_ = last_snapshot_stamp_ns_;
}


SemanticEpochGate::SnapshotResult SemanticEpochGate::observe(
  const audition_msgs::msg::AcousticSourceArray & snapshot)
{
  using State = audition_msgs::msg::AcousticSource;
  const auto reject = [this]() {
    ++rejected_source_snapshots_;
    return SnapshotResult::Rejected;
  };
  if (snapshot.generation == 0 || snapshot.sequence_number == 0 ||
      (have_stream_ && snapshot.sequence_number <= last_snapshot_sequence_) ||
      snapshot.header.frame_id.empty() ||
      snapshot.header.stamp.nanosec >= 1000000000U ||
      snapshot.sources.size() > 256U ||
      (have_stream_ && snapshot.generation < generation_) ||
      (reload_pending_ && have_stream_ &&
       snapshot.generation == generation_ &&
       stampNs(snapshot.header.stamp) <= reload_cutoff_ns_) ||
      (have_stream_ && snapshot.generation == generation_ &&
       snapshot.header.frame_id != frame_id_)) {
    return reject();
  }

  std::set<std::pair<std::uint64_t, std::string>> seen;
  std::set<std::string> admitted;
  for (const auto & source : snapshot.sources) {
    const bool current = source.source_generation == snapshot.generation;
    // SourceLifecycle emits one-shot ENDED records from the previous epoch
    // alongside the first snapshot of a new epoch, possibly in a new frame.
    const bool previous_ended = have_stream_ &&
      snapshot.generation > generation_ &&
      source.source_generation == generation_ &&
      source.state == State::ENDED &&
      source.header.frame_id == frame_id_;
    if (source.source_id.empty() ||
        (source.state != State::ACTIVE && source.state != State::LOST &&
         source.state != State::ENDED) ||
        (!current && !previous_ended) ||
        (current && source.header.frame_id != snapshot.header.frame_id) ||
        !seen.emplace(source.source_generation, source.source_id).second) {
      return reject();  // Atomic: the previous admission set remains intact.
    }
    if (current && source.state != State::ENDED) {
      admitted.insert(source.source_id);
    }
  }

  const bool reset = have_stream_ && snapshot.generation != generation_;
  if (reset) ++stream_resets_;
  have_stream_ = true;
  generation_ = snapshot.generation;
  last_snapshot_sequence_ = snapshot.sequence_number;
  frame_id_ = snapshot.header.frame_id;
  admitted_.swap(admitted);
  last_snapshot_stamp_ns_ = stampNs(snapshot.header.stamp);
  if (reload_pending_) {
    reload_pending_ = false;
    if (snapshot.generation == reload_fence_generation_) {
      // Reject queued PCM captured at/before the re-admission snapshot.
      reload_cutoff_ns_ = last_snapshot_stamp_ns_;
    } else {
      reload_audio_fence_ = false;  // Upstream generation already fences PCM.
    }
  }
  return reset ? SnapshotResult::NewGeneration : SnapshotResult::Accepted;
}

bool SemanticEpochGate::accept(
  const audition_msgs::msg::SourceAudioFrame & audio)
{
  // Fail closed until canonical source admission has reached this process.
  // DDS ordering between different topics is not guaranteed: the first audio
  // chunk may be dropped, but no orphan/stale chunk can create a worker.
  const bool ok = !reload_pending_ && have_stream_ &&
    !(reload_audio_fence_ && audio.source_generation == reload_fence_generation_ &&
      stampNs(audio.header.stamp) <= reload_cutoff_ns_) &&
    audio.source_generation == generation_ &&
    audio.header.frame_id == frame_id_ &&
    admitted_.count(audio.source_id) != 0;
  if (ok) ++accepted_audio_frames_;
  else ++dropped_audio_frames_;
  return ok;
}

void SemanticEpochGate::status(
  audition_msgs::srv::GetSemanticStatus::Response & out) const
{
  out.have_stream = have_stream_;
  out.source_generation = generation_;
  out.last_source_sequence = last_snapshot_sequence_;
  out.frame_id = frame_id_;
  out.admitted_sources = static_cast<std::uint32_t>(admitted_.size());
  out.accepted_audio_frames = accepted_audio_frames_;
  out.dropped_audio_frames = dropped_audio_frames_;
  out.rejected_source_snapshots = rejected_source_snapshots_;
  out.stream_resets = stream_resets_;
}

}  // namespace audition_perception
