#include "audition_perception/mono_router.hpp"
#include "audition_perception/spatial_session_id.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace audition_perception {
namespace {
constexpr std::int64_t kNsPerSecond = 1000000000LL;
constexpr std::int64_t kMaxCaptureJitterNs = 250000000LL;
constexpr std::uint32_t kSampleRate = 16000;
constexpr std::uint32_t kMaxFrames = 4096;

void validate(const audition_msgs::msg::AudioFrame & frame) {
  if (frame.header.frame_id.empty() ||
      frame.header.stamp.nanosec >= static_cast<std::uint32_t>(kNsPerSecond) ||
      frame.sample_rate_hz != kSampleRate || frame.channel_count != 1 ||
      frame.frame_count == 0 || frame.frame_count > kMaxFrames ||
      frame.channel_indices.size() != 1 || frame.channel_indices[0] != 0 ||
      frame.layout > audition_msgs::msg::AudioFrame::PLANAR ||
      frame.pcm.size() != frame.frame_count ||
      !std::all_of(frame.pcm.begin(), frame.pcm.end(), [](float sample) {
        return std::isfinite(sample) && sample >= -1.0f && sample <= 1.0f;
      })) {
    throw std::invalid_argument(
      "Mono route requires finite normalized 16kHz one-channel PCM, <=4096 frames, "
      "physical channel 0 and a valid capture frame/time");
  }
}

std::int64_t stampNs(const builtin_interfaces::msg::Time & time) {
  return static_cast<std::int64_t>(time.sec) * kNsPerSecond + time.nanosec;
}
}  // namespace

MonoRouter::MonoRouter()
: spatial_session_id_(makeSpatialSessionId()),
  source_id_("mono-capture-" + spatial_session_id_) {}

MonoRouter::Output MonoRouter::process(const audition_msgs::msg::AudioFrame & frame) {
  // Reject malformed frames before touching the source epoch or publishing
  // anything. No resampling or implicit channel mixing is performed.
  validate(frame);
  const auto session = capture_sessions_.inspect(frame.capture_session_id);
  const auto stamp = stampNs(frame.header.stamp);
  if (session == CaptureSessionGate::Decision::Changed || (started_ &&
      (frame.header.frame_id != frame_id_ ||
       frame.sequence_number != last_sequence_ + 1 ||
       stamp < last_stamp_ns_ ||
       stamp > expected_next_ns_ + kMaxCaptureJitterNs))) {
    ++generation_;
    started_ = false;
  }
  if (!started_) {
    frame_id_ = frame.header.frame_id;
    first_seen_ = frame.header.stamp;
    started_ = true;
  }
  capture_sessions_.commit(frame.capture_session_id);
  last_sequence_ = frame.sequence_number;
  last_stamp_ns_ = stamp;
  expected_next_ns_ = stamp +
    static_cast<std::int64_t>(frame.frame_count) * kNsPerSecond / kSampleRate;

  Output result;
  result.sources.header = frame.header;
  result.sources.generation = generation_;
  result.sources.spatial_session_id = spatial_session_id_;
  result.sources.sequence_number = ++snapshot_sequence_;
  audition_msgs::msg::AcousticSource source;
  source.header = frame.header;
  source.source_id = source_id_;
  source.source_generation = generation_;
  source.state = audition_msgs::msg::AcousticSource::ACTIVE;
  source.first_seen = first_seen_;
  source.last_seen = frame.header.stamp;
  // All geometry flags stay false. ACTIVE means capture present, not speech.
  // A zero activity_score here is *unknown*, not evidence of silence.
  result.sources.sources.push_back(std::move(source));

  result.audio.header = frame.header;
  result.audio.source_id = source_id_;
  result.audio.source_generation = generation_;
  result.audio.spatial_session_id = spatial_session_id_;
  result.audio.sequence_number = frame.sequence_number;
  result.audio.sample_rate_hz = kSampleRate;
  result.audio.frame_count = frame.frame_count;
  result.audio.pcm = frame.pcm;
  return result;
}

}  // namespace audition_perception
