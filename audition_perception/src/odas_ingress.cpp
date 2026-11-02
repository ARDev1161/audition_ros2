#include "audition_perception/odas_ingress.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace audition_perception {
namespace {
constexpr std::int64_t kSecond = 1000000000LL;
constexpr std::int64_t kMaxJitter = 250000000LL;
std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<std::int64_t>(stamp.sec) * kSecond + stamp.nanosec;
}
}  // namespace

OdasIngress::OdasIngress(std::uint32_t rate, std::uint32_t hop,
                         std::string frame_id, std::uint32_t min_channels,
                         std::uint32_t max_pending_hops)
: rate_(rate), hop_(hop), min_channels_(min_channels),
  max_pending_hops_(max_pending_hops), frame_id_(std::move(frame_id)) {
  if (rate_ == 0 || hop_ == 0 || hop_ > 2048 ||
      min_channels_ == 0 || min_channels_ > 32 ||
      max_pending_hops_ < 2 || max_pending_hops_ > 32 || frame_id_.empty()) {
    throw std::invalid_argument("Invalid ODAS ingress limits");
  }
  pending_.reserve(static_cast<std::size_t>(hop_) * 8U * min_channels_);
}

void OdasIngress::validate(const audition_msgs::msg::AudioFrame & msg) const {
  const auto sample_count =
    static_cast<std::uint64_t>(msg.frame_count) * msg.channel_count;
  if (msg.header.frame_id != frame_id_ ||
      msg.header.stamp.nanosec >= static_cast<std::uint32_t>(kSecond) ||
      msg.sample_rate_hz != rate_ ||
      msg.frame_count == 0 || msg.frame_count > 4U * hop_ ||
      msg.channel_count < min_channels_ || msg.channel_count > 32 ||
      msg.channel_indices.size() != msg.channel_count ||
      sample_count != msg.pcm.size() ||
      msg.layout > audition_msgs::msg::AudioFrame::PLANAR ||
      !std::all_of(msg.pcm.begin(), msg.pcm.end(), [](float v) {
        return std::isfinite(v) && v >= -1.0f && v <= 1.0f;
      })) {
    throw std::invalid_argument("ODAS requires bounded normalized PCM, calibrated frame/rate/channels");
  }
  for (std::size_t i = 0; i < msg.channel_indices.size(); ++i) {
    if (msg.channel_indices[i] != i) {
      throw std::invalid_argument("ODAS requires original physical channel order");
    }
  }
}

void OdasIngress::reset() noexcept {
  pending_.clear();
  pending_channels_ = 0;
  have_previous_ = false;
  hop_sequence_ = 0;
  ++generation_;
}

std::uint32_t OdasIngress::bufferedFrames() const noexcept {
  return pending_channels_ == 0 ? 0U :
    static_cast<std::uint32_t>(pending_.size() / pending_channels_);
}

void OdasIngress::backendFailure() noexcept {
  reset();
  ++stats_.backend_resets;
}

OdasIngress::Batch OdasIngress::push(
  const audition_msgs::msg::AudioFrame & msg) {
  CaptureSessionGate::Decision session;
  try {
    validate(msg);
    session = capture_sessions_.inspect(msg.capture_session_id);
  } catch (...) {
    ++stats_.rejected_frames;
    throw;
  }
  const auto stamp = stampNs(msg.header.stamp);
  Batch batch;
  const bool discontinuity = session == CaptureSessionGate::Decision::Changed ||
    (have_previous_ && (msg.sequence_number != last_sequence_ + 1 ||
     stamp < last_stamp_ns_ ||
     stamp > expected_next_ns_ + kMaxJitter ||
     (pending_channels_ != 0 && pending_channels_ != msg.channel_count)));
  if (discontinuity) {
    reset();
    ++stats_.discontinuity_resets;
    batch.reset = true;
  }

  // Commit only after payload and session validation. Even an oversized
  // dropped chunk establishes the new incarnation and fences old replay.
  capture_sessions_.commit(msg.capture_session_id);
  const auto maximum = static_cast<std::size_t>(hop_) *
    max_pending_hops_ * msg.channel_count;
  if (pending_.size() + msg.pcm.size() > maximum) {
    // Do not partially process a frame or join PCM across a dropped chunk.
    // A smaller configured buffer can reject an oversized but otherwise
    // valid frame; the next valid frame starts a fresh generation.
    reset();
    ++stats_.overflow_drops;
    batch.reset = true;
    batch.generation = generation_;
    return batch;
  }

  if (pending_.empty()) pending_stamp_ns_ = stamp;
  if (msg.layout == audition_msgs::msg::AudioFrame::INTERLEAVED) {
    pending_.insert(pending_.end(), msg.pcm.begin(), msg.pcm.end());
  } else {
    for (std::size_t n = 0; n < msg.frame_count; ++n) {
      for (std::size_t ch = 0; ch < msg.channel_count; ++ch) {
        pending_.push_back(msg.pcm[ch * msg.frame_count + n]);
      }
    }
  }
  pending_channels_ = msg.channel_count;
  have_previous_ = true;
  last_sequence_ = msg.sequence_number;
  last_stamp_ns_ = stamp;
  expected_next_ns_ = stamp +
    static_cast<std::int64_t>(msg.frame_count) * kSecond / rate_;
  ++stats_.accepted_frames;

  const auto hop_samples = static_cast<std::size_t>(hop_) * msg.channel_count;
  while (pending_.size() >= hop_samples) {
    Hop hop;
    hop.pcm.assign(pending_.begin(), pending_.begin() + hop_samples);
    pending_.erase(pending_.begin(), pending_.begin() + hop_samples);
    hop.capture_ns = pending_stamp_ns_;
    hop.sequence = ++hop_sequence_;
    hop.channels = msg.channel_count;
    batch.hops.push_back(std::move(hop));
    pending_stamp_ns_ += static_cast<std::int64_t>(hop_) * kSecond / rate_;
    ++stats_.staged_hops;
  }
  batch.generation = generation_;
  return batch;
}
}  // namespace audition_perception
