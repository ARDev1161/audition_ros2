#include "audition_perception/spatial_perception_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef AUDITION_ROS2_WITH_ODAS
#include <audition/audio/audio_buffer.hpp>
#include <audition/backends/odas/odas_spatial_engine.hpp>
#include <audition/backends/odas/respeaker_usb4_profile.hpp>
#include <audition/core/time.hpp>
#endif

#include <rclcpp_components/register_node_macro.hpp>

namespace audition_perception {
namespace {

bool validFrame(const audition_msgs::msg::AudioFrame & msg) {
  const auto samples = static_cast<std::uint64_t>(msg.frame_count) * msg.channel_count;
  if (msg.sample_rate_hz == 0 || msg.frame_count == 0 ||
      msg.frame_count > 8192 || msg.channel_count == 0 || msg.channel_count > 32 ||
      samples != msg.pcm.size() ||
      msg.channel_indices.size() != msg.channel_count ||
      msg.layout > audition_msgs::msg::AudioFrame::PLANAR ||
      msg.header.frame_id.empty()) {
    return false;
  }
  for (std::size_t i = 0; i < msg.channel_indices.size(); ++i) {
    // The capture adapter currently preserves the physical channel ordering.
    if (msg.channel_indices[i] != i) return false;
  }
  return std::all_of(msg.pcm.begin(), msg.pcm.end(), [](float v) {
    return std::isfinite(v) && v >= -1.0f && v <= 1.0f;
  });
}

#ifdef AUDITION_ROS2_WITH_ODAS
std::int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
builtin_interfaces::msg::Time asStamp(std::int64_t ns) {
  // Ingestion stamps are already ROS-domain timestamps (including sim time).
  return rclcpp::Time(ns, RCL_ROS_TIME).to_msg();
}
#endif
}  // namespace

class SpatialPerceptionNode::Impl {
public:
  explicit Impl(SpatialPerceptionNode & node, const std::string & backend)
  : backend_(backend), frame_id_(node.declare_parameter<std::string>(
      "microphone_frame", "microphone_array")) {
    if (backend_ == "disabled") return;
    if (backend_ != "odas") {
      throw std::invalid_argument("Unknown spatial backend: " + backend_);
    }
#ifndef AUDITION_ROS2_WITH_ODAS
    throw std::runtime_error(
      "ODAS backend requested but audition_perception was built without "
      "-DAUDITION_ROS2_WITH_ODAS=ON");
#else
    if (!node.declare_parameter<bool>("geometry_calibrated", false)) {
      throw std::invalid_argument("ODAS requires geometry_calibrated=true");
    }
    auto channels = node.declare_parameter<std::vector<std::int64_t>>(
      "input_channel_indices", std::vector<std::int64_t>{});
    auto positions = node.declare_parameter<std::vector<double>>(
      "microphone_positions_m", std::vector<double>{});
    const auto rate = node.declare_parameter<int>("sample_rate_hz", 16000);
    const auto hop = node.declare_parameter<int>("odas_hop_frames", 128);
    const auto frame = node.declare_parameter<int>("odas_frame_size", 256);
    const auto limit = node.declare_parameter<int>("max_tracks", 4);
    const bool separation = node.declare_parameter<bool>("enable_separation", true);
    const bool respeaker_profile = node.declare_parameter<bool>(
      "respeaker_angular_profile", false);

    if (channels.size() < 2 || channels.size() > 16 ||
        positions.size() != channels.size() * 3 ||
        rate < 8000 || rate > 192000 ||
        hop <= 0 || hop > 2048 || frame < hop || frame > 8192 ||
        limit <= 0 || limit > 64 || frame_id_.empty()) {
      throw std::invalid_argument("Invalid ODAS geometry, frame size or rate");
    }
    for (auto index : channels) {
      if (index < 0 || index >= 32) throw std::invalid_argument("Invalid input channel index");
      input_channels_.push_back(static_cast<std::size_t>(index));
    }
    auto sorted = input_channels_;
    std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
      throw std::invalid_argument("Duplicate input channel indices");
    }
    audition::OdasOptions options;
    options.sample_rate_hz = static_cast<std::uint32_t>(rate);
    options.hop_size = static_cast<std::uint32_t>(hop);
    options.frame_size = static_cast<std::uint32_t>(frame);
    options.input_channels = input_channels_;
    options.sst.max_tracks = static_cast<std::uint32_t>(limit);
    options.sss.enabled = separation;
    for (std::size_t i = 0; i < channels.size(); ++i) {
      const audition::Vec3 position{positions[3*i], positions[3*i+1], positions[3*i+2]};
      if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
          !std::isfinite(position.z)) {
        throw std::invalid_argument("Non-finite microphone position");
      }
      options.microphone_array.microphones.push_back(audition::MicrophoneGeometry{position});
    }
    if (respeaker_profile) {
      audition::applyReSpeakerUsb4MicAngularProfile(options);
    }
    audition::validateOdasOptions(options);
    sample_rate_ = options.sample_rate_hz;
    hop_ = options.hop_size;
    odas_ = std::make_unique<audition::OdasSpatialEngine>(std::move(options));
#endif
  }

  void process(const audition_msgs::msg::AudioFrame & msg,
      const rclcpp::Publisher<audition_msgs::msg::AcousticSourceArray>::SharedPtr & source_pub,
      const rclcpp::Publisher<audition_msgs::msg::SourceAudioFrame>::SharedPtr & audio_pub) {
    if (!validFrame(msg)) throw std::invalid_argument("Invalid AudioFrame payload/layout");

    audition_msgs::msg::AcousticSourceArray snapshot;
    snapshot.header = msg.header;
    snapshot.generation = generation_;

    if (backend_ == "disabled") {
      source_pub->publish(std::move(snapshot));
      return;
    }
#ifdef AUDITION_ROS2_WITH_ODAS
    if (msg.header.frame_id != frame_id_ || msg.sample_rate_hz != sample_rate_ ||
        *std::max_element(input_channels_.begin(), input_channels_.end()) >= msg.channel_count) {
      throw std::invalid_argument("Input does not match the calibrated ODAS frame/rate/channels");
    }
    // A discontinuity must never join samples from different source generations.
    const auto stamp_ns = toNanoseconds(msg.header.stamp);
    if ((have_sequence_ && msg.sequence_number != last_sequence_ + 1) ||
        (have_stamp_ && stamp_ns < last_stamp_ns_)) {
      restart();
      snapshot.generation = generation_;
    }
    have_sequence_ = true;
    last_sequence_ = msg.sequence_number;
    have_stamp_ = true;
    last_stamp_ns_ = stamp_ns;

    if (pending_channels_ != 0 && pending_channels_ != msg.channel_count) {
      restart();
      audition_msgs::msg::AcousticSourceArray empty;
      empty.header = msg.header;
      empty.generation = generation_;
      source_pub->publish(empty);
      throw std::invalid_argument("Channel count changed; ODAS restarted");
    }
    if (msg.frame_count > hop_ * 4) {
      throw std::invalid_argument("AudioFrame exceeds bounded ODAS ingress chunk");
    }
    if (pending_.empty()) pending_stamp_ns_ = stamp_ns;
    const auto frame_count = static_cast<std::size_t>(msg.frame_count);
    const auto channel_count = static_cast<std::size_t>(msg.channel_count);
    const auto pending_count = pending_.size();
    const auto maximum = (hop_ * channel_count) * 8;
    if (pending_count + msg.pcm.size() > maximum) {
      restart();
      snapshot.generation = generation_;
      pending_stamp_ns_ = stamp_ns;
    }
    if (msg.layout == audition_msgs::msg::AudioFrame::INTERLEAVED) {
      pending_.insert(pending_.end(), msg.pcm.begin(), msg.pcm.end());
    } else {
      for (std::size_t n = 0; n < frame_count; ++n)
        for (std::size_t ch = 0; ch < channel_count; ++ch)
          pending_.push_back(msg.pcm[ch * frame_count + n]);
    }
    pending_channels_ = msg.channel_count;
    const auto hop_samples = static_cast<std::size_t>(hop_) * channel_count;
    while (pending_.size() >= hop_samples) {
      std::vector<float> hop_pcm(pending_.begin(), pending_.begin() + hop_samples);
      pending_.erase(pending_.begin(), pending_.begin() + hop_samples);
      const auto hop_stamp = pending_stamp_ns_;
      pending_stamp_ns_ += static_cast<std::int64_t>(hop_) * 1000000000LL / sample_rate_;
      audition::AudioBuffer buffer{
        std::move(hop_pcm),
        audition::AudioFormat{sample_rate_, msg.channel_count, audition::AudioLayout::Interleaved},
        audition::Timestamp{hop_stamp, {audition::ClockDomain::External, 1}},
        ++hop_sequence_
      };
      auto result = odas_->process(buffer.view());
      std::unordered_map<std::uint64_t, bool> seen;
      snapshot.sources.clear();
      snapshot.header.stamp = asStamp(hop_stamp);
      for (const auto & track : result.tracks) {
        const auto id = track.track_id.value();
        if (!track.track_id.valid()) continue;
        auto it = tracks_.find(id);
        if (it == tracks_.end()) {
          const std::string source_id = frame_id_ + "-" + std::to_string(session_) +
            "-" + std::to_string(++source_counter_);
          it = tracks_.emplace(id, source_id).first;
        }
        const auto vec = track.direction.direction.vector();
        if (!std::isfinite(vec.x) || !std::isfinite(vec.y) || !std::isfinite(vec.z)) continue;
        audition_msgs::msg::AcousticSource source;
        source.header.frame_id = frame_id_;
        source.header.stamp = asStamp(hop_stamp);
        source.source_id = it->second;
        source.source_generation = generation_;
        source.spatial_track_id = std::to_string(id);
        source.state = audition_msgs::msg::AcousticSource::ACTIVE;
        source.has_direction = true;
        source.direction_unit.x = vec.x;
        source.direction_unit.y = vec.y;
        source.direction_unit.z = vec.z;
        if (track.direction.angular_variance_rad2.has_value()) {
          source.has_angular_variance = true;
          source.angular_variance_rad2 = *track.direction.angular_variance_rad2;
        }
        // Never synthesize a metric distance or covariance from ODAS DOA.
        source.activity_score = static_cast<float>(track.activity.value());
        source.first_seen = asStamp(track.first_seen.nanoseconds());
        source.last_seen = asStamp(track.last_seen.nanoseconds());
        snapshot.sources.push_back(std::move(source));
        seen[id] = true;
      }
      for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (!seen.count(it->first)) it = tracks_.erase(it);
        else ++it;
      }
      source_pub->publish(snapshot);
      for (const auto & frame : result.separated_frames) {
        const auto it = tracks_.find(frame.track_id.value());
        if (it == tracks_.end()) continue;
        const auto & samples = frame.audio.samples();
        if (samples.empty()) continue;
        audition_msgs::msg::SourceAudioFrame out;
        out.header.frame_id = frame_id_;
        out.header.stamp = asStamp(frame.audio.captureTime().nanoseconds());
        out.source_id = it->second;
        out.source_generation = generation_;
        out.sequence_number = frame.audio.sequenceNumber();
        out.sample_rate_hz = frame.audio.format().sample_rate_hz;
        out.frame_count = static_cast<std::uint32_t>(frame.audio.frameCount());
        out.pcm = samples;
        audio_pub->publish(std::move(out));
      }
    }
#endif
  }

private:
#ifdef AUDITION_ROS2_WITH_ODAS
  void restart() {
    odas_->reset();
    pending_.clear();
    tracks_.clear();
    pending_channels_ = 0;
    have_sequence_ = false;
    have_stamp_ = false;
    ++generation_;
  }
  std::unique_ptr<audition::OdasSpatialEngine> odas_;
  std::vector<std::size_t> input_channels_{};
  std::unordered_map<std::uint64_t, std::string> tracks_{};
  std::vector<float> pending_{};
  std::uint32_t sample_rate_{0}, hop_{0}, pending_channels_{0};
  std::int64_t pending_stamp_ns_{0}, last_stamp_ns_{0};
  std::uint64_t last_sequence_{0}, hop_sequence_{0}, source_counter_{0};
  std::uint64_t session_{static_cast<std::uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count())};
  bool have_sequence_{false}, have_stamp_{false};
#endif
  std::string backend_, frame_id_;
  std::uint64_t generation_{1};
};

SpatialPerceptionNode::SpatialPerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("spatial_perception", options) {
  backend_ = declare_parameter<std::string>("backend", "disabled");
  impl_ = std::make_unique<Impl>(*this, backend_);
  audio_sub_ = create_subscription<audition_msgs::msg::AudioFrame>(
    "audio/raw", rclcpp::SensorDataQoS(),
    [this](const audition_msgs::msg::AudioFrame & msg) { onAudio(msg); });
  sources_pub_ = create_publisher<audition_msgs::msg::AcousticSourceArray>(
    "internal/spatial_sources", rclcpp::QoS(10).reliable());
  separated_pub_ = create_publisher<audition_msgs::msg::SourceAudioFrame>(
    "internal/source_audio", rclcpp::SensorDataQoS());
  RCLCPP_INFO(get_logger(), "Spatial backend: %s", backend_.c_str());
}
SpatialPerceptionNode::~SpatialPerceptionNode() = default;

void SpatialPerceptionNode::onAudio(const audition_msgs::msg::AudioFrame & msg) {
  try {
    impl_->process(msg, sources_pub_, separated_pub_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
      "Spatial processing rejected frame: %s", error.what());
  }
}

}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::SpatialPerceptionNode)
