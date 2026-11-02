#include "audition_perception/acoustic_scene_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <set>
#include <string>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace audition_perception {
namespace {
std::string makeSceneSessionId() {
  // This is an incarnation discriminator, NOT authentication or an
  // authorization token. A new scene process gets a fresh 128-bit value.
  std::random_device random;
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) {
    stream << std::setw(8) << static_cast<std::uint32_t>(random());
  }
  return stream.str();
}
}  // namespace

AcousticSceneNode::AcousticSceneNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("acoustic_scene", options),
  scene_session_id_(makeSceneSessionId())
{
  const auto lost_ms = declare_parameter<int>("source_lost_after_ms", 300);
  const auto end_ms = declare_parameter<int>("source_end_after_ms", 1500);
  const auto capacity = declare_parameter<int>("max_sources", 16);
  const auto final_grace_ms = declare_parameter<int>("late_transcript_grace_ms", 5000);
  if (capacity <= 0) throw std::invalid_argument("max_sources must be positive");
  lifecycle_ = std::make_unique<SourceLifecycle>(
    std::chrono::milliseconds(lost_ms), std::chrono::milliseconds(end_ms),
    static_cast<std::size_t>(capacity), std::chrono::milliseconds(final_grace_ms));

  event_ledger_ = std::make_unique<EventLedger>(
    static_cast<std::size_t>(declare_parameter<int>("max_active_events", 128)));
  transcript_sub_ = create_subscription<audition_msgs::msg::Transcript>(
    "internal/transcripts", rclcpp::QoS(20).reliable(),
    [this](const audition_msgs::msg::Transcript & msg) {onTranscript(msg);});
  transcript_pub_ = create_publisher<audition_msgs::msg::Transcript>(
    "transcripts", rclcpp::QoS(20).reliable());
  event_sub_ = create_subscription<audition_msgs::msg::AcousticEvent>(
    "internal/events", rclcpp::QoS(20).reliable(),
    [this](const audition_msgs::msg::AcousticEvent & msg) { onEvent(msg); });
  class_sub_ = create_subscription<audition_msgs::msg::SoundClassification>(
    "internal/classifications", rclcpp::QoS(20).reliable(),
    [this](const audition_msgs::msg::SoundClassification & msg) { onClassification(msg); });
  event_pub_ = create_publisher<audition_msgs::msg::AcousticEvent>(
    "events", rclcpp::QoS(20).reliable());
  class_pub_ = create_publisher<audition_msgs::msg::SoundClassification>(
    "classifications", rclcpp::QoS(20).reliable());
  source_sub_ = create_subscription<audition_msgs::msg::AcousticSourceArray>(
    "internal/spatial_sources", rclcpp::QoS(10).reliable(),
    [this](const audition_msgs::msg::AcousticSourceArray & msg) { onSources(msg); });
  source_pub_ = create_publisher<audition_msgs::msg::AcousticSourceArray>(
    "sources", rclcpp::QoS(10).reliable());
  scene_pub_ = create_publisher<audition_msgs::msg::AcousticScene>(
    "scene", rclcpp::QoS(10).reliable());
  timer_ = create_wall_timer(std::chrono::milliseconds(100),
    [this]() { onTimer(); });
}

void AcousticSceneNode::publish(SourceLifecycle::Update update) {
  if (!update.changed) return;
  // End every event whose acoustic source vanished from the canonical scene.
  // This is deliberately independent of delayed semantic/ASR worker completion.
  std::set<std::pair<std::uint64_t, std::string>> live;
  for (const auto & source : update.scene.sources) {
    live.emplace(source.source_generation, source.source_id);
  }
  for (const auto & active : event_ledger_->active()) {
    if (live.count({active.source_generation, active.source_id})) continue;
    auto cancelled = active;
    cancelled.transition = audition_msgs::msg::AcousticEvent::CANCELLED;
    cancelled.header.stamp = update.scene.header.stamp;
    cancelled.termination_reason = active.source_generation == update.scene.generation
      ? "source_ended" : "spatial_stream_reset";
    if (event_ledger_->apply(cancelled)) event_pub_->publish(std::move(cancelled));
  }
  update.scene.active_events = event_ledger_->active();
  // /scene contains only ACTIVE and LOST sources; /sources additionally
  // carries one-shot ENDED notifications. Both share the same generation.
  // A single process-local order key also covers timer and event updates.
  update.sources.sequence_number = ++canonical_sequence_;
  update.sources.scene_session_id = scene_session_id_;
  source_pub_->publish(std::move(update.sources));
  scene_pub_->publish(std::move(update.scene));
}

void AcousticSceneNode::onSources(
  const audition_msgs::msg::AcousticSourceArray & msg)
{
  try {
    if (have_stream_ && msg.generation < stream_generation_) {
      RCLCPP_WARN(get_logger(), "Ignoring stale spatial stream generation");
      return;
    }
    // Validate/mutate source state FIRST. Malformed snapshots must not cancel
    // existing events or alter stream generation/frame identity.
    auto update = lifecycle_->observe(msg, SourceLifecycle::Clock::now());
    stream_generation_ = msg.generation;
    have_stream_ = true;
    frame_id_ = msg.header.frame_id;
    publish(std::move(update));
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Invalid spatial source snapshot: %s", e.what());
  }
}

void AcousticSceneNode::onTimer() {
  if (frame_id_.empty()) return;
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = frame_id_;
  publish(lifecycle_->tick(SourceLifecycle::Clock::now(), header));
}

void AcousticSceneNode::onEvent(const audition_msgs::msg::AcousticEvent & msg) {
  if (!have_stream_ || msg.source_generation != stream_generation_ ||
      msg.header.frame_id != frame_id_ ||
      !lifecycle_->isLive(msg.source_id, msg.source_generation) ||
      !event_ledger_->apply(msg)) return;
  event_pub_->publish(msg);
  // Reconstruct the current scene after event transition. Publishing this
  // snapshot must not update source observation freshness/TTL.
  std_msgs::msg::Header header;
  header.stamp = now(); // scene snapshot time; event keeps audio observation stamp
  header.frame_id = frame_id_;
  auto snapshot = lifecycle_->tick(SourceLifecycle::Clock::now(), header);
  snapshot.changed = true;
  publish(std::move(snapshot));
}

void AcousticSceneNode::onClassification(
  const audition_msgs::msg::SoundClassification & msg)
{
  if (!have_stream_ || msg.source_generation != stream_generation_ ||
      msg.header.frame_id != frame_id_ || msg.source_id.empty() ||
      msg.class_label.empty() ||
      !lifecycle_->isLive(msg.source_id, msg.source_generation) ||
      !std::isfinite(msg.score) ||
      msg.score < 0.0f || msg.score > 1.0f) return;
  class_pub_->publish(msg);
}
void AcousticSceneNode::onTranscript(const audition_msgs::msg::Transcript & msg) {
  if (!have_stream_ || msg.source_generation != stream_generation_ ||
      msg.header.frame_id != frame_id_ || !msg.is_final ||
      msg.source_id.empty() || msg.text.empty() ||
      !lifecycle_->allowsFinalTranscript(
        msg.source_id, msg.source_generation, SourceLifecycle::Clock::now()) ||
      (msg.has_score && (!std::isfinite(msg.score) ||
       msg.score < 0.0f || msg.score > 1.0f))) return;
  transcript_pub_->publish(msg);
}
}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::AcousticSceneNode)
