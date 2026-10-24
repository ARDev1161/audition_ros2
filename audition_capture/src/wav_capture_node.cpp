#include "audition_capture/wav_capture_node.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace audition_capture {

WavCaptureNode::WavCaptureNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("audio_capture", options)
{
  const auto path = declare_parameter<std::string>("wav_path", "");
  frame_id_ = declare_parameter<std::string>("frame_id", "microphone_array");
  const auto size = declare_parameter<int>("frames_per_message", 160);
  loop_ = declare_parameter<bool>("loop", false);

  if (path.empty() || frame_id_.empty() || size <= 0 || size > 192000) {
    throw std::invalid_argument("wav_path, frame_id and frames_per_message must be valid");
  }
  frames_per_message_ = static_cast<std::uint32_t>(size);
  reader_ = std::make_unique<WavReader>(path);
  publisher_ = create_publisher<audition_msgs::msg::AudioFrame>(
    "audio/raw", rclcpp::SensorDataQoS());

  const auto period_ns = std::max<std::int64_t>(1,
    (1000000000LL * frames_per_message_) / reader_->sampleRate());
  timer_ = create_wall_timer(
    std::chrono::nanoseconds(period_ns),
    [this]() { publishNext(); });

  RCLCPP_INFO(get_logger(), "WAV replay: %u Hz / %u channels / %llu frames",
    reader_->sampleRate(), reader_->channels(),
    static_cast<unsigned long long>(reader_->totalFrames()));
}

void WavCaptureNode::publishNext() {
  try {
    auto pcm = reader_->readFrames(frames_per_message_);
    if (pcm.empty() && loop_ && reader_->totalFrames() > 0) {
      reader_->rewind();
      pcm = reader_->readFrames(frames_per_message_);
    }
    if (pcm.empty()) {
      RCLCPP_INFO(get_logger(), "WAV replay finished");
      timer_->cancel();
      return;
    }
    audition_msgs::msg::AudioFrame message;
    // ROS replay acquisition clock: first frame time at this playback callback.
    // Original recording clock metadata is not available in generic RIFF WAV.
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.sequence_number = sequence_++;
    message.sample_rate_hz = reader_->sampleRate();
    message.channel_count = reader_->channels();
    message.frame_count = static_cast<std::uint32_t>(pcm.size() / message.channel_count);
    message.layout = audition_msgs::msg::AudioFrame::INTERLEAVED;
    for (std::uint32_t i = 0; i < message.channel_count; ++i) {
      message.channel_indices.push_back(i);
    }
    message.pcm = std::move(pcm);
    publisher_->publish(std::move(message));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Stopping corrupt WAV replay: %s", error.what());
    timer_->cancel();
  }
}

}  // namespace audition_capture
RCLCPP_COMPONENTS_REGISTER_NODE(audition_capture::WavCaptureNode)
