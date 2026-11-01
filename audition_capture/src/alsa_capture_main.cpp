#include "audition_capture/alsa_pcm_reader.hpp"

#include <audition_msgs/msg/audio_frame.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace audition_capture {

class AlsaCaptureNode final : public rclcpp::Node {
public:
  AlsaCaptureNode() : Node("audio_capture") {
    const auto device = declare_parameter<std::string>("device", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "microphone_array");
    const int rate = declare_parameter<int>("sample_rate_hz", 16000);
    const int channels = declare_parameter<int>("channel_count", 6);
    const int chunk = declare_parameter<int>("frames_per_message", 160);
    const int timeout = declare_parameter<int>("wait_timeout_ms", 100);
    if (frame_id_.empty() || rate < 8000 || rate > 192000 ||
        channels <= 0 || channels > 32 || chunk <= 0 || chunk > 8192 ||
        timeout <= 0 || timeout > 1000) {
      throw std::invalid_argument("Invalid ALSA capture node parameters");
    }
    reader_ = std::make_unique<AlsaPcmReader>(
      device, static_cast<std::uint32_t>(rate),
      static_cast<std::uint32_t>(channels),
      static_cast<std::uint32_t>(chunk), timeout);

    publisher_ = create_publisher<audition_msgs::msg::AudioFrame>(
      "audio/raw", rclcpp::SensorDataQoS());
    worker_ = std::thread([this]() { run(); });
    RCLCPP_INFO(get_logger(), "ALSA capture: %s at %d Hz, %d channels",
                device.c_str(), rate, channels);
  }

  ~AlsaCaptureNode() override {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
  }

private:
  void run() {
    using Clock = std::chrono::steady_clock;
    const auto nominal_ns = std::chrono::nanoseconds{
      1000000000LL * reader_->chunkFrames() / reader_->sampleRate()};
    auto next_emit = Clock::now();
    std::vector<float> pcm;
    while (running_.load() && rclcpp::ok()) {
      std::uint32_t frames = 0;
      bool recovered = false;
      try {
        if (!reader_->read(pcm, frames, recovered)) continue;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "ALSA capture stopped: %s", e.what());
        break;
      }
      if (recovered) {
        // Strictly advance a sequence number without publishing audio:
        // spatial processing detects the gap and resets its ODAS generation.
        ++sequence_;
        RCLCPP_WARN(get_logger(),
          "ALSA discontinuity after recovery #%llu",
          static_cast<unsigned long long>(reader_->recoveries()));
      }
      audition_msgs::msg::AudioFrame msg;
      msg.header.frame_id = frame_id_;
      // Estimated time of first PCM frame, not a hardware capture timestamp:
      // this uses the current ROS clock at completion of snd_pcm_readi.
      const auto delay = static_cast<std::int64_t>(frames) *
                         1000000000LL / reader_->sampleRate();
      msg.header.stamp = now() - rclcpp::Duration::from_nanoseconds(delay);
      msg.sequence_number = sequence_++;
      msg.sample_rate_hz = reader_->sampleRate();
      msg.channel_count = reader_->channels();
      msg.frame_count = frames;
      msg.layout = audition_msgs::msg::AudioFrame::INTERLEAVED;
      for (std::uint32_t i = 0; i < msg.channel_count; ++i) {
        msg.channel_indices.push_back(i);
      }
      msg.pcm = std::move(pcm);
      publisher_->publish(std::move(msg));

      // A virtual ALSA "null" device produces data infinitely fast. Do not
      // flood DDS; physical capture normally keeps us at this wall-clock rate.
      next_emit += std::chrono::duration_cast<Clock::duration>(
        nominal_ns * frames / reader_->chunkFrames());
      const auto current = Clock::now();
      if (next_emit > current) {
        std::this_thread::sleep_until(next_emit);
      } else if (current - next_emit > std::chrono::milliseconds(250)) {
        next_emit = current;
      }
    }
  }

  std::string frame_id_;
  std::unique_ptr<AlsaPcmReader> reader_;
  rclcpp::Publisher<audition_msgs::msg::AudioFrame>::SharedPtr publisher_;
  std::atomic<bool> running_{true};
  std::thread worker_;
  std::uint64_t sequence_{0};
};

}  // namespace audition_capture

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<audition_capture::AlsaCaptureNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("alsa_capture"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
