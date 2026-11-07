#include "audition_capture/alsa_pcm_reader.hpp"
#include "audition_capture/capture_continuity.hpp"
#include "audition_capture/capture_session_id.hpp"

#include <audition_msgs/msg/audio_frame.hpp>
#include <audition_msgs/srv/get_capture_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace audition_capture {

class AlsaCaptureNode final : public rclcpp::Node {
public:
  AlsaCaptureNode() : Node("audio_capture") {
    device_ = declare_parameter<std::string>("device", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "microphone_array");
    const int rate = declare_parameter<int>("sample_rate_hz", 16000);
    const int channels = declare_parameter<int>("channel_count", 6);
    const int chunk = declare_parameter<int>("frames_per_message", 160);
    timeout_ms_ = declare_parameter<int>("wait_timeout_ms", 100);
    stall_timeout_ms_ = declare_parameter<int>("stall_timeout_ms", 2000);
    const int max_attempts = declare_parameter<int>("max_reconnect_attempts", 0);
    if (frame_id_.empty() || rate < 8000 || rate > 192000 ||
        channels <= 0 || channels > 32 || chunk <= 0 || chunk > 8192 ||
        timeout_ms_ <= 0 || timeout_ms_ > 1000 ||
        stall_timeout_ms_ < 500 || stall_timeout_ms_ > 60000 ||
        max_attempts < 0 || max_attempts > 10000) {
      throw std::invalid_argument("Invalid ALSA capture/reconnect parameters");
    }
    rate_ = static_cast<std::uint32_t>(rate);
    channels_ = static_cast<std::uint32_t>(channels);
    chunk_ = static_cast<std::uint32_t>(chunk);
    continuity_ = CaptureContinuity(static_cast<std::uint32_t>(max_attempts));
    // Initial configuration is fail-fast; reconnect only after a running
    // device fails. Never silently substitute a different ALSA device/rate.
    reader_ = openReader();
    publisher_ = create_publisher<audition_msgs::msg::AudioFrame>(
      "audio/raw", rclcpp::SensorDataQoS());
    status_srv_ = create_service<audition_msgs::srv::GetCaptureStatus>(
      "audio_capture/get_capture_status",
      [this](const std::shared_ptr<audition_msgs::srv::GetCaptureStatus::Request>,
             std::shared_ptr<audition_msgs::srv::GetCaptureStatus::Response> response) {
        CaptureContinuity::Snapshot state;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          state = continuity_.snapshot();
        }
        response->connected = state.connected;
        response->terminal_failure = state.terminal_failure;
        response->published_frames = state.published_frames;
        response->last_sequence_number = state.last_sequence_number;
        response->xrun_recoveries = state.xrun_recoveries;
        response->fatal_errors = state.fatal_errors;
        response->stall_reconnects = state.stall_reconnects;
        response->reconnect_attempts = state.reconnect_attempts;
        response->reconnect_successes = state.reconnect_successes;
        response->reconnect_failures = state.reconnect_failures;
        response->consecutive_failures = state.consecutive_failures;
        response->next_retry_delay_ms = state.next_retry_delay_ms;
        response->last_error = state.last_error;
        response->note = "Capture estimates timestamps at read completion, not hardware time; "
          "XRUN count is recovered episodes delivered with PCM. Sequence gaps fence "
          "downstream ODAS/mono generations; reconnect does not certify microphone geometry.";
      });
    worker_ = std::thread([this]() { run(); });
    RCLCPP_INFO(get_logger(), "ALSA capture: %s at %d Hz, %d channels",
                device_.c_str(), rate, channels);
  }

  ~AlsaCaptureNode() override {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
  }

private:
  using Clock = std::chrono::steady_clock;

  std::unique_ptr<AlsaPcmReader> openReader() const {
    return std::make_unique<AlsaPcmReader>(
      device_, rate_, channels_, chunk_, timeout_ms_);
  }

  void sleepInterruptibly(std::chrono::milliseconds duration) {
    while (duration.count() > 0 && running_.load() && rclcpp::ok()) {
      const auto slice = std::min(duration, std::chrono::milliseconds(50));
      std::this_thread::sleep_for(slice);
      duration -= slice;
    }
  }

  void reportFailure(const std::string & reason, bool stalled) {
    reader_.reset();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      continuity_.readerFailed(reason, stalled);
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "ALSA capture disconnected: %s; attempting bounded-backoff reopen",
      reason.c_str());
  }

  bool tryReconnect() {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!continuity_.canRetry()) return false;
      continuity_.reconnectAttempt();
    }
    try {
      reader_ = openReader();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        continuity_.reconnected();
      }
      RCLCPP_INFO(get_logger(), "ALSA capture reconnected to %s", device_.c_str());
      return true;
    } catch (const std::exception & e) {
      std::uint32_t delay;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        continuity_.reconnectFailed(e.what());
        delay = continuity_.snapshot().next_retry_delay_ms;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "ALSA reopen failed: %s (retry in %u ms)", e.what(), delay);
      sleepInterruptibly(std::chrono::milliseconds(delay));
      return false;
    }
  }

  void run() {
    const auto nominal_ns = std::chrono::nanoseconds{
      1000000000LL * chunk_ / rate_};
    auto next_emit = Clock::now();
    auto last_frame = Clock::now();
    std::vector<float> pcm;
    while (running_.load() && rclcpp::ok()) {
      if (!reader_) {
        bool retry_allowed;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          retry_allowed = continuity_.canRetry();
        }
        if (!retry_allowed) {
          RCLCPP_ERROR(get_logger(), "ALSA reconnect attempt limit exhausted");
          break;
        }
        if (!tryReconnect()) continue;
        last_frame = Clock::now();
        next_emit = last_frame;
      }
      std::uint32_t frames = 0;
      bool recovered = false;
      try {
        if (!reader_->read(pcm, frames, recovered)) {
          if (Clock::now() - last_frame > std::chrono::milliseconds(stall_timeout_ms_)) {
            reportFailure("no capture PCM before stall_timeout_ms", true);
          }
          continue;
        }
      } catch (const std::exception & e) {
        reportFailure(e.what(), false);
        continue;
      }
      last_frame = Clock::now();
      audition_msgs::msg::AudioFrame msg;
      msg.header.frame_id = frame_id_;
      msg.capture_session_id = capture_session_id_;
      // Estimate the first sample's ROS timestamp from read completion.
      const auto delay = static_cast<std::int64_t>(frames) * 1000000000LL / rate_;
      msg.header.stamp = now() - rclcpp::Duration::from_nanoseconds(delay);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        msg.sequence_number = continuity_.published(recovered);
      }
      if (recovered) {
        RCLCPP_WARN(get_logger(), "ALSA XRUN/suspend recovery; source epoch fenced");
      }
      msg.sample_rate_hz = rate_;
      msg.channel_count = channels_;
      msg.frame_count = frames;
      msg.layout = audition_msgs::msg::AudioFrame::INTERLEAVED;
      for (std::uint32_t i = 0; i < channels_; ++i) msg.channel_indices.push_back(i);
      msg.pcm = std::move(pcm);
      publisher_->publish(std::move(msg));

      // Virtual ALSA null produces frames faster than real time.
      next_emit += std::chrono::duration_cast<Clock::duration>(
        nominal_ns * frames / chunk_);
      const auto current = Clock::now();
      if (next_emit > current) {
        sleepInterruptibly(std::chrono::duration_cast<std::chrono::milliseconds>(
          next_emit - current));
      } else if (current - next_emit > std::chrono::milliseconds(250)) {
        next_emit = current;
      }
    }
  }

  std::string device_, frame_id_;
  const std::string capture_session_id_{makeCaptureSessionId()};
  std::uint32_t rate_{0}, channels_{0}, chunk_{0};
  int timeout_ms_{100}, stall_timeout_ms_{2000};
  std::unique_ptr<AlsaPcmReader> reader_;  // owned exclusively by capture worker after start
  rclcpp::Publisher<audition_msgs::msg::AudioFrame>::SharedPtr publisher_;
  rclcpp::Service<audition_msgs::srv::GetCaptureStatus>::SharedPtr status_srv_;
  std::mutex state_mutex_;
  CaptureContinuity continuity_;
  std::atomic<bool> running_{true};
  std::thread worker_;
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
