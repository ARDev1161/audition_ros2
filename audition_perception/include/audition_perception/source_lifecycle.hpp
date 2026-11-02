#pragma once

#include <audition_msgs/msg/acoustic_scene.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace audition_perception {

// A canonical ROS source lifetime policy, intentionally independent of ODAS.
// LOST is reversible; ENDED is emitted exactly once on /sources and removed
// from /scene. It never converts a bearing into metric position.
class SourceLifecycle final {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct Update {
    audition_msgs::msg::AcousticSourceArray sources;
    audition_msgs::msg::AcousticScene scene;
    bool changed{false};
  };

  SourceLifecycle(std::chrono::milliseconds lost_after,
                  std::chrono::milliseconds end_after,
                  std::size_t max_sources,
                  std::chrono::milliseconds final_transcript_grace =
                    std::chrono::milliseconds{5000});

  // A full spatial-source snapshot. Even an empty snapshot is significant.
  Update observe(const audition_msgs::msg::AcousticSourceArray & spatial,
                 TimePoint arrival);

  // Called periodically even if capture/ODAS stops delivering messages.
  Update tick(TimePoint now, const std_msgs::msg::Header & header);

  // Only admitted spatial sources may own new semantic observations/events.
  bool isLive(const std::string & id, std::uint64_t generation) const;

  // ASR may finish after physical retirement. Permit a bounded final-result
  // grace for a known retired source but never across spatial stream resets.
  bool allowsFinalTranscript(const std::string & id, std::uint64_t generation,
                             TimePoint arrival) const;

private:
  using Key = std::pair<std::uint64_t, std::string>;
  struct Entry {
    audition_msgs::msg::AcousticSource source;
    TimePoint observed_at;
  };

  Update build(const std_msgs::msg::Header & header,
               const std::vector<audition_msgs::msg::AcousticSource> & ended,
               bool changed) const;
  void retireExpired(TimePoint now,
                     std::vector<audition_msgs::msg::AcousticSource> & ended,
                     bool & changed);

  std::chrono::milliseconds lost_after_;
  std::chrono::milliseconds end_after_;
  std::chrono::milliseconds final_transcript_grace_;
  std::size_t max_sources_;
  std::map<Key, Entry> tracked_;
  std::map<Key, TimePoint> recently_ended_;
  std::string frame_id_;
  void pruneEnded(TimePoint now);
  std::uint64_t stream_generation_{0};
  bool have_stream_{false};
};

}  // namespace audition_perception
