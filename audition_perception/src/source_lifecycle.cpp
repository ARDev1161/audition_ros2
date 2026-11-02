#include "audition_perception/source_lifecycle.hpp"

#include <stdexcept>
#include <algorithm>
#include <utility>

namespace audition_perception {

SourceLifecycle::SourceLifecycle(
  std::chrono::milliseconds lost_after,
  std::chrono::milliseconds end_after,
  std::size_t max_sources,
  std::chrono::milliseconds final_transcript_grace)
: lost_after_(lost_after), end_after_(end_after),
  final_transcript_grace_(final_transcript_grace), max_sources_(max_sources)
{
  if (lost_after.count() < 0 || end_after <= lost_after ||
      end_after.count() > 60000 || max_sources == 0 || max_sources > 128 ||
      final_transcript_grace.count() < 0 || final_transcript_grace.count() > 30000) {
    throw std::invalid_argument("Invalid source lifecycle limits");
  }
}

SourceLifecycle::Update SourceLifecycle::build(
  const std_msgs::msg::Header & header,
  const std::vector<audition_msgs::msg::AcousticSource> & ended,
  bool changed) const
{
  Update update;
  update.changed = changed;
  update.sources.header = header;
  update.sources.generation = stream_generation_;
  update.sources.spatial_session_id = spatial_sessions_.current();
  update.scene.header = header;
  update.scene.generation = stream_generation_;
  update.sources.sources.reserve(tracked_.size() + ended.size());
  update.scene.sources.reserve(tracked_.size());

  for (const auto & element : tracked_) {
    update.sources.sources.push_back(element.second.source);
    update.scene.sources.push_back(element.second.source);
  }
  for (const auto & source : ended) update.sources.sources.push_back(source);
  return update;
}

bool SourceLifecycle::isLive(const std::string & id, std::uint64_t gen) const {
  if (!have_stream_ || gen != stream_generation_) return false;
  return tracked_.find(Key{gen, id}) != tracked_.end();
}

bool SourceLifecycle::allowsFinalTranscript(
  const std::string & id, std::uint64_t gen, TimePoint arrival) const
{
  if (isLive(id, gen)) return true;
  if (!have_stream_ || gen != stream_generation_) return false;
  const auto it = recently_ended_.find(Key{gen, id});
  return it != recently_ended_.end() && arrival >= it->second &&
         arrival - it->second <= final_transcript_grace_;
}

void SourceLifecycle::pruneEnded(TimePoint now) {
  for (auto it = recently_ended_.begin(); it != recently_ended_.end();) {
    if (now >= it->second && now - it->second > final_transcript_grace_) {
      it = recently_ended_.erase(it);
    } else {
      ++it;
    }
  }
  // Even pathological rapid source churn cannot grow an unbounded tombstone map.
  while (recently_ended_.size() > max_sources_ * 4U) {
    recently_ended_.erase(recently_ended_.begin());
  }
}

void SourceLifecycle::retireExpired(
  TimePoint now,
  std::vector<audition_msgs::msg::AcousticSource> & ended,
  bool & changed)
{
  for (auto it = tracked_.begin(); it != tracked_.end();) {
    const auto elapsed = now - it->second.observed_at;
    if (elapsed >= end_after_) {
      auto source = it->second.source;
      source.state = audition_msgs::msg::AcousticSource::ENDED;
      recently_ended_[it->first] = now;
      ended.push_back(std::move(source));
      it = tracked_.erase(it);
      changed = true;
    } else {
      if (elapsed >= lost_after_ &&
          it->second.source.state == audition_msgs::msg::AcousticSource::ACTIVE) {
        it->second.source.state = audition_msgs::msg::AcousticSource::LOST;
        changed = true;
      }
      ++it;
    }
  }
}

SourceLifecycle::Update SourceLifecycle::observe(
  const audition_msgs::msg::AcousticSourceArray & spatial, TimePoint arrival)
{
  const auto decision = spatial_sessions_.inspect(spatial.spatial_session_id);
  const bool new_session = decision == CaptureSessionGate::Decision::Changed;
  if (spatial.header.frame_id.empty() || spatial.generation == 0 ||
      spatial.sequence_number == 0 ||
      (decision == CaptureSessionGate::Decision::Same &&
       spatial.sequence_number <= last_spatial_sequence_) ||
      spatial.sources.size() > max_sources_ ||
      (have_stream_ && !new_session && spatial.generation < stream_generation_) ||
      (have_stream_ && !new_session && spatial.generation == stream_generation_ &&
       spatial.header.frame_id != frame_id_)) {
    throw std::invalid_argument("Invalid frame or excessive source snapshot");
  }
  std::set<Key> keys;
  for (const auto & source : spatial.sources) {
    const Key key{source.source_generation, source.source_id};
    if (source.source_id.empty() ||
        source.source_generation != spatial.generation ||
        source.header.frame_id != spatial.header.frame_id ||
        source.state != audition_msgs::msg::AcousticSource::ACTIVE ||
        !keys.insert(key).second) {
      throw std::invalid_argument("Invalid or duplicated active spatial source");
    }
  }

  std::vector<audition_msgs::msg::AcousticSource> ended;
  if (have_stream_ && (new_session || stream_generation_ != spatial.generation)) {
    // A new producer invalidates old IDs. Do not mix old ENDED records into
    // the new scene session; the scene cancels outstanding events separately.
    if (!new_session) {
      for (const auto & pair : tracked_) {
        auto source = pair.second.source;
        source.state = audition_msgs::msg::AcousticSource::ENDED;
        ended.push_back(std::move(source));
      }
    }
    tracked_.clear();
    recently_ended_.clear();
  }
  spatial_sessions_.commit(spatial.spatial_session_id);
  last_spatial_sequence_ = spatial.sequence_number;
  stream_generation_ = spatial.generation;
  frame_id_ = spatial.header.frame_id;
  have_stream_ = true;

  for (auto & pair : tracked_) {
    if (!keys.count(pair.first)) {
      pair.second.source.state = audition_msgs::msg::AcousticSource::LOST;
    }
  }
  for (const auto & source : spatial.sources) {
    const Key key{source.source_generation, source.source_id};
    tracked_[key] = Entry{source, arrival};
  }
  bool changed = true;  // A new snapshot always updates its observation time.
  retireExpired(arrival, ended, changed);
  pruneEnded(arrival);
  return build(spatial.header, ended, changed);
}

SourceLifecycle::Update SourceLifecycle::tick(
  TimePoint now, const std_msgs::msg::Header & header)
{
  std::vector<audition_msgs::msg::AcousticSource> ended;
  bool changed = false;
  retireExpired(now, ended, changed);
  pruneEnded(now);
  return build(header, ended, changed);
}

}  // namespace audition_perception
