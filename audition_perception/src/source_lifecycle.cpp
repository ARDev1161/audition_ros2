#include "audition_perception/source_lifecycle.hpp"

#include <stdexcept>
#include <utility>

namespace audition_perception {

SourceLifecycle::SourceLifecycle(
  std::chrono::milliseconds lost_after,
  std::chrono::milliseconds end_after,
  std::size_t max_sources)
: lost_after_(lost_after), end_after_(end_after), max_sources_(max_sources)
{
  if (lost_after.count() < 0 || end_after <= lost_after ||
      end_after.count() > 60000 || max_sources == 0 || max_sources > 128) {
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
  if (spatial.header.frame_id.empty() || spatial.sources.size() > max_sources_) {
    throw std::invalid_argument("Invalid frame or excessive source snapshot");
  }
  std::set<Key> keys;
  for (const auto & source : spatial.sources) {
    const Key key{source.source_generation, source.source_id};
    if (source.source_id.empty() ||
        source.state != audition_msgs::msg::AcousticSource::ACTIVE ||
        !keys.insert(key).second) {
      throw std::invalid_argument("Invalid or duplicated active spatial source");
    }
  }

  std::vector<audition_msgs::msg::AcousticSource> ended;
  if (have_stream_ && stream_generation_ != spatial.generation) {
    for (const auto & pair : tracked_) {
      auto source = pair.second.source;
      source.state = audition_msgs::msg::AcousticSource::ENDED;
      ended.push_back(std::move(source));
    }
    tracked_.clear();
  }
  stream_generation_ = spatial.generation;
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
  return build(spatial.header, ended, changed);
}

SourceLifecycle::Update SourceLifecycle::tick(
  TimePoint now, const std_msgs::msg::Header & header)
{
  std::vector<audition_msgs::msg::AcousticSource> ended;
  bool changed = false;
  retireExpired(now, ended, changed);
  return build(header, ended, changed);
}

}  // namespace audition_perception
