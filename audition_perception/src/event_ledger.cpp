#include "audition_perception/event_ledger.hpp"
#include <stdexcept>

namespace audition_perception {
EventLedger::EventLedger(std::size_t capacity) : capacity_(capacity) {
  if (capacity_ == 0 || capacity_ > 4096) {
    throw std::invalid_argument("Invalid maximum number of acoustic events");
  }
}
bool EventLedger::apply(const audition_msgs::msg::AcousticEvent & event) {
  using Msg = audition_msgs::msg::AcousticEvent;
  if (event.event_id.empty() || event.source_id.empty() ||
      event.class_label.empty() || event.header.frame_id.empty()) return false;
  auto it = active_.find(event.event_id);
  if (event.transition == Msg::STARTED) {
    if (it != active_.end() || active_.size() >= capacity_) return false;
    active_.emplace(event.event_id, event);
    return true;
  }
  if (it == active_.end() || it->second.source_id != event.source_id ||
      it->second.source_generation != event.source_generation ||
      it->second.class_label != event.class_label) return false;
  if (event.transition == Msg::UPDATED) {
    it->second = event;
    return true;
  }
  if (event.transition == Msg::ENDED || event.transition == Msg::CANCELLED) {
    active_.erase(it);
    return true;
  }
  return false;
}
std::vector<audition_msgs::msg::AcousticEvent> EventLedger::active() const {
  std::vector<audition_msgs::msg::AcousticEvent> events;
  events.reserve(active_.size());
  for (const auto & entry : active_) events.push_back(entry.second);
  return events;
}
}  // namespace audition_perception
