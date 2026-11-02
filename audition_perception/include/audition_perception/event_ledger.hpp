#pragma once
#include <audition_msgs/msg/acoustic_event.hpp>
#include <map>
#include <string>
#include <vector>

namespace audition_perception {
// Canonical *publication state* only. libaudition owns event detection.
class EventLedger {
public:
  explicit EventLedger(std::size_t capacity = 128);
  // Valid transitions alone modify the active snapshot.
  bool apply(const audition_msgs::msg::AcousticEvent & event);
  void clear() noexcept { active_.clear(); }
  std::vector<audition_msgs::msg::AcousticEvent> active() const;
private:
  std::size_t capacity_;
  std::map<std::string, audition_msgs::msg::AcousticEvent> active_;
};
}  // namespace audition_perception
