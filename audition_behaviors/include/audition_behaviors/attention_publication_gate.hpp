#pragma once

namespace audition_behaviors {

// Edge-triggered timer publication state. Scene callbacks always publish;
// the timer emits only transitions, including recovery from an invalid scene
// advisory to a still-valid last accepted target.
class AttentionPublicationGate {
public:
  void on_scene_published(bool valid) {
    have_publication_ = true;
    last_valid_ = valid;
  }

  bool should_publish_timer(bool valid) const {
    return !have_publication_ || valid != last_valid_;
  }

  void on_timer_published(bool valid) {
    on_scene_published(valid);
  }

private:
  bool have_publication_ = false;
  bool last_valid_ = false;
};

}  // namespace audition_behaviors
