#include "audition_behaviors/attention_publication_gate.hpp"

#include <gtest/gtest.h>

using audition_behaviors::AttentionPublicationGate;

TEST(AttentionPublicationGate, StartupPublishesInvalidOnce) {
  AttentionPublicationGate gate;
  EXPECT_TRUE(gate.should_publish_timer(false));
  gate.on_timer_published(false);
  EXPECT_FALSE(gate.should_publish_timer(false));
}

TEST(AttentionPublicationGate, TimeoutPublishesInvalidTransitionOnlyOnce) {
  AttentionPublicationGate gate;
  gate.on_scene_published(true);
  EXPECT_FALSE(gate.should_publish_timer(true));
  EXPECT_TRUE(gate.should_publish_timer(false));
  gate.on_timer_published(false);
  EXPECT_FALSE(gate.should_publish_timer(false));
}

TEST(AttentionPublicationGate, RejectedSceneRecoversPreviousValidAdvisory) {
  AttentionPublicationGate gate;
  gate.on_scene_published(true);
  // Policy rejects a replay, but preserves the previously accepted target.
  gate.on_scene_published(false);
  EXPECT_TRUE(gate.should_publish_timer(true));
  gate.on_timer_published(true);
  EXPECT_FALSE(gate.should_publish_timer(true));
}

TEST(AttentionPublicationGate, SceneCallbackOverridesPendingTimerState) {
  AttentionPublicationGate gate;
  gate.on_scene_published(false);
  EXPECT_TRUE(gate.should_publish_timer(true));
  // A new accepted scene has already published the valid target.
  gate.on_scene_published(true);
  EXPECT_FALSE(gate.should_publish_timer(true));
}
