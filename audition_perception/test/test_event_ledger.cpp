#include "audition_perception/event_ledger.hpp"
#include <gtest/gtest.h>

using audition_perception::EventLedger;
using audition_msgs::msg::AcousticEvent;
namespace {
AcousticEvent event(const char * id, std::uint8_t transition) {
  AcousticEvent e;
  e.header.frame_id = "microphone_array";
  e.event_id = id;
  e.source_id = "source_a";
  e.source_generation = 3;
  e.class_label = "Speech";
  e.transition = transition;
  return e;
}
}
TEST(EventLedger, StartUpdateAndEnd) {
  EventLedger ledger(2);
  EXPECT_TRUE(ledger.apply(event("evt-1", AcousticEvent::STARTED)));
  EXPECT_FALSE(ledger.apply(event("evt-1", AcousticEvent::STARTED)));
  EXPECT_EQ(ledger.active().size(), 1U);
  EXPECT_TRUE(ledger.apply(event("evt-1", AcousticEvent::UPDATED)));
  EXPECT_TRUE(ledger.apply(event("evt-1", AcousticEvent::ENDED)));
  EXPECT_TRUE(ledger.active().empty());
  EXPECT_FALSE(ledger.apply(event("evt-1", AcousticEvent::ENDED)));
}
TEST(EventLedger, EnforcesBoundAndGeneration) {
  EventLedger ledger(1);
  EXPECT_TRUE(ledger.apply(event("1", AcousticEvent::STARTED)));
  EXPECT_FALSE(ledger.apply(event("2", AcousticEvent::STARTED)));
  auto mismatch = event("1", AcousticEvent::ENDED);
  mismatch.source_generation = 4;
  EXPECT_FALSE(ledger.apply(mismatch));
  EXPECT_EQ(ledger.active().size(), 1U);
  ledger.clear();
  EXPECT_TRUE(ledger.active().empty());
}

TEST(EventLedger, CancelEndsOnlyActiveEvent) {
  EventLedger ledger(2);
  EXPECT_TRUE(ledger.apply(event("a", AcousticEvent::STARTED)));
  EXPECT_TRUE(ledger.apply(event("a", AcousticEvent::UPDATED)));
  EXPECT_TRUE(ledger.apply(event("a", AcousticEvent::CANCELLED)));
  EXPECT_TRUE(ledger.active().empty());
  EXPECT_FALSE(ledger.apply(event("a", AcousticEvent::UPDATED)));
}
