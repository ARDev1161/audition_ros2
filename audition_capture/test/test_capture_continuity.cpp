#include "audition_capture/capture_continuity.hpp"
#include <gtest/gtest.h>

using audition_capture::CaptureContinuity;

TEST(CaptureContinuity, NormalFramesAndRecoveredXrunFenceOnlyOnce) {
  CaptureContinuity state;
  EXPECT_EQ(state.published(false), 0U);
  EXPECT_EQ(state.published(false), 1U);
  EXPECT_EQ(state.published(true), 3U);
  EXPECT_EQ(state.published(false), 4U);
  auto snapshot = state.snapshot();
  EXPECT_EQ(snapshot.published_frames, 4U);
  EXPECT_EQ(snapshot.xrun_recoveries, 1U);
  EXPECT_EQ(snapshot.last_sequence_number, 4U);
  EXPECT_TRUE(snapshot.connected);
}

TEST(CaptureContinuity, ReopenCreatesGapWithoutInventingPCM) {
  CaptureContinuity state;
  EXPECT_EQ(state.published(false), 0U);
  state.readerFailed("ENODEV");
  EXPECT_FALSE(state.snapshot().connected);
  EXPECT_EQ(state.snapshot().published_frames, 1U);
  state.reconnectAttempt();
  state.reconnected();
  EXPECT_EQ(state.published(false), 2U);
  EXPECT_EQ(state.published(false), 3U);
  EXPECT_EQ(state.snapshot().reconnect_successes, 1U);
  EXPECT_EQ(state.snapshot().fatal_errors, 1U);
}

TEST(CaptureContinuity, FailureBeforeFirstFrameDoesNotFabricateGap) {
  CaptureContinuity state;
  state.readerFailed("stalled", true);
  state.reconnectAttempt();
  state.reconnected();
  EXPECT_EQ(state.published(false), 0U);
  EXPECT_EQ(state.snapshot().stall_reconnects, 1U);
}

TEST(CaptureContinuity, BoundedExponentialBackoffAndRetryLimit) {
  CaptureContinuity state(8);
  state.readerFailed("unplugged");
  for (int i = 0; i < 8; ++i) {
    ASSERT_TRUE(state.canRetry());
    state.reconnectAttempt();
    state.reconnectFailed("still missing");
    const auto expected = (i < 5) ? (100U << i) : 2000U;
    EXPECT_EQ(state.snapshot().next_retry_delay_ms, expected);
  }
  EXPECT_FALSE(state.canRetry());
  EXPECT_TRUE(state.snapshot().terminal_failure);
  EXPECT_EQ(state.snapshot().reconnect_attempts, 8U);
  EXPECT_EQ(state.snapshot().reconnect_failures, 8U);
}

TEST(CaptureContinuity, SuccessfulReopenClearsBackoffAndAllowsLaterRecovery) {
  CaptureContinuity state(2);
  state.readerFailed("lost");
  state.reconnectAttempt();
  state.reconnectFailed("missing");
  EXPECT_EQ(state.snapshot().consecutive_failures, 1U);
  state.reconnectAttempt();
  state.reconnected();
  EXPECT_TRUE(state.canRetry());
  EXPECT_TRUE(state.snapshot().connected);
  EXPECT_EQ(state.snapshot().consecutive_failures, 0U);
  EXPECT_EQ(state.snapshot().next_retry_delay_ms, 0U);
  EXPECT_TRUE(state.snapshot().last_error.empty());
  state.readerFailed("again");
  EXPECT_TRUE(state.canRetry());
}
