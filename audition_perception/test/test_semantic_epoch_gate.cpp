#include "audition_perception/semantic_epoch_gate.hpp"

#include <gtest/gtest.h>
#include <audition_msgs/msg/acoustic_source.hpp>
#include <audition_msgs/msg/acoustic_source_array.hpp>
#include <audition_msgs/msg/source_audio_frame.hpp>
#include <audition_msgs/srv/get_semantic_status.hpp>

using audition_perception::SemanticEpochGate;
using Source = audition_msgs::msg::AcousticSource;
using Snapshot = audition_msgs::msg::AcousticSourceArray;
using Audio = audition_msgs::msg::SourceAudioFrame;

static Source source(const char * id, std::uint64_t generation,
                     const char * frame, std::uint8_t state = Source::ACTIVE) {
  Source out;
  out.source_id = id;
  out.source_generation = generation;
  out.header.frame_id = frame;
  out.state = state;
  return out;
}
static Snapshot snapshot(std::uint64_t generation, const char * frame) {
  Snapshot out;
  out.generation = generation;
  static std::uint64_t sequence = 0;
  out.sequence_number = ++sequence;
  out.header.frame_id = frame;
  return out;
}
static Audio audio(const char * id, std::uint64_t generation,
                   const char * frame) {
  Audio out;
  out.source_id = id;
  out.source_generation = generation;
  out.header.frame_id = frame;
  return out;
}
static audition_msgs::srv::GetSemanticStatus::Response status(
  const SemanticEpochGate & gate) {
  audition_msgs::srv::GetSemanticStatus::Response out;
  gate.status(out);
  return out;
}

TEST(SemanticEpochGate, RequiresCanonicalAdmissionAndRejectsOrphanAudio) {
  SemanticEpochGate gate;
  EXPECT_FALSE(gate.accept(audio("s", 1, "mic")));
  auto first = snapshot(1, "mic");
  first.sources.push_back(source("s", 1, "mic"));
  EXPECT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_TRUE(gate.accept(audio("s", 1, "mic")));
  EXPECT_FALSE(gate.accept(audio("other", 1, "mic")));
  EXPECT_FALSE(gate.accept(audio("s", 0, "mic")));
  EXPECT_FALSE(gate.accept(audio("s", 1, "other_mic")));
  const auto counters = status(gate);
  EXPECT_TRUE(counters.have_stream);
  EXPECT_EQ(counters.source_generation, 1U);
  EXPECT_EQ(counters.admitted_sources, 1U);
  EXPECT_EQ(counters.accepted_audio_frames, 1U);
  EXPECT_EQ(counters.dropped_audio_frames, 4U);
}

TEST(SemanticEpochGate, MalformedSnapshotDoesNotChangeAdmissionOrGeneration) {
  SemanticEpochGate gate;
  auto first = snapshot(4, "mic");
  first.sources.push_back(source("s", 4, "mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  auto bad = snapshot(5, "new_mic");
  bad.sources.push_back(source("s", 4, "mic", Source::ACTIVE));
  EXPECT_EQ(gate.observe(bad), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_TRUE(gate.accept(audio("s", 4, "mic")));
  auto duplicate = first;
  duplicate.sources.push_back(first.sources.front());
  EXPECT_EQ(gate.observe(duplicate), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_TRUE(gate.accept(audio("s", 4, "mic")));
  EXPECT_EQ(status(gate).stream_resets, 0U);
  EXPECT_EQ(status(gate).rejected_source_snapshots, 2U);
}

TEST(SemanticEpochGate, EpochChangeAcceptsOldEndedAndFencesOldPCM) {
  SemanticEpochGate gate;
  auto first = snapshot(8, "old_mic");
  first.sources.push_back(source("s", 8, "old_mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  auto next = snapshot(9, "new_mic");
  next.sources.push_back(source("s", 8, "old_mic", Source::ENDED));
  next.sources.push_back(source("s", 9, "new_mic"));
  EXPECT_EQ(gate.observe(next), SemanticEpochGate::SnapshotResult::NewGeneration);
  EXPECT_FALSE(gate.accept(audio("s", 8, "old_mic")));
  EXPECT_TRUE(gate.accept(audio("s", 9, "new_mic")));
  EXPECT_EQ(status(gate).stream_resets, 1U);
  auto empty = snapshot(9, "new_mic");
  EXPECT_EQ(gate.observe(empty), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_FALSE(gate.accept(audio("s", 9, "new_mic")));
  EXPECT_EQ(status(gate).admitted_sources, 0U);
  EXPECT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Rejected);
}

TEST(SemanticEpochGate, EndedAndLostSourcesAreHandledSeparately) {
  SemanticEpochGate gate;
  auto first = snapshot(2, "mic");
  first.sources.push_back(source("live", 2, "mic", Source::LOST));
  first.sources.push_back(source("retired", 2, "mic", Source::ENDED));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_TRUE(gate.accept(audio("live", 2, "mic")));
  EXPECT_FALSE(gate.accept(audio("retired", 2, "mic")));
  EXPECT_EQ(status(gate).admitted_sources, 1U);
}


TEST(SemanticEpochGate, ReloadRejectsQueuedSnapshotsAndAudioUntilNewAdmission) {
  SemanticEpochGate gate;
  auto first = snapshot(12, "mic");
  first.header.stamp.sec = 100;
  first.sources.push_back(source("s", 12, "mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  auto old_pcm = audio("s", 12, "mic");
  old_pcm.header.stamp.sec = 100;
  EXPECT_TRUE(gate.accept(old_pcm));

  gate.beginReload();
  EXPECT_TRUE(gate.reloadPending());
  EXPECT_EQ(status(gate).admitted_sources, 0U);
  EXPECT_FALSE(gate.accept(old_pcm));
  EXPECT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_TRUE(gate.reloadPending());

  auto fresh = first;
  fresh.header.stamp.sec = 101;
  ++fresh.sequence_number;
  EXPECT_EQ(gate.observe(fresh), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_FALSE(gate.reloadPending());
  EXPECT_FALSE(gate.accept(old_pcm));
  old_pcm.header.stamp.sec = 101;
  EXPECT_FALSE(gate.accept(old_pcm));  // Snapshot-time PCM may be queued.
  old_pcm.header.stamp.sec = 102;
  EXPECT_TRUE(gate.accept(old_pcm));
  EXPECT_EQ(status(gate).source_generation, 12U);  // Never fake upstream epoch.
}

TEST(SemanticEpochGate, ReloadAcceptsRealUpstreamGenerationChange) {
  SemanticEpochGate gate;
  auto first = snapshot(5, "old");
  first.header.stamp.sec = 20;
  first.sources.push_back(source("s", 5, "old"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  gate.beginReload();
  auto next = snapshot(6, "new");
  next.header.stamp.sec = 20;
  next.sources.push_back(source("s", 6, "new"));
  EXPECT_EQ(gate.observe(next), SemanticEpochGate::SnapshotResult::NewGeneration);
  EXPECT_FALSE(gate.reloadPending());
  EXPECT_TRUE(gate.accept(audio("s", 6, "new")));
  EXPECT_FALSE(gate.accept(audio("s", 5, "old")));
}

TEST(SemanticEpochGate, RejectsReorderedAndDuplicatedCanonicalSnapshots) {
  SemanticEpochGate gate;
  auto first = snapshot(40, "mic");
  first.header.stamp.sec = 100;
  first.sources.push_back(source("live", 40, "mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  const auto accepted = status(gate).last_source_sequence;

  auto retirement = snapshot(40, "mic");
  retirement.header.stamp.sec = 99;  // Capture stamps may regress vs timers.
  retirement.sources.push_back(source("live", 40, "mic", Source::ENDED));
  ASSERT_EQ(gate.observe(retirement), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_FALSE(gate.accept(audio("live", 40, "mic")));

  // A replayed ACTIVE snapshot cannot resurrect the retired source.
  EXPECT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_EQ(gate.observe(retirement), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_FALSE(gate.accept(audio("live", 40, "mic")));
  EXPECT_EQ(status(gate).last_source_sequence, retirement.sequence_number);
  EXPECT_GT(status(gate).last_source_sequence, accepted);
  EXPECT_EQ(status(gate).rejected_source_snapshots, 2U);
}

TEST(SemanticEpochGate, RejectsZeroAndOldSequenceAcrossGenerations) {
  SemanticEpochGate gate;
  auto first = snapshot(70, "mic");
  first.sources.push_back(source("old", 70, "mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  auto reset = snapshot(71, "new_mic");
  reset.sources.push_back(source("new", 71, "new_mic"));
  reset.sequence_number = first.sequence_number;
  EXPECT_EQ(gate.observe(reset), SemanticEpochGate::SnapshotResult::Rejected);
  reset.sequence_number = 0;
  EXPECT_EQ(gate.observe(reset), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_TRUE(gate.accept(audio("old", 70, "mic")));
  EXPECT_FALSE(gate.accept(audio("new", 71, "new_mic")));
  reset.sequence_number = first.sequence_number + 1;
  EXPECT_EQ(gate.observe(reset), SemanticEpochGate::SnapshotResult::NewGeneration);
  EXPECT_FALSE(gate.accept(audio("old", 70, "mic")));
  EXPECT_TRUE(gate.accept(audio("new", 71, "new_mic")));
}
