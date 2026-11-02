#include "audition_perception/semantic_epoch_gate.hpp"

#include <gtest/gtest.h>

#include <cstdint>
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
  out.spatial_session_id = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  out.header.frame_id = frame;
  out.state = state;
  return out;
}
static Snapshot snapshot(std::uint64_t generation, const char * frame) {
  Snapshot out;
  out.generation = generation;
  out.scene_session_id = "0123456789abcdef0123456789abcdef";
  out.spatial_session_id = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
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
  static std::uint64_t next_audio_sequence = 0;
  out.sequence_number = ++next_audio_sequence;
  out.header.stamp.sec = static_cast<std::int32_t>(next_audio_sequence);
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


TEST(SemanticEpochGate, SceneRestartAcceptsNewSessionWithResetSequenceAndGeneration) {
  SemanticEpochGate gate;
  auto first = snapshot(50, "mic");
  first.sequence_number = 300;
  first.header.stamp.sec = 100;
  first.sources.push_back(source("s", 50, "mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_TRUE(gate.accept(audio("s", 50, "mic")));

  auto restarted = snapshot(1, "mic");
  restarted.scene_session_id = "fedcba9876543210fedcba9876543210";
  restarted.sequence_number = 1;
  restarted.header.stamp.sec = 101;
  restarted.sources.push_back(source("s", 1, "mic"));
  EXPECT_EQ(gate.observe(restarted), SemanticEpochGate::SnapshotResult::NewGeneration);
  EXPECT_EQ(status(gate).last_source_sequence, 1U);
  EXPECT_EQ(status(gate).scene_session_resets, 1U);
  EXPECT_EQ(status(gate).scene_session_id, restarted.scene_session_id);
  EXPECT_FALSE(gate.accept(audio("s", 50, "mic")));

  auto queued = audio("s", 1, "mic");
  queued.header.stamp.sec = 101;
  EXPECT_FALSE(gate.accept(queued));
  queued.header.stamp.sec = 102;
  EXPECT_TRUE(gate.accept(queued));

  // Old session cannot reclaim authority, even with an enormous sequence.
  first.sequence_number = 9999;
  EXPECT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Rejected);
  ++queued.sequence_number;
  queued.header.stamp.sec = 103;
  EXPECT_TRUE(gate.accept(queued));
  EXPECT_EQ(status(gate).scene_session_resets, 1U);
}

TEST(SemanticEpochGate, SameGenerationSceneRestartResetsAdmissionAndRejectsMalformedIds) {
  SemanticEpochGate gate;
  auto first = snapshot(3, "mic");
  first.header.stamp.sec = 100;
  first.sources.push_back(source("old", 3, "mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);

  auto bad = snapshot(3, "mic");
  bad.scene_session_id = "not-a-valid-session";
  bad.sources.push_back(source("injected", 3, "mic"));
  EXPECT_EQ(gate.observe(bad), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_TRUE(gate.accept(audio("old", 3, "mic")));

  auto next = snapshot(3, "mic");
  next.scene_session_id = "11111111111111111111111111111111";
  next.sequence_number = 1;
  next.header.stamp.sec = 101;
  next.sources.push_back(source("new", 3, "mic"));
  EXPECT_EQ(gate.observe(next), SemanticEpochGate::SnapshotResult::NewGeneration);
  EXPECT_FALSE(gate.accept(audio("old", 3, "mic")));
  auto fresh = audio("new", 3, "mic");
  fresh.header.stamp.sec = 102;
  EXPECT_TRUE(gate.accept(fresh));
  EXPECT_EQ(status(gate).stream_resets, 1U);
}

TEST(SemanticEpochGate, RejectsDuplicateReorderedAndRegressingPCMWithoutPoisoningCursor) {
  SemanticEpochGate gate;
  auto sources = snapshot(91, "mic");
  sources.sources.push_back(source("s", 91, "mic"));
  ASSERT_EQ(gate.observe(sources), SemanticEpochGate::SnapshotResult::Accepted);
  auto packet = audio("s", 91, "mic");
  packet.sequence_number = 100;
  packet.header.stamp.sec = 100;
  ASSERT_TRUE(gate.accept(packet));
  EXPECT_FALSE(gate.accept(packet));
  auto older = packet;
  older.sequence_number = 99;
  older.header.stamp.sec = 101;
  EXPECT_FALSE(gate.accept(older));
  older.sequence_number = 101;
  older.header.stamp.sec = 99;
  EXPECT_FALSE(gate.accept(older));
  auto fresh = packet;
  fresh.sequence_number = 101;
  fresh.header.stamp.sec = 101;
  EXPECT_TRUE(gate.accept(fresh));
  EXPECT_EQ(status(gate).accepted_audio_frames, 2U);
  EXPECT_EQ(status(gate).out_of_order_audio_frames, 3U);
  EXPECT_EQ(status(gate).dropped_audio_frames, 3U);
}

TEST(SemanticEpochGate, PerSourceCursorsResetOnlyOnRetirementReloadOrSceneEpoch) {
  SemanticEpochGate gate;
  auto first = snapshot(92, "mic");
  first.sources = {source("a", 92, "mic"), source("b", 92, "mic")};
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  auto a = audio("a", 92, "mic");
  a.sequence_number = 55;
  a.header.stamp.sec = 100;
  auto b = audio("b", 92, "mic");
  b.sequence_number = 2;
  b.header.stamp.sec = 100;
  ASSERT_TRUE(gate.accept(a));
  ASSERT_TRUE(gate.accept(b));
  auto refresh = first;
  ++refresh.sequence_number;
  ASSERT_EQ(gate.observe(refresh), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_FALSE(gate.accept(a));
  ++b.sequence_number;
  b.header.stamp.sec = 101;
  EXPECT_TRUE(gate.accept(b));
  auto retire = refresh;
  ++retire.sequence_number;
  retire.sources = {source("b", 92, "mic")};
  ASSERT_EQ(gate.observe(retire), SemanticEpochGate::SnapshotResult::Accepted);
  EXPECT_FALSE(gate.accept(a));
  gate.beginReload();
  EXPECT_FALSE(gate.accept(b));
  auto readmit = retire;
  ++readmit.sequence_number;
  readmit.header.stamp.sec = 102;
  ASSERT_EQ(gate.observe(readmit), SemanticEpochGate::SnapshotResult::Accepted);
  b.header.stamp.sec = 103;
  b.sequence_number = 1;
  EXPECT_TRUE(gate.accept(b));
  auto new_scene = readmit;
  new_scene.scene_session_id = "99999999999999999999999999999999";
  new_scene.sequence_number = 1;
  new_scene.header.stamp.sec = 104;
  ASSERT_EQ(gate.observe(new_scene), SemanticEpochGate::SnapshotResult::NewGeneration);
  b.header.stamp.sec = 105;
  b.sequence_number = 0;
  EXPECT_TRUE(gate.accept(b));
  EXPECT_EQ(status(gate).out_of_order_audio_frames, 1U);
}


TEST(SemanticEpochGate, SpatialRestartRejectsOldPCMEvenWithFutureTimestamp) {
  SemanticEpochGate gate;
  auto first = snapshot(25, "mic");
  first.sources.push_back(source("same", 25, "mic"));
  ASSERT_EQ(gate.observe(first), SemanticEpochGate::SnapshotResult::Accepted);
  auto old = audio("same", 25, "mic");
  old.header.stamp.sec = 100;
  ASSERT_TRUE(gate.accept(old));
  auto restarted = snapshot(1, "mic");
  restarted.scene_session_id = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
  restarted.spatial_session_id = "cccccccccccccccccccccccccccccccc";
  restarted.sequence_number = 1;
  restarted.header.stamp.sec = 101;
  restarted.sources.push_back(source("same", 1, "mic"));
  ASSERT_EQ(gate.observe(restarted), SemanticEpochGate::SnapshotResult::NewGeneration);
  old.source_generation = 1;
  old.sequence_number = 100000;
  old.header.stamp.sec = 100000;
  EXPECT_FALSE(gate.accept(old));
  auto fresh = old;
  fresh.spatial_session_id = restarted.spatial_session_id;
  EXPECT_TRUE(gate.accept(fresh));
  auto inconsistent = restarted;
  ++inconsistent.sequence_number;
  inconsistent.spatial_session_id = old.spatial_session_id;
  EXPECT_EQ(gate.observe(inconsistent), SemanticEpochGate::SnapshotResult::Rejected);
  EXPECT_FALSE(gate.accept(old));
}
