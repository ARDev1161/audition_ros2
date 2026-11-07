#include <gtest/gtest.h>
#include <audition/classify/source_classification_runtime.hpp>
#include <audition/pipeline/temporal_acoustic_event_detector.hpp>
#include <audition/interfaces/classify.hpp>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

class DeterministicClassifier final : public audition::IAudioClassifier {
public:
  audition::BackendInfo backendInfo() const override { return {"test-classifier", "1"}; }
  audition::ClassifierCapabilities capabilities() const override {
    audition::ClassifierCapabilities out;
    out.audio.supported_sample_rates_hz = {16000};
    out.audio.min_channels = 1;
    out.audio.max_channels = 1;
    return out;
  }
  audition::ClassificationResult classify(audition::AudioView audio) const override {
    const auto value = audio.data()[0] >= 0.0f ? 0.90 : 0.10;
    audition::ClassificationResult result;
    result.classes.push_back({"Speech", audition::Probability::from(value)});
    return result;
  }
};

TEST(SemanticRuntime, LibauditionYieldsSourceSpecificStartAndEndTransitions) {
  audition::SourceClassificationRuntime runtime(
    std::make_unique<DeterministicClassifier>(), 16000, 8, 4, 8);
  audition::TemporalAcousticEventDetector detector;
  std::vector<audition::TemporalAcousticEvent> transitions;
  std::uint64_t timestamp = 0;

  for (int i = 0; i < 4; ++i) {
    std::vector<float> pcm(8, i < 2 ? 0.5f : -0.5f);
    runtime.push(7, pcm.data(), pcm.size());
    bool received = false;
    for (int attempt = 0; attempt < 100 && !received; ++attempt) {
      const auto completed = runtime.takeCompleted();
      if (!completed.empty()) {
        ASSERT_EQ(completed.size(), 1U);
        const auto & item = completed.front();
        EXPECT_EQ(item.source_id, 7U);
        EXPECT_EQ(item.end_sample, static_cast<std::uint64_t>((i + 1) * 8));
        timestamp += 500000;
        const auto events = detector.observe(
          audition::AcousticSourceId{item.source_id},
          audition::Timestamp{static_cast<std::int64_t>(timestamp),
                              {audition::ClockDomain::External, 1}},
          item.result);
        transitions.insert(transitions.end(), events.begin(), events.end());
        received = true;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    ASSERT_TRUE(received) << "Native libaudition inference worker did not finish";
  }

  ASSERT_EQ(transitions.size(), 2U);
  EXPECT_EQ(transitions[0].kind, audition::TemporalAcousticEventKind::Started);
  EXPECT_EQ(transitions[1].kind, audition::TemporalAcousticEventKind::Ended);
  EXPECT_EQ(transitions[0].occurrence_id, transitions[1].occurrence_id);
  EXPECT_EQ(transitions[0].label, "Speech");
  EXPECT_EQ(transitions[0].source_id.value(), 7U);
}
