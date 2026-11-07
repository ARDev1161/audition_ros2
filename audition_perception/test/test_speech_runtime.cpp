#include <gtest/gtest.h>
#include <audition/speech/per_source_speech_runtime.hpp>
#include <audition/audio/audio_buffer.hpp>
#include <audition/interfaces/speech.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

class TestVadSession : public audition::IVadSession {
public:
  void reset() override {}
  audition::VadResult process(audition::AudioView audio) override {
    audition::VadResult output;
    output.speech_active = audio.sampleCount()>0 && audio.data()[0]>0.1f;
    return output;
  }
};
class TestVad : public audition::IVoiceActivityDetector {
public:
  audition::BackendInfo backendInfo() const override {return {"test-vad","1"};}
  audition::AudioRequirements audioRequirements() const override {
    audition::AudioRequirements r;
    r.supported_sample_rates_hz={16000};return r;
  }
  std::unique_ptr<audition::IVadSession> createSession() const override {
    return std::make_unique<TestVadSession>();
  }
};
class TestAsr : public audition::IAsrEngine {
public:
  audition::BackendInfo backendInfo() const override {return {"test-asr","1"};}
  audition::AsrCapabilities capabilities() const override {return {};}
  audition::Transcript transcribe(const audition::SpeechSegment& s) const override {
    audition::Transcript t;
    t.segment_id=s.segment_id;t.language="ru";t.text="тест";
    return t;
  }
};
static void feed(audition::PerSourceSpeechRuntime & runtime,
                 std::uint64_t source, float value, std::int64_t at) {
  std::vector<float> pcm(32,value);
  audition::AudioView audio{pcm.data(),pcm.size(),
    {16000U,1U,audition::AudioLayout::Interleaved},
    {at,{audition::ClockDomain::External,1}},0};
  runtime.push(source,audio);
}
TEST(SpeechRuntime, KeepsPerSourceGenerationsAndRetiresWithoutLosingText) {
  TestVad vad;
  auto asr=std::make_shared<TestAsr>();
  audition::PerSourceSpeechOptions options;
  options.min_speech_samples=16;
  options.pre_roll_samples=0;
  options.post_roll_samples=0;
  options.native_initial_context_ms=1000;
  options.max_pending_segments=8;
  audition::PerSourceSpeechRuntime runtime(vad,asr,options);

  feed(runtime,11,0.5f,1000000000LL);
  feed(runtime,22,0.5f,1000000000LL);
  const auto first=runtime.activeGeneration(11);
  const auto second=runtime.activeGeneration(22);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_NE(*first,*second);
  feed(runtime,11,0.0f,1002000000LL);
  const auto retired=runtime.retire(22);
  ASSERT_TRUE(retired.has_value());
  EXPECT_EQ(*retired,*second);

  std::vector<audition::SourceSpeechResult> completed;
  bool finished=false;
  for(int n=0;n<200 && !finished;++n) {
    auto drained=runtime.drainReady();
    for(auto & r:drained.results) completed.push_back(std::move(r));
    for(auto gen:drained.ready_retired_generations) {
      if(gen==*second) finished=true;
      runtime.releaseRetired(gen);
    }
    if(!finished) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(finished);
  bool saw_first=false,saw_second=false;
  for(const auto & r:completed) {
    EXPECT_EQ(r.transcript.language,"ru");
    EXPECT_EQ(r.transcript.text,"тест");
    EXPECT_GT(r.audio_end_sample,r.audio_start_sample);
    if(r.track_id==11){saw_first=true;EXPECT_EQ(r.generation,*first);}
    if(r.track_id==22){saw_second=true;EXPECT_EQ(r.generation,*second);}
  }
  EXPECT_TRUE(saw_first);
  EXPECT_TRUE(saw_second);
  // A new track with the same numerical ID always gets a different epoch.
  feed(runtime,22,0.5f,2000000000LL);
  ASSERT_TRUE(runtime.activeGeneration(22).has_value());
  EXPECT_NE(*runtime.activeGeneration(22),*second);
  runtime.forget(22);
}
