#include "audition_perception/speech_bridge.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#ifdef AUDITION_ROS2_WITH_SPEECH
#include <audition/audio/audio_buffer.hpp>
#include <audition/backends/sherpa/vad.hpp>
#include <audition/backends/whispercpp/asr.hpp>
#include <audition/speech/per_source_speech_runtime.hpp>
#endif

namespace audition_perception {
namespace {
template<typename T>
T readOrDeclare(rclcpp::Node & node, const std::string & name, const T & fallback) {
  if (!node.has_parameter(name)) return node.declare_parameter<T>(name, fallback);
  T value{};
  if (!node.get_parameter(name, value)) {
    throw std::invalid_argument("Cannot read existing speech parameter: " + name);
  }
  return value;
}

std::int64_t ns(const builtin_interfaces::msg::Time & t) {
  return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;
}
builtin_interfaces::msg::Time rosTime(std::int64_t n) {
  constexpr std::int64_t B=1000000000LL;
  auto s=n/B, sub=n%B;
  if(sub<0){--s;sub+=B;}
  if(s<std::numeric_limits<std::int32_t>::min() ||
     s>std::numeric_limits<std::int32_t>::max()) throw std::overflow_error("Speech stamp");
  builtin_interfaces::msg::Time t;
  t.sec=static_cast<std::int32_t>(s);t.nanosec=static_cast<std::uint32_t>(sub);
  return t;
}
}
class SpeechBridge::Impl {
public:
  explicit Impl(rclcpp::Node & node) {
    enabled_=readOrDeclare<bool>(node, "speech_enabled",false);
    if(!enabled_) return;
#ifndef AUDITION_ROS2_WITH_SPEECH
    throw std::runtime_error("Speech requested but binary lacks AUDITION_ROS2_WITH_SPEECH");
#else
    const auto vad_type=readOrDeclare<std::string>(node, "speech_vad_backend","sherpa");
    const auto asr_type=readOrDeclare<std::string>(node, "speech_asr_backend","whispercpp");
    const auto vad_model=readOrDeclare<std::string>(node, "speech_vad_model","");
    const auto asr_model=readOrDeclare<std::string>(node, "speech_asr_model","");
    const auto language=readOrDeclare<std::string>(node, "speech_language","ru");
    const int threads=readOrDeclare<int>(node, "speech_asr_threads",4);
    const int initial_ms=readOrDeclare<int>(node, "speech_native_initial_context_ms",1000);
    const int sources=readOrDeclare<int>(node, "speech_max_sources",8);
    const int pending=readOrDeclare<int>(node, "speech_max_pending_segments",16);
    const bool normalize=readOrDeclare<bool>(node, "speech_normalize_asr_segments",false);
    const double threshold=readOrDeclare<double>(node, "speech_vad_threshold",0.5);
    if(vad_type!="sherpa" || asr_type!="whispercpp" || language.empty() ||
       threads<1 || threads>32 || initial_ms<0 || initial_ms>1000 ||
       sources<1 || sources>64 || pending<1 || pending>128 ||
       !std::isfinite(threshold) || threshold<=0 || threshold>=1 ||
       !std::filesystem::is_regular_file(vad_model) ||
       !std::filesystem::is_regular_file(asr_model))
      throw std::invalid_argument("Invalid speech configuration or model files");
    audition::SherpaVadOptions vo;
    audition::SherpaSileroVadModel sm;
    sm.model=vad_model;sm.threshold=static_cast<float>(threshold);
    vo.model=sm;vo.sample_rate_hz=16000;
    vad_=std::make_unique<audition::SherpaVad>(vo);
    audition::WhisperCppAsrOptions wo;
    wo.model_path=asr_model;wo.language=language;wo.threads=threads;
    asr_=std::make_shared<audition::WhisperCppAsr>(wo);
    audition::PerSourceSpeechOptions so;
    so.sample_rate_hz=16000;so.max_input_chunk_samples=4096;
    so.max_sources=static_cast<std::size_t>(sources);
    so.max_pending_segments=static_cast<std::size_t>(pending);
    so.max_retired_generations=static_cast<std::size_t>(pending*2);
    so.native_initial_context_ms=static_cast<std::uint32_t>(initial_ms);
    so.normalize_asr_segments=normalize;
    runtime_=std::make_unique<audition::PerSourceSpeechRuntime>(*vad_,asr_,so);
    max_sources_=static_cast<std::size_t>(sources);
#endif
  }
  bool enabled() const noexcept {return enabled_;}
#ifdef AUDITION_ROS2_WITH_SPEECH
  struct Source {
    std::uint64_t id{0},generation{0},sequence{0},worker_generation{0};
    std::int64_t last_stamp{0},next_stamp{0};
    std::string name,frame_id;
  };
  void forget(std::map<std::string,Source>::iterator it) {
    runtime_->forget(it->second.id);
    if(it->second.worker_generation) meta_.erase(it->second.worker_generation);
    active_.erase(it);
  }
#endif
  void onAudio(const audition_msgs::msg::SourceAudioFrame & msg) {
    if(!enabled_) return;
#ifdef AUDITION_ROS2_WITH_SPEECH
    if(msg.source_id.empty() || msg.header.frame_id.empty() ||
       msg.sample_rate_hz!=16000 || msg.frame_count==0 || msg.frame_count>4096 ||
       msg.pcm.size()!=msg.frame_count ||
       !std::all_of(msg.pcm.begin(),msg.pcm.end(),[](float v){
         return std::isfinite(v) && v>=-1.0f && v<=1.0f;
       })) throw std::invalid_argument("Expected normalized mono 16k PCM");
    if(have_stream_ && msg.source_generation!=stream_generation_) return;
    const auto stamp=ns(msg.header.stamp);
    auto it=active_.find(msg.source_id);
    if(it!=active_.end()) {
      if(msg.source_generation < it->second.generation) return;
      if(msg.source_generation!=it->second.generation ||
         it->second.frame_id!=msg.header.frame_id ||
         msg.sequence_number<=it->second.sequence ||
         stamp<it->second.last_stamp ||
         stamp>it->second.next_stamp+250000000LL) {
        // A reset invalidates old queued ASR, rather than stitching sources.
        forget(it);
        it=active_.end();
      }
    }
    if(it==active_.end()) {
      if(active_.size()>=max_sources_) throw std::runtime_error("Speech max_sources");
      Source s;
      s.id=next_id_++;s.generation=msg.source_generation;
      s.name=msg.source_id;s.frame_id=msg.header.frame_id;
      it=active_.emplace(s.name,std::move(s)).first;
    }
    auto & s=it->second;
    s.sequence=msg.sequence_number;s.last_stamp=stamp;
    s.next_stamp=stamp+static_cast<std::int64_t>(msg.frame_count)*1000000000LL/16000;
    audition::AudioView audio{
      msg.pcm.data(),msg.pcm.size(),
      {16000U,1U,audition::AudioLayout::Interleaved},
      {stamp,{audition::ClockDomain::External,1}},msg.sequence_number};
    runtime_->push(s.id,audio); // Synchronous VAD in separate semantic process.
    const auto gen=runtime_->activeGeneration(s.id);
    if(gen && !s.worker_generation) {
      s.worker_generation=*gen;meta_[*gen]=s;
    }
#else
    (void)msg;
#endif
  }
  void resetStream() {
#ifdef AUDITION_ROS2_WITH_SPEECH
    if (!enabled_) return;
    runtime_->reset();
    active_.clear();
    meta_.clear();
    have_stream_ = false;
    stream_generation_ = 0;
#endif
  }
  void onSources(const audition_msgs::msg::AcousticSourceArray & msg) {
    if(!enabled_) return;
#ifdef AUDITION_ROS2_WITH_SPEECH
    if(have_stream_ && msg.generation<stream_generation_) return;
    if(have_stream_ && msg.generation!=stream_generation_) {
      runtime_->reset();active_.clear();meta_.clear();
    }
    have_stream_=true;stream_generation_=msg.generation;
    for(const auto & src:msg.sources) {
      if(src.state!=audition_msgs::msg::AcousticSource::ENDED) continue;
      auto it=active_.find(src.source_id);
      if(it==active_.end() || it->second.generation!=src.source_generation) continue;
      const auto gen=runtime_->retire(it->second.id);
      if(gen) {
        meta_[*gen]=it->second;active_.erase(it);
      } else forget(it);
    }
#else
    (void)msg;
#endif
  }
  std::vector<audition_msgs::msg::Transcript> drain() {
    std::vector<audition_msgs::msg::Transcript> out;
#ifdef AUDITION_ROS2_WITH_SPEECH
    if(!enabled_) return out;
    const auto drained=runtime_->drainReady();
    for(const auto & item:drained.results) {
      const auto m=meta_.find(item.generation);
      if(m==meta_.end() || item.transcript.text.empty()) continue;
      const auto & source=m->second;
      if(have_stream_ && source.generation!=stream_generation_) continue;
      audition_msgs::msg::Transcript t;
      t.header.frame_id=source.frame_id;
      t.header.stamp=rosTime(item.end_time.nanoseconds());
      t.source_id=source.name;t.source_generation=source.generation;
      t.speaker_id=""; // Source ID is NOT speaker identity.
      t.model_id="whispercpp";
      t.language=item.transcript.language;
      t.text=item.transcript.text;
      t.is_final=true;
      t.observed_start=rosTime(item.start_time.nanoseconds());
      t.observed_end=rosTime(item.end_time.nanoseconds());
      t.has_score=item.transcript.confidence.has_value();
      t.score_kind=audition_msgs::msg::Transcript::UNCALIBRATED_SCORE;
      if(item.transcript.confidence)
        t.score=static_cast<float>(item.transcript.confidence->value());
      out.push_back(std::move(t));
    }
    for(auto gen:drained.ready_retired_generations) {
      runtime_->releaseRetired(gen);meta_.erase(gen);
    }
#endif
    return out;
  }
private:
#ifdef AUDITION_ROS2_WITH_SPEECH
  std::uint64_t next_id_{1},stream_generation_{0};
  std::size_t max_sources_{8};
  bool have_stream_{false};
  std::map<std::string,Source> active_;
  std::map<std::uint64_t,Source> meta_;
  // Member teardown is reverse order: the ASR worker stops before the models.
  std::unique_ptr<audition::SherpaVad> vad_;
  std::shared_ptr<audition::WhisperCppAsr> asr_;
  std::unique_ptr<audition::PerSourceSpeechRuntime> runtime_;
#endif
  bool enabled_{false};
};

SpeechBridge::SpeechBridge(rclcpp::Node & node)
  : impl_(std::make_unique<Impl>(node)) {}
SpeechBridge::~SpeechBridge() = default;
bool SpeechBridge::enabled() const noexcept {return impl_->enabled();}
void SpeechBridge::onAudio(const audition_msgs::msg::SourceAudioFrame & f) {impl_->onAudio(f);}
void SpeechBridge::resetStream() {impl_->resetStream();}
void SpeechBridge::onSources(const audition_msgs::msg::AcousticSourceArray & s) {
  impl_->onSources(s);
}
std::vector<audition_msgs::msg::Transcript> SpeechBridge::drain() {
  return impl_->drain();
}
}  // namespace audition_perception
