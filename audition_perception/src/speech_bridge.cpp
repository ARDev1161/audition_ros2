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
    enabled_=node.declare_parameter<bool>("speech_enabled",false);
    if(!enabled_) return;
#ifndef AUDITION_ROS2_WITH_SPEECH
    throw std::runtime_error("Speech requested but binary lacks AUDITION_ROS2_WITH_SPEECH");
#else
    const auto vad_type=node.declare_parameter<std::string>("speech_vad_backend","sherpa");
    const auto asr_type=node.declare_parameter<std::string>("speech_asr_backend","whispercpp");
    const auto vad_model=node.declare_parameter<std::string>("speech_vad_model","");
    const auto asr_model=node.declare_parameter<std::string>("speech_asr_model","");
    const auto language=node.declare_parameter<std::string>("speech_language","ru");
    const int threads=node.declare_parameter<int>("speech_asr_threads",4);
    const int initial_ms=node.declare_parameter<int>("speech_native_initial_context_ms",1000);
    const int sources=node.declare_parameter<int>("speech_max_sources",8);
    const int pending=node.declare_parameter<int>("speech_max_pending_segments",16);
    const bool normalize=node.declare_parameter<bool>("speech_normalize_asr_segments",false);
    const double threshold=node.declare_parameter<double>("speech_vad_threshold",0.5);
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
