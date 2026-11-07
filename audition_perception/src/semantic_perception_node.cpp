#include "audition_perception/semantic_perception_node.hpp"
#include "audition_perception/classifier_state_deadline_tracker.hpp"
#include "audition_perception/classifier_observation_config.hpp"

#include <algorithm>
#include <chrono>
#include <set>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef AUDITION_ROS2_WITH_SEMANTIC
#include <audition/classify/source_classification_runtime.hpp>
#include <audition/pipeline/temporal_acoustic_event_detector.hpp>
#ifdef AUDITION_ROS2_WITH_YAMNET
#include <audition/backends/yamnet/audio_tagger.hpp>
#endif
#endif

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace audition_perception {
namespace {
template<typename T>
T readOrDeclare(rclcpp::Node & node, const std::string & name, const T & fallback) {
  if (!node.has_parameter(name)) return node.declare_parameter<T>(name, fallback);
  T value{};
  if (!node.get_parameter(name, value)) {
    throw std::invalid_argument("Cannot read existing semantic parameter: " + name);
  }
  return value;
}

bool validInput(const audition_msgs::msg::SourceAudioFrame & msg) {
  return !msg.source_id.empty() && !msg.header.frame_id.empty() &&
    msg.header.stamp.nanosec < 1000000000U &&
    msg.sample_rate_hz == 16000 && msg.frame_count > 0 &&
    msg.frame_count <= 4096 && msg.pcm.size() == msg.frame_count &&
    std::all_of(msg.pcm.begin(), msg.pcm.end(), [](float value) {
      return std::isfinite(value) && value >= -1.0f && value <= 1.0f;
    });
}
#ifdef AUDITION_ROS2_WITH_SEMANTIC
std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
builtin_interfaces::msg::Time stampFromNs(std::int64_t ns) {
  constexpr auto billion = std::int64_t{1000000000};
  auto sec = ns / billion;
  auto sub = ns % billion;
  if (sub < 0) { --sec; sub += billion; }
  if (sec < std::numeric_limits<std::int32_t>::min() ||
      sec > std::numeric_limits<std::int32_t>::max()) {
    throw std::overflow_error("Semantic observation stamp outside ROS range");
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(sec);
  stamp.nanosec = static_cast<std::uint32_t>(sub);
  return stamp;
}
#endif
}  // namespace

class SemanticPerceptionNode::Impl {
public:
  Impl(SemanticPerceptionNode & node) {
    observation_config_.sample_period_ms =
      readOrDeclare<std::int64_t>(node, "classifier_workload_sample_period_ms", 1000);
    observation_config_.queued_threshold_ms =
      readOrDeclare<std::int64_t>(node, "classifier_queued_threshold_ms", 10000);
    observation_config_.inference_threshold_ms =
      readOrDeclare<std::int64_t>(node, "classifier_inference_threshold_ms", 30000);
    observation_config_.validate();
    backend_ = readOrDeclare<std::string>(node, "classifier_backend", "disabled");
    if (backend_ == "disabled") return;
    if (backend_ != "yamnet") {
      throw std::invalid_argument("Unknown semantic classifier backend: " + backend_);
    }
#ifndef AUDITION_ROS2_WITH_YAMNET
    throw std::runtime_error("YAMNet requested but this binary lacks "
                             "-DAUDITION_ROS2_WITH_YAMNET=ON");
#else
    const auto model = readOrDeclare<std::string>(node, "yamnet_model", "");
    const auto labels = readOrDeclare<std::string>(node, "yamnet_labels", "");
    const auto top_k = readOrDeclare<int>(node, "yamnet_top_k", 5);
    const auto threads = readOrDeclare<int>(node, "yamnet_threads", 1);
    const auto window = readOrDeclare<int>(node, "window_samples", 15600);
    const auto hop = readOrDeclare<int>(node, "hop_samples", 7800);
    max_sources_ = readOrDeclare<int>(node, "max_sources", 16);
    const auto start = readOrDeclare<double>(node, "event_start_threshold", 0.70);
    const auto stop = readOrDeclare<double>(node, "event_stop_threshold", 0.40);
    const auto rising = readOrDeclare<int>(node, "event_min_start_observations", 2);
    const auto falling = readOrDeclare<int>(node, "event_min_stop_observations", 2);
    if (model.empty() || labels.empty() || top_k <= 0 || top_k > 521 ||
        threads <= 0 || threads > 32 || window <= 0 || window > 160000 ||
        hop <= 0 || hop > window || max_sources_ <= 0 || max_sources_ > 128 ||
        rising < 1 || falling < 1) {
      throw std::invalid_argument("Invalid YAMNet model paths/window/event limits");
    }
    audition::YamnetOnnxOptions options;
    options.model = model;
    options.labels = labels;
    options.top_k = static_cast<std::size_t>(top_k);
    options.intra_op_threads = threads;
    auto classifier = std::make_unique<audition::YamnetAudioTagger>(options);
    if (!classifier->supportsWindowSamples(static_cast<std::size_t>(window))) {
      throw std::invalid_argument("YAMNet ONNX input shape conflicts with window_samples");
    }
    audition::TemporalAcousticEventDetectorOptions detector_options;
    detector_options.start_threshold = audition::Probability::from(start);
    detector_options.stop_threshold = audition::Probability::from(stop);
    detector_options.min_start_observations = static_cast<std::size_t>(rising);
    detector_options.min_stop_observations = static_cast<std::size_t>(falling);
    detector_options.max_sources = static_cast<std::size_t>(max_sources_);
    detector_options.max_labels_per_source = 64;
    detector_options.max_observation_gap = audition::Duration{2000000000};
    detector_ = std::make_unique<audition::TemporalAcousticEventDetector>(detector_options);
    // This is libaudition's bounded inference worker, not a new ROS worker.
    runtime_ = std::make_unique<audition::SourceClassificationRuntime>(
      std::move(classifier), 16000, static_cast<std::size_t>(window),
      static_cast<std::size_t>(max_sources_), static_cast<std::size_t>(hop));
#endif
  }

  bool enabled() const noexcept { return backend_ == "yamnet"; }
  const ClassifierObservationConfig & observationConfig() const { return observation_config_; }
  ClassifierWorkloadSnapshot workload() const {
    ClassifierWorkloadAccumulator accumulator;
#ifdef AUDITION_ROS2_WITH_YAMNET
    if (enabled()) {
      // Query native status for each live worker; source counts are not job counts.
      for (const auto & item : sources_) {
        const auto status = runtime_->status(item.second.id);
        ClassifierWorkState state = ClassifierWorkState::Waiting;
        switch (status.state) {
          case audition::SourceClassificationState::WaitingForAudio:
            state = ClassifierWorkState::Waiting; break;
          case audition::SourceClassificationState::Collecting:
            state = ClassifierWorkState::Collecting; break;
          case audition::SourceClassificationState::Queued:
            state = ClassifierWorkState::Queued; break;
          case audition::SourceClassificationState::Inferencing:
            state = ClassifierWorkState::Inferencing; break;
          case audition::SourceClassificationState::Classified:
            state = ClassifierWorkState::Classified; break;
          case audition::SourceClassificationState::Error:
            state = ClassifierWorkState::Error; break;
        }
        deadline_tracker_.observe(std::to_string(item.second.id), state);
        accumulator.observe(state, status.dropped_windows,
                            status.last_inference_ms, status.last_queue_wait_ms);
      }
    }
#endif
    deadline_tracker_.finishSample();
    return accumulator.snapshot();
  }

  ClassifierWorkloadSnapshot cachedWorkload() const { return cached_workload_; }
  bool hasSample() const { return sample_count_ > 0; }
  std::uint64_t sampleCount() const { return sample_count_; }
  std::uint64_t sampleFailures() const { return sample_failures_; }
  std::uint64_t sampleAgeMs() const {
    if (!hasSample()) return 0;
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_sample_).count());
  }
  void sampleWorkload() {
    try {
      const auto sampled = workload();
      cached_workload_ = sampled;
      last_sample_ = std::chrono::steady_clock::now();
      ++sample_count_;
      sample_valid_ = true;
    } catch (...) {
      ++sample_failures_;
      sample_valid_ = false;
      deadline_tracker_.reset();
      cached_workload_ = {};
      throw;
    }
  }
  bool sampleValid() const { return sample_valid_; }
  void invalidateSample() {
    deadline_tracker_.reset();
    cached_workload_ = {};
    sample_valid_ = false;
  }
  ClassifierDeadlineSnapshot deadlines() const {
    return deadline_tracker_.snapshot(
      static_cast<std::uint64_t>(observation_config_.queued_threshold_ms),
      static_cast<std::uint64_t>(observation_config_.inference_threshold_ms));
  }

  void resetStream() {
    invalidateSample();
#ifdef AUDITION_ROS2_WITH_YAMNET
    if (!enabled()) return;
    // Forget workers before a new epoch is admitted. A late completed job
    // has no source-ID mapping and can never publish into the new stream.
    for (const auto & item : sources_) {
      runtime_->forget(item.second.id);
      const auto stamp_ns = std::max(item.second.last_timestamp_ns,
                                     item.second.last_result_ns);
      detector_->retire(audition::AcousticSourceId{item.second.id},
        audition::Timestamp{stamp_ns, {audition::ClockDomain::External, 1}});
    }
    sources_.clear();
    id_to_name_.clear();
#endif
  }

  void onAudio(const audition_msgs::msg::SourceAudioFrame & msg,
    const rclcpp::Publisher<audition_msgs::msg::AcousticEvent>::SharedPtr & event_pub)
  {
    if (!enabled()) return;
#ifdef AUDITION_ROS2_WITH_YAMNET
    const auto timestamp = stampNs(msg.header.stamp);
    auto it = sources_.find(msg.source_id);
    if (it != sources_.end() && msg.source_generation < it->second.generation) return;

    // Never join two separated-source epochs or gaps into one YAMNet window.
    const auto discontinuity = it != sources_.end() &&
      (msg.source_generation != it->second.generation ||
       msg.sequence_number != it->second.sequence + 1 ||
       timestamp < it->second.last_timestamp_ns ||
       timestamp > it->second.expected_next_ns + 250000000LL);
    if (discontinuity) {
      retire(it, timestamp, event_pub, "source_discontinuity");
      it = sources_.end();
    }
    if (it == sources_.end()) {
      if (sources_.size() >= static_cast<std::size_t>(max_sources_)) {
        throw std::runtime_error("Semantic max_sources exceeded");
      }
      Source state;
      state.id = next_id_++;
      state.generation = msg.source_generation;
      state.anchor_ns = timestamp;
      state.frame_id = msg.header.frame_id;
      it = sources_.emplace(msg.source_id, std::move(state)).first;
      id_to_name_.emplace(it->second.id, msg.source_id);
    }
    if (it->second.frame_id != msg.header.frame_id) {
      retire(it, timestamp, event_pub, "frame_changed");
      throw std::invalid_argument("Source frame changed without generation");
    }
    it->second.sequence = msg.sequence_number;
    it->second.last_timestamp_ns = timestamp;
    it->second.sample_count += msg.frame_count;
    it->second.expected_next_ns = timestamp +
      static_cast<std::int64_t>(msg.frame_count) * 1000000000LL / 16000;
    runtime_->push(it->second.id, msg.pcm.data(), msg.pcm.size());
#else
    (void) msg; (void) event_pub;
#endif
  }

  void onSources(const audition_msgs::msg::AcousticSourceArray & msg,
    const rclcpp::Publisher<audition_msgs::msg::AcousticEvent>::SharedPtr & event_pub)
  {
#ifdef AUDITION_ROS2_WITH_YAMNET
    if (!enabled()) return;
    for (const auto & source : msg.sources) {
      if (source.state != audition_msgs::msg::AcousticSource::ENDED) continue;
      auto it = sources_.find(source.source_id);
      if (it != sources_.end() && it->second.generation == source.source_generation) {
        retire(it, stampNs(msg.header.stamp), event_pub, "source_ended");
      }
    }
#else
    (void) msg; (void) event_pub;
#endif
  }

  void poll(
    const rclcpp::Publisher<audition_msgs::msg::SoundClassification>::SharedPtr & class_pub,
    const rclcpp::Publisher<audition_msgs::msg::AcousticEvent>::SharedPtr & event_pub,
    InferenceCompletionStats & stats)
  {
#ifdef AUDITION_ROS2_WITH_YAMNET
    if (!enabled()) return;
    for (const auto & completed : runtime_->takeCompleted()) {
      const auto name = id_to_name_.find(completed.source_id);
      if (name == id_to_name_.end()) {
        stats.classifierDiscarded(); continue;  // retired worker generation
      }
      const auto it = sources_.find(name->second);
      if (it == sources_.end() || it->second.id != completed.source_id ||
          completed.end_sample <= it->second.last_result_end_sample) {
        stats.classifierDiscarded(); continue;
      }
      auto & source = it->second;
      if (completed.end_sample > source.sample_count) {
        stats.classifierDiscarded(); continue;
      }
      const auto end_ns = source.anchor_ns +
        static_cast<std::int64_t>(completed.end_sample) * 1000000000LL / 16000;
      if (end_ns <= source.last_result_ns) {
        stats.classifierDiscarded(); continue;
      }
      source.last_result_end_sample = completed.end_sample;
      source.last_result_ns = end_ns;
      const audition::Timestamp observed{end_ns, {audition::ClockDomain::External, 1}};
      for (const auto & cls : completed.result.classes) {
        audition_msgs::msg::SoundClassification out;
        out.header.frame_id = source.frame_id;
        out.header.stamp = stampFromNs(end_ns);
        out.source_id = name->second;
        out.source_generation = source.generation;
        out.class_label = cls.label;
        out.taxonomy_version = "AudioSet-521";
        out.model_id = "yamnet-onnx";
        out.score_kind = audition_msgs::msg::SoundClassification::UNCALIBRATED_SCORE;
        out.score = static_cast<float>(cls.probability.value());
        class_pub->publish(std::move(out));
      }
      std::set<std::uint64_t> newly_started;
      for (const auto & event : detector_->observe(
              audition::AcousticSourceId{completed.source_id}, observed,
              completed.result)) {
        if (event.kind == audition::TemporalAcousticEventKind::Started) {
          newly_started.insert(event.occurrence_id);
        }
        event_pub->publish(toRosEvent(event, name->second, source,
          "classification_hysteresis"));
      }
      // A new supporting observation for an already active occurrence
      // updates the canonical event without fabricating a new onset.
      for (const auto & event : detector_->activeEvents()) {
        if (event.source_id.value() != completed.source_id ||
            event.last_supported_time.nanoseconds() != end_ns ||
            newly_started.count(event.occurrence_id) != 0) continue;
        auto update = toRosEvent(event, name->second, source, "");
        update.transition = audition_msgs::msg::AcousticEvent::UPDATED;
        update.header.stamp = stampFromNs(end_ns);
        event_pub->publish(std::move(update));
      }
      // A completed window need not have any class above the event threshold.
      stats.classifierCompleted();
    }
#else
    (void) class_pub; (void) event_pub; (void) stats;
#endif
  }

private:
#ifdef AUDITION_ROS2_WITH_YAMNET
  struct Source {
    std::uint64_t id{0};
    std::uint64_t generation{0};
    std::uint64_t sequence{0};
    std::uint64_t sample_count{0};
    std::uint64_t last_result_end_sample{0};
    std::int64_t anchor_ns{0};
    std::int64_t last_timestamp_ns{0};
    std::int64_t expected_next_ns{0};
    std::int64_t last_result_ns{std::numeric_limits<std::int64_t>::min()};
    std::string frame_id;
  };
  audition_msgs::msg::AcousticEvent toRosEvent(
    const audition::TemporalAcousticEvent & event,
    const std::string & name, const Source & source,
    const std::string & reason) const {
    audition_msgs::msg::AcousticEvent out;
    out.header.frame_id = source.frame_id;
    out.header.stamp = stampFromNs(event.last_supported_time.nanoseconds());
    out.event_id = "yamnet-" + std::to_string(source.id) + "-" +
                   std::to_string(event.occurrence_id);
    out.source_id = name;
    out.source_generation = source.generation;
    out.class_label = event.label;
    out.taxonomy_version = "AudioSet-521";
    out.model_id = "yamnet-onnx";
    out.transition = event.kind == audition::TemporalAcousticEventKind::Started
      ? audition_msgs::msg::AcousticEvent::STARTED
      : audition_msgs::msg::AcousticEvent::ENDED;
    out.score_kind = audition_msgs::msg::AcousticEvent::UNCALIBRATED_SCORE;
    out.score = static_cast<float>(event.peak_score.value());
    out.observed_start = stampFromNs(event.start_time.nanoseconds());
    out.observed_end = stampFromNs(event.last_supported_time.nanoseconds());
    if (out.transition == audition_msgs::msg::AcousticEvent::ENDED) {
      out.termination_reason = reason;
    }
    return out;
  }
  void retire(
    std::map<std::string, Source>::iterator it, std::int64_t stamp_ns,
    const rclcpp::Publisher<audition_msgs::msg::AcousticEvent>::SharedPtr & event_pub,
    const std::string & reason) {
    const auto ending = std::max(stamp_ns, it->second.last_result_ns);
    const audition::Timestamp stamp{
      ending, {audition::ClockDomain::External, 1}};
    for (const auto & event : detector_->retire(
           audition::AcousticSourceId{it->second.id}, stamp)) {
      event_pub->publish(toRosEvent(event, it->first, it->second, reason));
    }
    runtime_->forget(it->second.id);
    id_to_name_.erase(it->second.id);
    sources_.erase(it);
    invalidateSample();
  }
  std::uint64_t next_id_{1};
  int max_sources_{16};
  std::map<std::string, Source> sources_;
  std::map<std::uint64_t, std::string> id_to_name_;
  std::unique_ptr<audition::SourceClassificationRuntime> runtime_;
  std::unique_ptr<audition::TemporalAcousticEventDetector> detector_;
#endif
  mutable ClassifierStateDeadlineTracker deadline_tracker_;
  ClassifierWorkloadSnapshot cached_workload_{};
  std::uint64_t sample_count_{0}, sample_failures_{0};
  bool sample_valid_{false};
  std::chrono::steady_clock::time_point last_sample_{};
  ClassifierObservationConfig observation_config_{};
  std::string backend_;
};

SemanticPerceptionNode::SemanticPerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("semantic_perception", options)
{
  impl_ = std::make_unique<Impl>(*this);
  speech_ = std::make_unique<SpeechBridge>(*this);
  asr_observation_config_.minimum_segments = declare_parameter<std::int64_t>("asr_pressure_minimum_segments", 4);
  asr_observation_config_.minimum_age_ms = declare_parameter<std::int64_t>("asr_pressure_minimum_age_ms", 10000);
  asr_observation_config_.max_sample_age_ms = declare_parameter<std::int64_t>("asr_observation_max_sample_age_ms", 3000);
  asr_observation_config_.validate(impl_->observationConfig().sample_period_ms);
  transcript_pub_ = create_publisher<audition_msgs::msg::Transcript>(
    "internal/transcripts", rclcpp::QoS(20).reliable());
  progress_pub_ = create_publisher<audition_msgs::msg::SemanticProgress>(
    "internal/semantic_progress", rclcpp::QoS(20).reliable());
  class_pub_ = create_publisher<audition_msgs::msg::SoundClassification>(
    "internal/classifications", rclcpp::QoS(20).reliable());
  event_pub_ = create_publisher<audition_msgs::msg::AcousticEvent>(
    "internal/events", rclcpp::QoS(20).reliable());
  sub_ = create_subscription<audition_msgs::msg::SourceAudioFrame>(
    "internal/source_audio", rclcpp::SensorDataQoS(),
    [this](const audition_msgs::msg::SourceAudioFrame & msg) { onSourceAudio(msg); });
  sources_sub_ = create_subscription<audition_msgs::msg::AcousticSourceArray>(
    "sources", rclcpp::QoS(10).reliable(),
    [this](const audition_msgs::msg::AcousticSourceArray & msg) { onSources(msg); });
  // Native worker and graph configuration are startup-only. A parameter
  // update without rebuilding workers would silently keep stale state.
  profile_name_ = declare_parameter<std::string>("profile_name", "manual");
  profile_spatial_backend_ =
    declare_parameter<std::string>("profile_spatial_backend", "unknown");
  parameter_guard_ = add_on_set_parameters_callback(
    [](const std::vector<rclcpp::Parameter> & changes) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = changes.empty();
      if (!result.successful) {
        result.reason = "M2 pipeline parameters are startup-only; stop and relaunch "
                        "with a validated profile (transactional reload not implemented)";
      }
      return result;
    });
  capabilities_srv_ = create_service<audition_msgs::srv::GetCapabilities>(
    "semantic_perception/get_capabilities",
    [this](
      const std::shared_ptr<audition_msgs::srv::GetCapabilities::Request>,
      std::shared_ptr<audition_msgs::srv::GetCapabilities::Response> response)
    {
      response->profile_name = profile_name_;
      response->spatial_backend = profile_spatial_backend_;
      response->classifier_backend = impl_->enabled() ? "yamnet" : "disabled";
      response->classifier_active = impl_->enabled();
      response->speech_active = speech_->enabled();
      response->compiled_classifiers = {"disabled"};
#ifdef AUDITION_ROS2_WITH_ODAS
      response->odas_compiled = true;
#endif
#ifdef AUDITION_ROS2_WITH_YAMNET
      response->yamnet_compiled = true;
      response->compiled_classifiers.push_back("yamnet");
#endif
#ifdef AUDITION_ROS2_WITH_SPEECH
      response->speech_compiled = true;
      response->compiled_speech_backends = {"sherpa", "whispercpp"};
#endif
      if (impl_->enabled()) {
        response->loaded_models.push_back("yamnet");
        response->loaded_models.push_back("yamnet_labels");
      }
      if (speech_->enabled()) {
        response->loaded_models.push_back("speech_vad");
        response->loaded_models.push_back("speech_asr");
      }
      response->runtime_reconfiguration_supported = false;
      response->note = "Loaded models reflect successful startup, not accuracy or "
                       "runtime health; spatial_backend is declared profile intent, "
                       "not an independent ODAS health check";
    });
  semantic_status_srv_ = create_service<audition_msgs::srv::GetSemanticStatus>(
    "semantic_perception/get_semantic_status",
    [this](
      const std::shared_ptr<audition_msgs::srv::GetSemanticStatus::Request>,
      std::shared_ptr<audition_msgs::srv::GetSemanticStatus::Response> response)
    {
      gate_.status(*response);
      response->reload_epoch = reload_epoch_;
      response->reload_successes = reload_successes_;
      response->reload_failures = reload_failures_;
      response->reload_pending = gate_.reloadPending();
      const auto inference = inference_stats_.snapshot();
      response->classifier_active = impl_->enabled();
      response->speech_active = speech_->enabled();
      response->classifier_completed_windows = inference.classifier_completed_windows;
      response->classifier_discarded_results = inference.classifier_discarded_results;
      response->classifier_last_completion_age_ms =
        inference.classifier_last_completion_age_ms;
      response->asr_completed_segments = inference.asr_completed_segments;
      response->asr_empty_segments = inference.asr_empty_segments;
      response->asr_discarded_results = inference.asr_discarded_results;
      response->asr_last_completion_age_ms = inference.asr_last_completion_age_ms;
      response->inference_poll_failures = inference.inference_poll_failures;
      const auto classifier_work = impl_->cachedWorkload();
      response->classifier_has_workload_sample = impl_->hasSample();
      response->classifier_workload_sample_valid = impl_->sampleValid();
      response->classifier_workload_sample_period_ms =
        static_cast<std::uint64_t>(impl_->observationConfig().sample_period_ms);
      response->classifier_queued_threshold_ms =
        static_cast<std::uint64_t>(impl_->observationConfig().queued_threshold_ms);
      response->classifier_inference_threshold_ms =
        static_cast<std::uint64_t>(impl_->observationConfig().inference_threshold_ms);
      response->classifier_workload_samples = impl_->sampleCount();
      response->classifier_workload_sample_failures = impl_->sampleFailures();
      response->classifier_last_workload_sample_age_ms = impl_->sampleAgeMs();
      response->classifier_active_sources = classifier_work.active_sources;
      response->classifier_waiting_sources = classifier_work.waiting_sources;
      response->classifier_collecting_sources = classifier_work.collecting_sources;
      response->classifier_queued_sources = classifier_work.queued_sources;
      response->classifier_inferencing_sources = classifier_work.inferencing_sources;
      response->classifier_classified_sources = classifier_work.classified_sources;
      response->classifier_error_sources = classifier_work.error_sources;
      response->classifier_dropped_windows_active = classifier_work.dropped_windows_active;
      response->classifier_has_inference_timing = classifier_work.have_inference_timing;
      response->classifier_max_last_inference_ms = classifier_work.max_last_inference_ms;
      response->classifier_has_queue_wait_timing = classifier_work.have_queue_wait_timing;
      response->classifier_max_last_queue_wait_ms = classifier_work.max_last_queue_wait_ms;
      const auto deadlines = impl_->sampleValid() ? impl_->deadlines() :
        ClassifierDeadlineSnapshot{};
      response->classifier_overdue_queued_sources = deadlines.overdue_queued_sources;
      response->classifier_overdue_inferencing_sources = deadlines.overdue_inferencing_sources;
      response->classifier_max_queued_state_age_ms = deadlines.max_queued_age_ms;
      response->classifier_max_inferencing_state_age_ms = deadlines.max_inferencing_age_ms;
      const auto speech_work = speech_observer_.current();
      response->asr_has_workload_sample = speech_observer_.everSampled();
      response->asr_workload_sample_valid = speech_observer_.valid();
      response->asr_workload_samples = speech_observer_.samples();
      response->asr_workload_sample_failures = speech_observer_.failures();
      response->asr_last_workload_sample_age_ms = speech_observer_.ageMs();
      response->asr_peak_queued_segments = speech_observer_.peakQueued();
      response->asr_peak_buffered_results = speech_observer_.peakBuffered();
      response->asr_queue_nonempty_age_ms = speech_observer_.queueNonemptyAgeMs();
      response->asr_mailbox_nonempty_age_ms = speech_observer_.mailboxNonemptyAgeMs();
      response->asr_sustained_queue_pressure = speech_observer_.queuePressure(
        static_cast<std::uint64_t>(asr_observation_config_.minimum_segments),
        static_cast<std::uint64_t>(asr_observation_config_.minimum_age_ms),
        SpeechWorkloadObserver::Clock::now(),
        static_cast<std::uint64_t>(asr_observation_config_.max_sample_age_ms));
      response->asr_workload_sample_fresh = speech_observer_.fresh(
        static_cast<std::uint64_t>(asr_observation_config_.max_sample_age_ms));
      response->asr_contiguous_queue_samples = speech_observer_.contiguousQueueSamples();
      response->asr_last_dropped_delta = speech_observer_.lastDroppedDelta();
      response->asr_last_evicted_delta = speech_observer_.lastEvictedDelta();
      response->asr_observed_dropped_delta_total = speech_observer_.observedDroppedDeltaTotal();
      response->asr_observed_evicted_delta_total = speech_observer_.observedEvictedDeltaTotal();
      response->asr_loss_on_last_sample = speech_observer_.lossOnLastSample();
      response->asr_pressure_minimum_segments = static_cast<std::uint64_t>(asr_observation_config_.minimum_segments);
      response->asr_pressure_minimum_age_ms = static_cast<std::uint64_t>(asr_observation_config_.minimum_age_ms);
      response->asr_observation_max_sample_age_ms = static_cast<std::uint64_t>(asr_observation_config_.max_sample_age_ms);
      response->asr_queued_segments = speech_work.queued_segments;
      response->asr_buffered_results = speech_work.buffered_results;
      response->asr_retired_generations = speech_work.retired_generations;
      response->asr_completed_segments_native = speech_work.completed_segments_native;
      response->asr_dropped_segments_native = speech_work.dropped_segments_native;
      response->asr_evicted_results_native = speech_work.evicted_results_native;
      response->asr_worker_error = speech_work.worker_error;
    });
  reload_srv_ = create_service<audition_msgs::srv::ReloadSemanticWorkers>(
    "semantic_perception/reload_workers",
    [this](
      const std::shared_ptr<audition_msgs::srv::ReloadSemanticWorkers::Request> request,
      std::shared_ptr<audition_msgs::srv::ReloadSemanticWorkers::Response> response)
    {
      // All subscriptions, timer and services use the default mutually-exclusive
      // callback group. No old inference completion can publish during swap.
      try {
        auto next_classifier = std::make_unique<Impl>(*this);
        auto next_speech = std::make_unique<SpeechBridge>(*this);
        if (!request->dry_run) {
          // Construct both before touching the running graph: strong rollback
          // guarantee on validation/model load errors.
          impl_.swap(next_classifier);
          speech_.swap(next_speech);
          speech_observer_.reset();
          gate_.beginReload();
          ++reload_epoch_;
          ++reload_successes_;
        }
        response->success = true;
        response->message = request->dry_run ?
          "Candidate workers validated; active workers unchanged" :
          "Semantic workers replaced; awaiting newer canonical source snapshot";
      } catch (const std::exception & error) {
        ++reload_failures_;
        response->success = false;
        response->message = error.what();
        RCLCPP_ERROR(get_logger(), "Semantic worker reload failed (rollback): %s",
          error.what());
      }
      response->reload_epoch = reload_epoch_;
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { pollCompleted(); });
  workload_timer_ = create_wall_timer(
    std::chrono::milliseconds(impl_->observationConfig().sample_period_ms), [this]() {
    try { impl_->sampleWorkload(); }
    catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Semantic workload sample failed: %s", error.what());
    }
    try { speech_observer_.sample(speech_->workload()); }
    catch (const std::exception & error) {
      speech_observer_.failed();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ASR workload sample failed: %s", error.what());
    }
  });
  RCLCPP_INFO(get_logger(), "Semantic backend: %s",
    impl_->enabled() ? "yamnet" : "disabled");
}
SemanticPerceptionNode::~SemanticPerceptionNode() = default;

void SemanticPerceptionNode::onSourceAudio(
  const audition_msgs::msg::SourceAudioFrame & msg)
{
  if (!validInput(msg)) {
    gate_.rejectInvalidAudio();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Rejected invalid source audio: expected normalized mono 16 kHz");
    return;
  }
  if (!gate_.accept(msg)) return;
  try {
    impl_->onAudio(msg, event_pub_);
    speech_->onAudio(msg);
    // Ack only after canonical admission AND successful worker submission.
    // This does not imply that asynchronous YAMNet/ASR inference completed.
    audition_msgs::msg::SemanticProgress progress;
    progress.spatial_session_id = msg.spatial_session_id;
    progress.source_id = msg.source_id;
    progress.source_generation = msg.source_generation;
    progress.sequence_number = msg.sequence_number;
    progress.frame_count = msg.frame_count;
    progress_pub_->publish(std::move(progress));
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
      "Semantic source rejected: %s", error.what());
  }
}
void SemanticPerceptionNode::onSources(
  const audition_msgs::msg::AcousticSourceArray & msg)
{
  const auto decision = gate_.observe(msg);
  if (decision == SemanticEpochGate::SnapshotResult::Rejected) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Rejected malformed or stale canonical source snapshot");
    return;
  }
  try {
    if (decision == SemanticEpochGate::SnapshotResult::NewGeneration) {
      impl_->resetStream();
      speech_->resetStream();
      speech_observer_.reset();
    }
    impl_->onSources(msg, event_pub_);
    speech_->onSources(msg);
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
      "Semantic source retirement error: %s", error.what());
  }
}
void SemanticPerceptionNode::pollCompleted() {
  try {
    impl_->poll(class_pub_, event_pub_, inference_stats_);
    auto drained = speech_->drain();
    inference_stats_.asrCompleted(drained.completed_segments,
      drained.empty_segments, drained.discarded_segments);
    for (auto & t : drained.transcripts) transcript_pub_->publish(std::move(t));
  } catch (const std::exception & error) {
    inference_stats_.pollFailed();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
      "Semantic inference completion failed: %s", error.what());
  }
}
}  // namespace audition_perception
RCLCPP_COMPONENTS_REGISTER_NODE(audition_perception::SemanticPerceptionNode)
