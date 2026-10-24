"""Generated ROS 2 interface smoke/serialization checks (not semantic validation)."""

import pytest
from rclpy.serialization import deserialize_message, serialize_message

from audition_msgs.msg import (
    AcousticEvent,
    AcousticScene,
    AcousticSource,
    AcousticSourceArray,
    AudioFrame,
    LocalizedSource,
    LocalizedSourceArray,
    SoundClassification,
    SourceAudioFrame,
    Transcript,
)


def test_capture_frame_round_trip():
    msg = AudioFrame()
    msg.header.frame_id = "microphone_array"
    msg.header.stamp.sec = 17
    msg.sequence_number = 29
    msg.sample_rate_hz = 16000
    msg.channel_count = 2
    msg.frame_count = 2
    msg.layout = AudioFrame.INTERLEAVED
    msg.channel_indices = [1, 2]
    msg.pcm = [0.25, -0.5, 0.125, -0.25]
    restored = deserialize_message(serialize_message(msg), AudioFrame)
    assert restored.header.stamp.sec == 17
    assert restored.header.frame_id == "microphone_array"
    assert list(restored.channel_indices) == [1, 2]
    assert list(restored.pcm) == pytest.approx([0.25, -0.5, 0.125, -0.25])


def test_source_identity_and_scene_round_trip():
    source = AcousticSource()
    source.header.frame_id = "microphone_array"
    source.source_id = "sensor-A/session-4/007"
    source.source_generation = 4
    source.spatial_track_id = "odas-2"
    source.has_direction = True
    source.direction_unit.x = 1.0
    source.has_range = False
    source.has_position = False

    snapshot = AcousticSourceArray()
    snapshot.generation = 4
    snapshot.sources = [source]
    back = deserialize_message(serialize_message(snapshot), AcousticSourceArray)
    assert back.sources[0].source_id == source.source_id
    assert back.sources[0].source_generation == 4
    assert not back.sources[0].has_range
    assert not back.sources[0].has_position

    evt = AcousticEvent()
    evt.source_id = source.source_id
    evt.source_generation = 4
    evt.event_id = "evt-7"
    evt.transition = AcousticEvent.STARTED
    evt.score_kind = AcousticEvent.UNCALIBRATED_SCORE
    scene = AcousticScene()
    scene.generation = 4
    scene.sources = [source]
    scene.active_events = [evt]
    scene_back = deserialize_message(serialize_message(scene), AcousticScene)
    assert scene_back.sources[0].source_generation == scene_back.generation
    assert scene_back.active_events[0].event_id == "evt-7"
    assert AcousticEvent.ENDED != AcousticEvent.CANCELLED


def test_semantic_message_generation_and_score_types():
    cls = SoundClassification()
    cls.source_id = "sensor-1/7"
    cls.source_generation = 12
    cls.class_label = "speech"
    cls.score_kind = SoundClassification.UNCALIBRATED_SCORE
    cls.score = 0.7
    result = deserialize_message(serialize_message(cls), SoundClassification)
    assert result.source_generation == 12
    assert result.score_kind == SoundClassification.UNCALIBRATED_SCORE
    assert SoundClassification.UNCALIBRATED_SCORE != SoundClassification.CALIBRATED_PROBABILITY

    audio = SourceAudioFrame()
    audio.source_id = cls.source_id
    audio.source_generation = cls.source_generation
    audio.sample_rate_hz = 16000
    audio.frame_count = 2
    audio.pcm = [0.0, 0.1]
    audio_back = deserialize_message(serialize_message(audio), SourceAudioFrame)
    assert audio_back.source_generation == result.source_generation

    transcript = Transcript()
    transcript.source_generation = result.source_generation
    transcript.is_final = True
    transcript.text = "hello"
    transcript_back = deserialize_message(serialize_message(transcript), Transcript)
    assert transcript_back.is_final
    assert transcript_back.text == "hello"


def test_localized_source_is_separate_from_bearing():
    result = LocalizedSource()
    result.source_id = "sensor-A/session-4/007"
    result.source_generation = 4
    result.header.frame_id = "map"
    result.position.x = 2.0
    result.covariance = [0.0] * 9
    result.localization_method = "test_fixture"
    frame = LocalizedSourceArray()
    frame.generation = 4
    frame.sources = [result]
    back = deserialize_message(serialize_message(frame), LocalizedSourceArray)
    assert back.sources[0].localization_method == "test_fixture"
    assert back.sources[0].header.frame_id == "map"
