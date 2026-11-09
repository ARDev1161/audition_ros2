"""Bounded temporal corroboration of radar/audio *diagnostic* candidates.

This module never verifies physical emitter identity. A radar return and
acoustic ray can belong to unrelated objects for arbitrarily many frames.
Both modalities must independently refresh on each counted observation.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class RadarTemporalEvidence:
    supporting_synchronized_observations: int
    temporally_correlated: bool


class RadarPairTemporalGate:
    def __init__(self, *, required_observations=3,
                 minimum_spacing_ns=100_000_000,
                 maximum_gap_ns=600_000_000, maximum_pairs=16):
        if (not 2 <= required_observations <= 10
                or not 0 < minimum_spacing_ns < maximum_gap_ns
                or not 1 <= maximum_pairs <= 64):
            raise ValueError('invalid temporal radar association parameters')
        self.required = required_observations
        self.spacing_ns = minimum_spacing_ns
        self.gap_ns = maximum_gap_ns
        self.max_pairs = maximum_pairs
        self.reset()

    def reset(self):
        self.epochs = None
        self.last_clock_ns = None
        self.history = {}

    @staticmethod
    def key(candidate):
        return (candidate.audio_id, candidate.audio_generation,
                candidate.radar_id, candidate.radar_generation)

    def observe(self, candidates, *, audio_session, radar_session,
                audio_stamp_ns, radar_stamp_ns, clock_ns):
        # Invalid snapshots must revoke all previously accumulated evidence.
        if (not audio_session or not radar_session
                or type(audio_stamp_ns) is not int or audio_stamp_ns <= 0
                or type(radar_stamp_ns) is not int or radar_stamp_ns <= 0
                or type(clock_ns) is not int or clock_ns <= 0
                or len(candidates) > self.max_pairs):
            self.reset()
            return {}
        epochs = (audio_session, radar_session)
        if (self.epochs != epochs
                or (self.last_clock_ns is not None
                    and clock_ns < self.last_clock_ns)):
            self.reset()
        self.epochs = epochs
        self.last_clock_ns = clock_ns
        keys = [self.key(c) for c in candidates]
        if len(set(keys)) != len(keys):
            self.history.clear()
            return {}
        audio_counts = {}
        radar_counts = {}
        for k in keys:
            audio_counts[k[:2]] = audio_counts.get(k[:2], 0)+1
            radar_counts[k[2:]] = radar_counts.get(k[2:], 0)+1
        # Competing edges cannot quietly carry forward a previous pair's
        # confirmation history, even when one has a lower residual.
        eligible = {
            self.key(c) for c in candidates
            if (c.unique_match
                and audio_counts[self.key(c)[:2]] == 1
                and radar_counts[self.key(c)[2:]] == 1)
        }
        self.history = {k:v for k,v in self.history.items() if k in eligible}
        results = {}
        for key in sorted(eligible):
            prev = self.history.get(key)
            if prev is None:
                value = (1,audio_stamp_ns,radar_stamp_ns)
                advanced = False
            else:
                count, prev_audio, prev_radar = prev
                da = audio_stamp_ns-prev_audio
                dr = radar_stamp_ns-prev_radar
                if (da < 0 or dr < 0
                        or da > self.gap_ns or dr > self.gap_ns):
                    value = (1,audio_stamp_ns,radar_stamp_ns)
                    advanced = False
                elif (da < self.spacing_ns or dr < self.spacing_ns):
                    # Same, replayed, or too-close observations do not
                    # provide independent cross-modal evidence.
                    value = prev
                    advanced = False
                else:
                    value = (min(self.required,count+1),
                             audio_stamp_ns,radar_stamp_ns)
                    advanced = True
            self.history[key] = value
            results[key] = RadarTemporalEvidence(
                value[0], advanced and value[0]>=self.required)
        return results
