"""Bounded M4 cross-modal temporal corroboration, not identity verification.

A diagnostic AV pair is temporally corroborated only after independently new
audio AND visual observations, with bounded spacing, no competing assignment,
and unchanged stream incarnation and source generations. No probability or
physical speaker identity is inferred.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Confirmation:
    count: int
    confirmed: bool


class TemporalPairGate:
    def __init__(self, *, confirmations=3, minimum_spacing_ns=100_000_000,
                 maximum_gap_ns=600_000_000, maximum_pairs=16):
        if (not 2 <= confirmations <= 10
                or not 0 < minimum_spacing_ns < maximum_gap_ns
                or not 1 <= maximum_pairs <= 64):
            raise ValueError("invalid temporal AV confirmation limits")
        self.confirmations = confirmations
        self.minimum_spacing_ns = minimum_spacing_ns
        self.maximum_gap_ns = maximum_gap_ns
        self.maximum_pairs = maximum_pairs
        self.reset()

    def reset(self):
        self.epochs = None
        self.history = {}
        self.last_clock_ns = None

    @staticmethod
    def key(pair):
        return (pair.audio.identity, pair.audio.generation,
                pair.visual.identity, pair.visual.generation)

    def observe(self, pairs, audio_session, visual_session, clock_ns):
        if (not audio_session or not visual_session or clock_ns <= 0
                or len(pairs) > self.maximum_pairs):
            self.reset()
            return {}
        epochs = (audio_session, visual_session)
        if (self.epochs != epochs or
                (self.last_clock_ns is not None and clock_ns < self.last_clock_ns)):
            self.reset()
        self.epochs = epochs
        self.last_clock_ns = clock_ns

        # Any ambiguous edge invalidates evidence for its endpoint, including
        # a seemingly unique competitor. This is deliberately conservative.
        ambiguous_audio = {self.key(p)[:2] for p in pairs if not p.unique_match}
        ambiguous_visual = {self.key(p)[2:] for p in pairs if not p.unique_match}
        eligible = {}
        for pair in pairs:
            key = self.key(pair)
            if (not pair.unique_match
                    or key[:2] in ambiguous_audio or key[2:] in ambiguous_visual
                    or pair.audio.stamp_ns <= 0 or pair.visual.stamp_ns <= 0):
                continue
            if key in eligible:
                self.reset()
                return {}
            eligible[key] = pair
        self.history = {k: v for k, v in self.history.items() if k in eligible}
        results = {}
        for key, pair in eligible.items():
            a, v = pair.audio.stamp_ns, pair.visual.stamp_ns
            prev = self.history.get(key)
            if prev is None:
                value = (1, a, v)
            else:
                count, a0, v0 = prev
                da, dv = a-a0, v-v0
                if (da < 0 or dv < 0 or
                        da > self.maximum_gap_ns or dv > self.maximum_gap_ns):
                    value = (1, a, v)
                elif da == 0 or dv == 0:
                    # Repeated pairing of the same observation does not
                    # contribute independent evidence.
                    value = prev
                elif min(da, dv) < self.minimum_spacing_ns:
                    value = prev
                else:
                    value = (min(self.confirmations, count+1), a, v)
            self.history[key] = value
            results[key] = Confirmation(value[0], value[0] >= self.confirmations)
        return results
