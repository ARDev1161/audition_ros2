"""M4 diagnostic admission using independent windows of calibrated planar rays.

No ROS output, no claim of metric localization, no navigation integration.
The caller must establish independent angular/pose covariance and direct-path
validity; shared SLAM and microphone extrinsic errors are not modeled here.
"""
from dataclasses import dataclass
import math

from association import CandidateTracker, Observation
from uncertainty import PlanarRay, conditional_planar_covariance
from stationarity import Fix, compare_disjoint_fixes


@dataclass(frozen=True)
class WindowEvidence:
    candidate: object
    covariance: object


@dataclass(frozen=True)
class WindowConsensus:
    first: WindowEvidence
    second: WindowEvidence
    stability: object


def _window(session, source_id, generation, rays):
    """Compute a candidate and conditional covariance from one bounded window."""
    if (not isinstance(session, str) or not isinstance(source_id, str)
            or not source_id or type(generation) is not int or generation <= 0
            or not isinstance(rays, (list, tuple)) or not 3 <= len(rays) <= 8):
        return None
    tracker = CandidateTracker()
    candidate = None
    for i, ray in enumerate(rays, 1):
        if not isinstance(ray, PlanarRay):
            return None
        if not tracker.begin_scene(session, i, ray.stamp_ns):
            return None
        obs = Observation(source_id, generation, ray.stamp_ns,
                          ray.origin, ray.direction)
        candidate = tracker.observe(obs)
        # An invalid/out-of-order ray must not silently disappear.
        if (source_id, generation) not in tracker.tracks or len(tracker.tracks[
                (source_id, generation)]) != i:
            return None
    if candidate is None or candidate.observations != len(rays):
        return None
    covariance = conditional_planar_covariance((candidate.x, candidate.y), rays)
    if covariance is None:
        return None
    return WindowEvidence(candidate, covariance)


def evaluate_disjoint_windows(session, source_id, generation, earlier, later):
    """Two independent windows must each pass triangulation and covariance.

    Return a diagnostic consensus only if they also agree in time and space.
    Caller must not reuse any observation between windows and must explicitly
    supply measured covariance; no zeros are fabricated for missing inputs.
    """
    a = _window(session, source_id, generation, earlier)
    b = _window(session, source_id, generation, later)
    if a is None or b is None:
        return None
    af = Fix(session, source_id, generation,
             a.candidate.first_stamp_ns, a.candidate.last_stamp_ns,
             a.candidate.x, a.candidate.y, a.covariance.major_std_m)
    bf = Fix(session, source_id, generation,
             b.candidate.first_stamp_ns, b.candidate.last_stamp_ns,
             b.candidate.x, b.candidate.y, b.covariance.major_std_m)
    stability = compare_disjoint_fixes(af, bf)
    return WindowConsensus(a, b, stability) if stability is not None else None
