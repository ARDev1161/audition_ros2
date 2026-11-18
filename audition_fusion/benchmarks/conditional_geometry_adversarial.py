#!/usr/bin/env python3
"""Deterministic adverse geometry audit of M4 conditional ray admission.

NOT a room-acoustics renderer. This intentionally exposes cases that the
planar two-window model cannot distinguish from direct-path observations.
A passing CI means the known blind spots were reproduced, NOT that any
claimed physical localization accuracy was validated.
"""
import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from evidence_buffer import DiagnosticEvidenceBuffer
from uncertainty import PlanarRay

SESSION = '1' * 32
SENSOR_X = (0., 1., 2., 0., 1., 2.)
STAMPS = tuple(1_000_000_000 + i * 300_000_000 for i in range(6))


def rays(first_target=(3., 2.), second_target=None, *,
         origin_shift=(0., 0.), measured_bearing_bias=(0., 0.)):
    """Generate distinct timestamped views of a target or virtual reflector."""
    if second_target is None:
        second_target = first_target
    samples = []
    for i, x in enumerate(SENSOR_X):
        target = first_target if i < 3 else second_target
        # Bearings can be biased together without appearing as excess random noise.
        dx = target[0] - x + measured_bearing_bias[0]
        dy = target[1] + measured_bearing_bias[1]
        length = math.hypot(dx, dy)
        assert length > 0
        samples.append(PlanarRay(
            STAMPS[i], (x + origin_shift[0], origin_shift[1]),
            (dx / length, dy / length), .001,
            (.01, 0., 0., .01)))
    return samples


def classify(samples):
    buffer = DiagnosticEvidenceBuffer()
    result = None
    for ray in samples:
        result = buffer.add(SESSION, 'stable-track-id', 1, ray)
    assert result is not None and result.evaluated
    assert not buffer.tracks, 'completed diagnostic window must not persist'
    return result.admitted


def main():
    # A stable, direct source is mathematically consistent.
    stable = classify(rays())
    # A source moving by two metres across disjoint windows must fail
    # the temporal-consensus check.
    moving = classify(rays(second_target=(5., 2.)))
    # A specular reflection with a stable image-source position is also
    # mathematically self-consistent; the model cannot identify it as a ghost.
    reflected_virtual = classify(rays(first_target=(3., -2.)))
    # If the robot poses share a common +1 m translation error, all bearing
    # intersections are translated together and remain internally consistent.
    common_slam_translation = classify(rays(origin_shift=(1., 0.)))
    report = {
        'schema_version': 1,
        'test_kind': 'deterministic_planar_geometry_only',
        'stable_direct_path_admitted': stable,
        'moving_source_admitted': moving,
        'reflection_virtual_source_admitted': reflected_virtual,
        'common_pose_bias_admitted': common_slam_translation,
        'physical_localization_validated': False,
        'observed_blind_spots': [
            'coherent_virtual_sources_from_reflections',
            'shared_robot_pose_translation_error',
        ],
    }
    print(json.dumps(report, sort_keys=True))
    if (not stable or moving or not reflected_virtual
            or not common_slam_translation):
        print('FAIL: M4 adverse-geometry behavior changed; review assumptions',
              file=sys.stderr)
        return 1
    print('PASS: stable and moving controls; known reflection/SLAM blind spots exposed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
