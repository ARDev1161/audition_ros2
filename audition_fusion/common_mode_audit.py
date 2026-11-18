"""Deterministic geometry-only common-mode error audit for M4.

Use the actual two-window candidate pipeline and independent synthetic truth.
This is a *negative* diagnostic: conditional covariance cannot detect a
shared position bias or a geometrically coherent reflection image.
"""
import math

from diagnostic_pipeline import evaluate_disjoint_windows
from uncertainty import PlanarRay

SESSION = "1" * 32
SOURCE = "synthetic"
TRUE_SOURCE = (3.0, 2.0)
SENSOR_X = (0.0, 1.0, 2.0, 0.0, 1.0, 2.0)
STAMPS_NS = tuple(1_000_000_000 + i * 300_000_000 for i in range(6))


def measured_rays(*, shared_pose_bias=(0.0, 0.0), apparent_source=TRUE_SOURCE):
    """Physical microphone bearings; only the reported poses are biased."""
    rays = []
    for index, x in enumerate(SENSOR_X):
        dx, dy = apparent_source[0] - x, apparent_source[1]
        norm = math.hypot(dx, dy)
        rays.append(PlanarRay(
            STAMPS_NS[index],
            (x + shared_pose_bias[0], shared_pose_bias[1]),
            (dx / norm, dy / norm),
            0.001, (0.01, 0.0, 0.0, 0.01)))
    return rays


def evaluate_case(*, bias=(0.0, 0.0), apparent_source=TRUE_SOURCE):
    rays = measured_rays(shared_pose_bias=bias, apparent_source=apparent_source)
    consensus = evaluate_disjoint_windows(
        SESSION, SOURCE, 1, rays[:3], rays[3:])
    if consensus is None:
        return {"admitted": False, "position_error_m": None,
                "conditional_major_std_m": None,
                "error_to_conditional_std_ratio": None}
    estimate = consensus.stability
    error = math.hypot(estimate.x - TRUE_SOURCE[0],
                       estimate.y - TRUE_SOURCE[1])
    # Take the worse conditional standard deviation, which is still
    # insufficient for shared systematic error or a virtual image.
    major = max(consensus.first.covariance.major_std_m,
                consensus.second.covariance.major_std_m)
    return {"admitted": True,
            "position_error_m": error,
            "conditional_major_std_m": major,
            "error_to_conditional_std_ratio": error / major if major > 0 else None}


def report():
    cases = {
        "direct_path": evaluate_case(),
        "shared_pose_bias_0_5m": evaluate_case(bias=(0.5, 0.0)),
        "shared_pose_bias_1m": evaluate_case(bias=(1.0, 0.0)),
        "coherent_reflection_image": evaluate_case(
            apparent_source=(3.0, -2.0)),
    }
    return {
        "schema_version": 1,
        "test_kind": "deterministic_planar_conditional_geometry",
        "cases": cases,
        "independent_truth": list(TRUE_SOURCE),
        "physical_validation_claim": False,
        "metric_position_published": False,
        "caveat": ("An admitted conditional ray intersection is not an "
                   "independently validated physical emitter position."),
    }


def assert_expected(result):
    cases = result["cases"]
    assert all(c["admitted"] for c in cases.values())
    assert cases["direct_path"]["position_error_m"] < 1e-6
    assert abs(cases["shared_pose_bias_0_5m"]["position_error_m"] - 0.5) < 1e-6
    assert abs(cases["shared_pose_bias_1m"]["position_error_m"] - 1.0) < 1e-6
    assert abs(cases["coherent_reflection_image"]["position_error_m"] - 4.0) < 1e-6
    assert (cases["shared_pose_bias_1m"]["conditional_major_std_m"]
            < cases["shared_pose_bias_1m"]["position_error_m"])
    assert not result["physical_validation_claim"]
    assert not result["metric_position_published"]
