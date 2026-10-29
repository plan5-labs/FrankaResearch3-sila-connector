import math

import numpy as np
import pytest

from unitelabs.fr3_connector.io import kinematics
from unitelabs.fr3_connector.io.limits import READY_POSE


def test_ready_pose_matches_published_franka_hand_tcp():
    pose = kinematics.matrix_to_pose(kinematics.forward_kinematics(READY_POSE))
    assert pose[:3] == pytest.approx([0.3069, 0.0, 0.4869], abs=1e-4)
    # The hand points straight down: a half turn about the base X axis.
    assert pose[3:] == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-9)


def test_jacobian_matches_finite_differences():
    q = np.array(READY_POSE) + 0.1
    eps = 1e-6
    numeric = np.zeros((6, 7))
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = eps
        moved, origin = kinematics.forward_kinematics(q + dq), kinematics.forward_kinematics(q)
        numeric[:, i] = kinematics.pose_error(moved, origin) / eps
    assert np.abs(kinematics.jacobian(q) - numeric).max() < 1e-5


@pytest.mark.parametrize("offset", [(0.05, -0.1, -0.1), (-0.1, 0.2, 0.05), (0.0, 0.0, 0.0)])
def test_inverse_kinematics_round_trip(offset):
    target = kinematics.forward_kinematics(READY_POSE)
    target[:3, 3] += offset
    solution = kinematics.inverse_kinematics(target, READY_POSE)
    assert solution is not None
    assert np.abs(kinematics.pose_error(target, kinematics.forward_kinematics(solution))).max() < 1e-4


def test_inverse_kinematics_reports_unreachable_targets():
    target = kinematics.forward_kinematics(READY_POSE)
    target[:3, 3] = [1.2, 0.0, 0.3]
    assert kinematics.inverse_kinematics(target, READY_POSE) is None


def test_quaternion_round_trip_and_sign():
    rotation = kinematics.quaternion_to_matrix([0.3, -0.2, 0.5, -0.7])
    quaternion = kinematics.matrix_to_quaternion(rotation)
    assert quaternion[3] >= 0
    assert np.allclose(kinematics.quaternion_to_matrix(quaternion), rotation)
    with pytest.raises(ValueError):
        kinematics.quaternion_to_matrix([0, 0, 0, 0])


def test_rotation_vector_handles_half_turns():
    assert np.linalg.norm(kinematics.rotation_vector(np.diag([1.0, -1.0, -1.0]))) == pytest.approx(math.pi)
    assert np.allclose(kinematics.rotation_vector(np.eye(3)), 0)


def test_linear_path_stays_on_a_line():
    start = list(READY_POSE)
    target = kinematics.forward_kinematics(start)
    target[:3, 3] += [0.0, 0.15, -0.1]
    path = kinematics.plan_linear_path(start, target, 20)
    assert path is not None and len(path) == 20
    origin = kinematics.forward_kinematics(start)[:3, 3]
    direction = (target[:3, 3] - origin) / np.linalg.norm(target[:3, 3] - origin)
    for q in path:
        offset = kinematics.forward_kinematics(q)[:3, 3] - origin
        assert np.linalg.norm(offset - np.dot(offset, direction) * direction) < 1e-4


def test_minimum_jerk_duration_respects_limits():
    duration = kinematics.minimum_jerk_duration(1.0, 0.5, 2.0)
    assert 1.875 * 1.0 / duration <= 0.5 + 1e-9
    assert kinematics.MINIMUM_JERK_PEAK_ACCELERATION * 1.0 / duration**2 <= 2.0 + 1e-9
    assert kinematics.minimum_jerk_duration(0.0, 1.0, 1.0) == 0.0
    assert kinematics.minimum_jerk(0.0) == 0.0
    assert kinematics.minimum_jerk(0.5) == pytest.approx(0.5)
    assert kinematics.minimum_jerk(1.0) == pytest.approx(1.0)
