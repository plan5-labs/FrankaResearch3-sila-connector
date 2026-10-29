import math

import pytest

from unitelabs.fr3_connector.io.errors import CommandRejected
from unitelabs.fr3_connector.io.limits import (
    JOINT_POSITION_MAX,
    READY_POSE,
    validate_dynamics_factor,
    validate_gripper_command,
    validate_joint_positions,
    validate_position_in_workspace,
)


def test_ready_pose_is_valid():
    assert validate_joint_positions(READY_POSE) == list(READY_POSE)


@pytest.mark.parametrize(
    ("positions", "message"),
    [
        ([0.0] * 6, "Expected 7"),
        ([*READY_POSE[:3], 0.0, *READY_POSE[4:]], "Joint 4"),
        ([JOINT_POSITION_MAX[0], *READY_POSE[1:]], "Joint 1"),
        ([math.nan, *READY_POSE[1:]], "not a finite"),
    ],
)
def test_invalid_joint_positions_are_rejected(positions, message):
    with pytest.raises(CommandRejected, match=message):
        validate_joint_positions(positions)


def test_dynamics_factor_is_capped():
    assert validate_dynamics_factor(0.1, 0.2) == 0.1
    for factor in (0.0, -0.1, 0.25, math.inf):
        with pytest.raises(CommandRejected):
            validate_dynamics_factor(factor, 0.2)


def test_workspace_envelope():
    validate_position_in_workspace([0.4, 0.0, 0.3], 0.0)
    with pytest.raises(CommandRejected, match="below"):
        validate_position_in_workspace([0.4, 0.0, 0.01], 0.05)
    with pytest.raises(CommandRejected, match="reach"):
        validate_position_in_workspace([1.0, 0.0, 0.3], 0.0)


def test_gripper_command_limits():
    validate_gripper_command(0.04, 0.05, 20.0)
    with pytest.raises(CommandRejected, match="width"):
        validate_gripper_command(0.09, 0.05)
    with pytest.raises(CommandRejected, match="speed"):
        validate_gripper_command(0.04, 0.2)
    with pytest.raises(CommandRejected, match="force"):
        validate_gripper_command(0.04, 0.05, 100.0)
