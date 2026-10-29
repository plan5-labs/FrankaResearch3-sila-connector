"""Franka Research 3 and Franka Hand limits from the published FR3 interface specification."""

import math

from .errors import CommandRejected

JOINT_COUNT = 7

JOINT_POSITION_MIN = (-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159)
"""Lower joint position limits in radians."""

JOINT_POSITION_MAX = (2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159)
"""Upper joint position limits in radians."""

JOINT_VELOCITY_MAX = (2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26)
"""Joint velocity limits in radians per second."""

JOINT_ACCELERATION_MAX = (10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0)
"""Joint acceleration limits in radians per square second."""

TRANSLATION_VELOCITY_MAX = 3.0
"""End-effector translation velocity limit in metres per second."""

TRANSLATION_ACCELERATION_MAX = 9.0
"""End-effector translation acceleration limit in metres per square second."""

ROTATION_VELOCITY_MAX = 2.5
"""End-effector rotation velocity limit in radians per second."""

ROTATION_ACCELERATION_MAX = 17.0
"""End-effector rotation acceleration limit in radians per square second."""

JOINT_LIMIT_MARGIN = 0.01
"""Keep commanded targets this many radians inside the hardware limits."""

REACH = 0.855
"""Maximum reach in metres, measured from the shoulder (joint 2) axis."""

SHOULDER_HEIGHT = 0.333
"""Height of the shoulder axis above the base frame origin in metres."""

READY_POSE = (0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4)
"""Franka's conventional "ready" configuration, used as the default home position."""

HAND_MAX_WIDTH = 0.08
"""Franka Hand maximum finger opening in metres."""

HAND_MAX_SPEED = 0.1
"""Franka Hand maximum finger opening speed in metres per second."""

HAND_MAX_FORCE = 70.0
"""Franka Hand continuous grasping force limit in newtons."""


def validate_joint_positions(positions: list[float] | tuple[float, ...]) -> list[float]:
    """
    Check a seven-axis joint target against the FR3 limits.

    Args:
      positions: Joint angles in radians, from joint 1 to joint 7.

    Returns:
      The validated joint angles as floats.

    Raises:
      CommandRejected: If the count is wrong, a value is not finite, or a joint is outside its limits.
    """

    if len(positions) != JOINT_COUNT:
        msg = f"Expected {JOINT_COUNT} joint positions, received {len(positions)}."
        raise CommandRejected(msg)

    values = [float(value) for value in positions]
    for index, value in enumerate(values):
        if not math.isfinite(value):
            msg = f"Joint {index + 1} target is not a finite number."
            raise CommandRejected(msg)
        low = JOINT_POSITION_MIN[index] + JOINT_LIMIT_MARGIN
        high = JOINT_POSITION_MAX[index] - JOINT_LIMIT_MARGIN
        if not low <= value <= high:
            msg = f"Joint {index + 1} target {value:.4f} rad is outside the permitted range [{low:.4f}, {high:.4f}]."
            raise CommandRejected(msg)

    return values


def validate_dynamics_factor(factor: float, maximum: float) -> float:
    """
    Check a relative dynamics factor against the configured cap.

    Args:
      factor: Requested fraction of the velocity, acceleration and jerk limits.
      maximum: Largest fraction permitted by the connector configuration.

    Returns:
      The validated factor.

    Raises:
      CommandRejected: If the factor is not in (0, maximum].
    """

    if not math.isfinite(factor) or not 0.0 < factor <= maximum:
        msg = f"Relative dynamics factor must be greater than 0 and at most {maximum}, received {factor}."
        raise CommandRejected(msg)

    return float(factor)


def validate_position_in_workspace(position: list[float] | tuple[float, ...], minimum_z: float) -> None:
    """
    Reject end-effector targets that are obviously out of reach or below the configured floor.

    This is a coarse envelope check; it does not model collisions with the robot or its surroundings.

    Args:
      position: Target position (x, y, z) in metres in the robot base frame.
      minimum_z: Lowest permitted target height in metres.

    Raises:
      CommandRejected: If the target is not finite, beyond the reach envelope, or below the floor.
    """

    x, y, z = (float(value) for value in position)
    if not all(math.isfinite(value) for value in (x, y, z)):
        msg = "Target position must contain finite numbers."
        raise CommandRejected(msg)
    if z < minimum_z:
        msg = f"Target height {z:.3f} m is below the configured minimum of {minimum_z:.3f} m."
        raise CommandRejected(msg)
    if math.dist((x, y, z), (0.0, 0.0, SHOULDER_HEIGHT)) > REACH:
        msg = f"Target ({x:.3f}, {y:.3f}, {z:.3f}) m is outside the {REACH} m reach envelope."
        raise CommandRejected(msg)


def validate_gripper_command(
    width: float, speed: float, force: float | None = None, max_width: float = HAND_MAX_WIDTH
) -> None:
    """
    Check Franka Hand command arguments.

    Args:
      width: Target finger opening in metres.
      speed: Finger speed in metres per second.
      force: Grasping force in newtons, if the command grasps.
      max_width: Maximum opening reported by the gripper.

    Raises:
      CommandRejected: If a value is outside the Franka Hand limits.
    """

    if not math.isfinite(width) or not 0.0 <= width <= max_width:
        msg = f"Gripper width must be between 0 and {max_width:.4f} m, received {width}."
        raise CommandRejected(msg)
    if not math.isfinite(speed) or not 0.0 < speed <= HAND_MAX_SPEED:
        msg = f"Gripper speed must be greater than 0 and at most {HAND_MAX_SPEED} m/s, received {speed}."
        raise CommandRejected(msg)
    if force is not None and (not math.isfinite(force) or not 0.0 < force <= HAND_MAX_FORCE):
        msg = f"Grasp force must be greater than 0 and at most {HAND_MAX_FORCE} N, received {force}."
        raise CommandRejected(msg)
