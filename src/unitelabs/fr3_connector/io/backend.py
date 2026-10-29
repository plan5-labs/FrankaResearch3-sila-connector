import asyncio
import collections.abc
import dataclasses

import typing_extensions as typing

ProgressCallback = collections.abc.Callable[[float, float | None], None]
"""Receives the completed fraction in [0, 1] and the estimated remaining seconds, if known."""


@dataclasses.dataclass(frozen=True)
class RobotState:
    """A snapshot of the arm state."""

    joint_positions: list[float]
    """Measured joint angles in radians, joint 1 to joint 7."""

    joint_velocities: list[float]
    """Measured joint velocities in radians per second."""

    end_effector_pose: list[float]
    """End-effector pose in the base frame as [x, y, z, qx, qy, qz, qw], position in metres."""

    robot_mode: str
    """Robot mode as reported by libfranka, e.g. Idle, Move, Guiding, Reflex or UserStopped."""

    has_errors: bool
    """Whether the robot reports errors that require error recovery before the next motion."""


@dataclasses.dataclass(frozen=True)
class GripperState:
    """A snapshot of the Franka Hand state."""

    width: float
    """Current finger opening in metres."""

    max_width: float
    """Maximum finger opening in metres, as measured by the last homing."""

    is_grasped: bool
    """Whether an object is currently held."""


class Backend(typing.Protocol):
    """
    Execute validated requests on a real or simulated FR3.

    Backends do not check configuration gates or limits; `Fr3Protocol` validates every request before delegating.
    Motion methods return when the target is reached and raise `MotionInterrupted` once `stop` is set.
    """

    async def open(self) -> None:
        """Connect without moving the robot."""

    async def close(self) -> None:
        """Disconnect without moving the robot."""

    async def read_state(self) -> RobotState:
        """Read the current arm state."""
        ...

    async def move_joints(
        self, target: list[float], dynamics_factor: float, stop: asyncio.Event, on_progress: ProgressCallback
    ) -> None:
        """Move to absolute joint angles in radians."""

    async def move_cartesian(
        self, target: list[float], dynamics_factor: float, stop: asyncio.Event, on_progress: ProgressCallback
    ) -> None:
        """Move the end effector to an absolute pose [x, y, z, qx, qy, qz, qw] in the base frame."""

    async def recover_from_errors(self) -> None:
        """Clear recoverable robot errors such as reflexes."""

    async def read_gripper(self) -> GripperState:
        """Read the current gripper state."""
        ...

    async def gripper_move(self, width: float, speed: float) -> bool:
        """Move the fingers to a width in metres; return whether the width was reached."""
        ...

    async def gripper_grasp(
        self, width: float, speed: float, force: float, epsilon_inner: float, epsilon_outer: float
    ) -> bool:
        """Close on an object; return whether its width is within the tolerances."""
        ...

    async def gripper_homing(self) -> bool:
        """Calibrate the finger range; return whether homing succeeded."""
        ...

    async def gripper_stop(self) -> None:
        """Abort the current gripper action."""
