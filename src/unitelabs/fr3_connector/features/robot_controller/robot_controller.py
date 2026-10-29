import dataclasses
import datetime

import typing_extensions as typing

from unitelabs.cdk import sila

from ...io import Fr3Protocol
from ...io.backend import ProgressCallback
from ...io.limits import JOINT_COUNT, JOINT_POSITION_MAX, JOINT_POSITION_MIN

Radian = typing.Annotated[
    float, sila.constraints.Unit(label="rad", components=[sila.constraints.UnitComponent("Dimensionless")])
]
Metre = typing.Annotated[float, sila.constraints.Unit(label="m", components=[sila.constraints.UnitComponent("Meter")])]
JointVector = typing.Annotated[list[float], sila.constraints.ElementCount(JOINT_COUNT)]
DynamicsFactor = typing.Annotated[float, sila.constraints.MinimalExclusive(0.0), sila.constraints.MaximalInclusive(1.0)]
JointNumber = typing.Annotated[int, sila.constraints.MinimalInclusive(1), sila.constraints.MaximalInclusive(7)]


@dataclasses.dataclass
class Pose(sila.CustomDataType):
    """
    End-effector pose in the robot base frame.

    Attributes:
      X: Position along the base X axis in metres.
      Y: Position along the base Y axis in metres.
      Z: Position along the base Z axis in metres.
      Qx: Orientation quaternion X component.
      Qy: Orientation quaternion Y component.
      Qz: Orientation quaternion Z component.
      Qw: Orientation quaternion W (scalar) component.
    """

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @classmethod
    def from_list(cls, values: list[float]) -> "Pose":
        """
        Build a pose from [x, y, z, qx, qy, qz, qw].

        Args:
          values: Position in metres followed by the quaternion.

        Returns:
          The pose structure.
        """

        return cls(*values)

    def to_list(self) -> list[float]:
        """
        Flatten the pose.

        Returns:
          [x, y, z, qx, qy, qz, qw].
        """

        return [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]


def _reporter(status: sila.Status) -> ProgressCallback:
    def report(fraction: float, remaining: float | None) -> None:
        status.update(
            progress=fraction,
            remaining_time=datetime.timedelta(seconds=remaining) if remaining is not None else None,
        )

    return report


class RobotController(sila.Feature):
    """
    Read the state of a Franka Research 3 arm and move it in joint or Cartesian space.

    Joint angles are in radians from joint 1 (base) to joint 7 (flange). Poses refer to the Franka Hand TCP in the
    robot base frame. Every motion takes a relative dynamics factor that scales the velocity, acceleration and jerk
    limits; the connector rejects factors above its configured maximum. Only one motion runs at a time; use Stop
    to end it early.
    """

    def __init__(self, protocol: Fr3Protocol) -> None:
        super().__init__(originator="io.unitelabs", category="robot", version="1.0", maturity_level="Draft")
        self.protocol = protocol

    @sila.UnobservableProperty()
    async def get_joint_positions(self) -> JointVector:
        """
        Read the seven measured joint angles in radians.

        Raises:
          DeviceUnavailable: If the robot state cannot be read.
        """

        return (await self.protocol.read_state()).joint_positions

    @sila.UnobservableProperty()
    async def get_joint_velocities(self) -> JointVector:
        """
        Read the seven measured joint velocities in radians per second.

        Raises:
          DeviceUnavailable: If the robot state cannot be read.
        """

        return (await self.protocol.read_state()).joint_velocities

    @sila.UnobservableProperty()
    async def get_end_effector_pose(self) -> Pose:
        """
        Read the end-effector pose in the robot base frame.

        Raises:
          DeviceUnavailable: If the robot state cannot be read.
        """

        return Pose.from_list((await self.protocol.read_state()).end_effector_pose)

    @sila.UnobservableProperty()
    async def get_robot_mode(self) -> str:
        """
        Read the robot mode: Idle, Move, Guiding, Reflex, UserStopped, AutomaticErrorRecovery or Other.

        Raises:
          DeviceUnavailable: If the robot state cannot be read.
        """

        return (await self.protocol.read_state()).robot_mode

    @sila.UnobservableProperty()
    async def get_has_errors(self) -> bool:
        """
        Report whether the robot has errors that require RecoverFromErrors before the next motion.

        Raises:
          DeviceUnavailable: If the robot state cannot be read.
        """

        return (await self.protocol.read_state()).has_errors

    @sila.UnobservableProperty()
    async def get_control_outcome_unknown(self) -> bool:
        """Report whether a real motion lost its connection; motion stays blocked until RecoverFromErrors."""

        return self.protocol.control_outcome_unknown

    @sila.UnobservableProperty()
    async def get_motion_enabled(self) -> bool:
        """Report whether arm motion is permitted in the current backend."""

        return self.protocol.motion_enabled

    @sila.UnobservableProperty()
    async def get_motion_active(self) -> bool:
        """Report whether a motion is currently being executed."""

        return self.protocol.motion_active

    @sila.UnobservableProperty()
    async def get_max_dynamics_factor(self) -> float:
        """Read the largest relative dynamics factor the connector accepts."""

        return self.protocol.max_dynamics_factor

    @sila.UnobservableProperty()
    async def get_lower_joint_limits(self) -> JointVector:
        """Read the lower joint position limits in radians."""

        return list(JOINT_POSITION_MIN)

    @sila.UnobservableProperty()
    async def get_upper_joint_limits(self) -> JointVector:
        """Read the upper joint position limits in radians."""

        return list(JOINT_POSITION_MAX)

    @sila.ObservableCommand()
    async def move_to_joint_positions(
        self, joint_positions: JointVector, relative_dynamics_factor: DynamicsFactor, *, status: sila.Status
    ) -> None:
        """
        Move to absolute joint angles with a synchronized point-to-point motion.

        Args:
          JointPositions: Seven target joint angles in radians, inside the joint limits.
          RelativeDynamicsFactor: Fraction of the velocity, acceleration and jerk limits.

        Raises:
          CommandRejected: If disabled, invalid, outside the limits or another motion is active.
          DeviceUnavailable: If the robot state cannot be read.
          MotionInterrupted: If Stop ended the motion before the target was reached.
          MotionFailed: If the robot aborted the motion; inspect it and call RecoverFromErrors.
          OutcomeUnknown: If the connection was lost during the motion. Do not retry blindly.
        """

        await self.protocol.move_to_joint_positions(joint_positions, relative_dynamics_factor, _reporter(status))

    @sila.ObservableCommand()
    async def move_joint(
        self,
        joint: JointNumber,
        delta: Radian,
        relative_dynamics_factor: DynamicsFactor,
        *,
        status: sila.Status,
    ) -> None:
        """
        Rotate one joint relative to its current angle while holding the others.

        Args:
          Joint: Joint number from 1 (base) to 7 (flange).
          Delta: Signed rotation in radians; the resulting angle must stay inside the joint limits.
          RelativeDynamicsFactor: Fraction of the velocity, acceleration and jerk limits.

        Raises:
          CommandRejected: If disabled, invalid, outside the limits or another motion is active.
          DeviceUnavailable: If the robot state cannot be read.
          MotionInterrupted: If Stop ended the motion before the target was reached.
          MotionFailed: If the robot aborted the motion; inspect it and call RecoverFromErrors.
          OutcomeUnknown: If the connection was lost during the motion. Do not retry blindly.
        """

        await self.protocol.move_joint(joint, delta, relative_dynamics_factor, _reporter(status))

    @sila.ObservableCommand()
    async def move_to_pose(self, pose: Pose, relative_dynamics_factor: DynamicsFactor, *, status: sila.Status) -> None:
        """
        Move the end effector to an absolute pose along a Cartesian path.

        Args:
          Pose: Target pose in the robot base frame; the quaternion is normalized.
          RelativeDynamicsFactor: Fraction of the velocity, acceleration and jerk limits.

        Raises:
          CommandRejected: If disabled, invalid, outside the workspace envelope, unreachable or another motion is
            active.
          DeviceUnavailable: If the robot state cannot be read.
          MotionInterrupted: If Stop ended the motion before the target was reached.
          MotionFailed: If the robot aborted the motion; inspect it and call RecoverFromErrors.
          OutcomeUnknown: If the connection was lost during the motion. Do not retry blindly.
        """

        await self.protocol.move_to_pose(pose.to_list(), relative_dynamics_factor, _reporter(status))

    @sila.ObservableCommand()
    async def move_linear(
        self,
        dx: Metre,
        dy: Metre,
        dz: Metre,
        relative_dynamics_factor: DynamicsFactor,
        *,
        status: sila.Status,
    ) -> None:
        """
        Translate the end effector along a straight line in the base frame, keeping its orientation.

        Args:
          Dx: Translation along the base X axis in metres.
          Dy: Translation along the base Y axis in metres.
          Dz: Translation along the base Z axis in metres.
          RelativeDynamicsFactor: Fraction of the velocity, acceleration and jerk limits.

        Raises:
          CommandRejected: If disabled, invalid, outside the workspace envelope, unreachable or another motion is
            active.
          DeviceUnavailable: If the robot state cannot be read.
          MotionInterrupted: If Stop ended the motion before the target was reached.
          MotionFailed: If the robot aborted the motion; inspect it and call RecoverFromErrors.
          OutcomeUnknown: If the connection was lost during the motion. Do not retry blindly.
        """

        await self.protocol.move_linear([dx, dy, dz], relative_dynamics_factor, _reporter(status))

    @sila.UnobservableCommand()
    async def stop(self) -> None:
        """Decelerate and end the active motion; the motion command then fails with MotionInterrupted."""

        await self.protocol.stop()

    @sila.UnobservableCommand()
    async def recover_from_errors(self) -> None:
        """
        Clear recoverable robot errors and the unknown-outcome block after inspecting the robot.

        Raises:
          CommandRejected: If a motion is active.
          DeviceUnavailable: If the robot is not connected.
          MotionFailed: If the robot rejects error recovery.
        """

        await self.protocol.recover_from_errors()
