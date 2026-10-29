import asyncio
import collections.abc
import contextlib
import math

from .backend import Backend, GripperState, ProgressCallback, RobotState
from .errors import CommandRejected, DeviceUnavailable, GripperFailed, OutcomeUnknown
from .franky_backend import FrankyBackend
from .kinematics import quaternion_to_matrix
from .limits import (
    JOINT_COUNT,
    READY_POSE,
    validate_dynamics_factor,
    validate_gripper_command,
    validate_joint_positions,
    validate_position_in_workspace,
)
from .simulation import SimulatedFr3

NAMED_POSITION_TOLERANCE = 0.01
"""Largest joint deviation in radians for the robot to count as being at a named position."""

GRIP_EMPTY_WIDTH = 0.001
"""Openings below this width in metres after a grip mean the fingers closed on nothing."""


def _ignore_progress(fraction: float, remaining: float | None) -> None:
    del fraction, remaining


class Fr3Protocol:
    """
    Validate and serialize FR3 requests, then delegate them to the simulated or real backend.

    Every gate lives here so that simulation and hardware enforce identical rules: configuration switches,
    joint limits, workspace envelope, dynamics cap, single active motion, and fail-closed handling of unknown
    outcomes. Stop requests bypass the motion guard so that they are never queued behind the motion they end.
    """

    def __init__(
        self,
        *,
        simulation_mode: bool = True,
        robot_host: str = "",
        realtime_config: str = "enforce",
        enable_motion: bool = False,
        enable_gripper: bool = False,
        default_dynamics_factor: float = 0.05,
        max_dynamics_factor: float = 0.2,
        gripper_speed: float = 0.05,
        gripper_force: float = 20.0,
        named_positions: dict[str, list[float]] | None = None,
        minimum_tcp_height: float = 0.0,
        simulation_time_scale: float = 1.0,
        simulated_object_width: float = 0.0,
        real_backend_factory: collections.abc.Callable[[], Backend] | None = None,
    ) -> None:
        if not 0.0 < max_dynamics_factor <= 1.0:
            msg = "max_dynamics_factor must be greater than 0 and at most 1."
            raise ValueError(msg)
        if not 0.0 < default_dynamics_factor <= max_dynamics_factor:
            msg = "default_dynamics_factor must be greater than 0 and at most max_dynamics_factor."
            raise ValueError(msg)
        try:
            validate_gripper_command(0.0, gripper_speed, gripper_force)
            positions = {"Home": list(READY_POSE)}
            for name, joints in (named_positions or {}).items():
                if not name.strip():
                    msg = "Named positions require a non-empty name."
                    raise CommandRejected(msg)
                positions[name] = validate_joint_positions(joints)
        except CommandRejected as error:
            msg = f"Invalid connector configuration: {error}"
            raise ValueError(msg) from error

        self.simulation_mode = simulation_mode
        self.robot_host = robot_host
        self.enable_motion = enable_motion
        self.enable_gripper = enable_gripper
        self.default_dynamics_factor = default_dynamics_factor
        self.max_dynamics_factor = max_dynamics_factor
        self.gripper_speed = gripper_speed
        self.gripper_force = gripper_force
        self.named_positions = positions
        self.minimum_tcp_height = minimum_tcp_height
        self.simulator = SimulatedFr3(time_scale=simulation_time_scale, object_width=simulated_object_width)
        self.control_outcome_unknown = False
        """Set when a real action lost its connection; real motion stays blocked until errors are recovered."""

        self._real_backend_factory = real_backend_factory or (
            lambda: FrankyBackend(robot_host, realtime_config=realtime_config, gripper_enabled=enable_gripper)
        )
        self._real: Backend | None = None
        self._stop: asyncio.Event | None = None
        self._gripper_busy = False
        self._switching = False

    @property
    def motion_active(self) -> bool:
        """Whether an arm motion is currently being executed."""

        return self._stop is not None

    @property
    def motion_enabled(self) -> bool:
        """Whether arm motion is permitted in the current backend."""

        return self.simulation_mode or self.enable_motion

    @property
    def gripper_enabled(self) -> bool:
        """Whether gripper actions are permitted in the current backend."""

        return self.simulation_mode or self.enable_gripper

    async def open(self) -> None:
        """
        Connect the configured backend without moving the robot.

        Raises:
          DeviceUnavailable: If real mode is configured but the robot cannot be reached.
        """

        if not self.simulation_mode and self._real is None:
            self._real = await self._connect_real()

    async def close(self) -> None:
        """Stop any active motion and disconnect."""

        await self.stop()
        if self._real is not None:
            real, self._real = self._real, None
            await real.close()

    async def set_simulation_mode(self, simulation_mode: bool) -> None:
        """
        Switch between the simulator and the physical robot while idle.

        Args:
          simulation_mode: True to use the simulator, False to connect to the configured robot.

        Raises:
          CommandRejected: If a motion or gripper action is active.
          DeviceUnavailable: If the physical robot cannot be reached; the previous mode is kept.
        """

        if simulation_mode == self.simulation_mode:
            if not simulation_mode and self._real is None:
                self._real = await self._connect_real()
            return
        if self.motion_active or self._gripper_busy or self._switching:
            msg = "Wait for the active motion or gripper action to finish before switching modes."
            raise CommandRejected(msg)

        self._switching = True
        try:
            if simulation_mode:
                real, self._real = self._real, None
                if real is not None:
                    await real.close()
            else:
                self._real = await self._connect_real()
            self.simulation_mode = simulation_mode
        finally:
            self._switching = False

    async def read_state(self) -> RobotState:
        """
        Read the current arm state from the active backend.

        Returns:
          The state snapshot.
        """

        return await self._backend().read_state()

    async def move_to_joint_positions(
        self, target: list[float], dynamics_factor: float, on_progress: ProgressCallback = _ignore_progress
    ) -> None:
        """
        Move to absolute joint angles.

        Args:
          target: Seven joint angles in radians.
          dynamics_factor: Fraction of the velocity, acceleration and jerk limits.
          on_progress: Receives progress updates.
        """

        target = validate_joint_positions(target)
        dynamics_factor = validate_dynamics_factor(dynamics_factor, self.max_dynamics_factor)
        async with self._motion() as (backend, _, stop):
            await backend.move_joints(target, dynamics_factor, stop, on_progress)

    async def move_joint(
        self, joint: int, delta: float, dynamics_factor: float, on_progress: ProgressCallback = _ignore_progress
    ) -> None:
        """
        Rotate a single joint relative to its current angle.

        Args:
          joint: Joint number from 1 to 7.
          delta: Signed angle in radians.
          dynamics_factor: Fraction of the velocity, acceleration and jerk limits.
          on_progress: Receives progress updates.

        Raises:
          CommandRejected: If the joint number is invalid or the resulting target is outside the limits.
        """

        if not 1 <= joint <= JOINT_COUNT:
            msg = f"Joint must be between 1 and {JOINT_COUNT}, received {joint}."
            raise CommandRejected(msg)
        if not math.isfinite(delta):
            msg = "Joint delta must be a finite number."
            raise CommandRejected(msg)
        dynamics_factor = validate_dynamics_factor(dynamics_factor, self.max_dynamics_factor)
        async with self._motion() as (backend, state, stop):
            target = list(state.joint_positions)
            target[joint - 1] += delta
            await backend.move_joints(validate_joint_positions(target), dynamics_factor, stop, on_progress)

    async def move_to_pose(
        self, pose: list[float], dynamics_factor: float, on_progress: ProgressCallback = _ignore_progress
    ) -> None:
        """
        Move the end effector to an absolute pose in the base frame.

        Args:
          pose: [x, y, z, qx, qy, qz, qw] with the position in metres.
          dynamics_factor: Fraction of the velocity, acceleration and jerk limits.
          on_progress: Receives progress updates.
        """

        target = self._validate_pose(pose)
        dynamics_factor = validate_dynamics_factor(dynamics_factor, self.max_dynamics_factor)
        async with self._motion() as (backend, _, stop):
            await backend.move_cartesian(target, dynamics_factor, stop, on_progress)

    async def move_linear(
        self, offset: list[float], dynamics_factor: float, on_progress: ProgressCallback = _ignore_progress
    ) -> None:
        """
        Translate the end effector along a straight line in the base frame, keeping its orientation.

        Args:
          offset: Translation (dx, dy, dz) in metres.
          dynamics_factor: Fraction of the velocity, acceleration and jerk limits.
          on_progress: Receives progress updates.

        Raises:
          CommandRejected: If the offset is invalid or the target leaves the workspace envelope.
        """

        if len(offset) != 3 or not all(math.isfinite(value) for value in offset):
            msg = "Offset must contain three finite values in metres."
            raise CommandRejected(msg)
        dynamics_factor = validate_dynamics_factor(dynamics_factor, self.max_dynamics_factor)
        async with self._motion() as (backend, state, stop):
            pose = list(state.end_effector_pose)
            target = [pose[0] + offset[0], pose[1] + offset[1], pose[2] + offset[2], *pose[3:]]
            await backend.move_cartesian(self._validate_pose(target), dynamics_factor, stop, on_progress)

    async def move_to_named_position(
        self, name: str, dynamics_factor: float | None = None, on_progress: ProgressCallback = _ignore_progress
    ) -> None:
        """
        Move in joint space to a configured named position.

        Args:
          name: Name of the position, e.g. Home.
          dynamics_factor: Fraction of the limits; the configured default if omitted.
          on_progress: Receives progress updates.

        Raises:
          CommandRejected: If the name is unknown.
        """

        if name not in self.named_positions:
            msg = f"Unknown position {name!r}; available: {', '.join(self.named_positions)}."
            raise CommandRejected(msg)
        await self.move_to_joint_positions(
            self.named_positions[name], dynamics_factor or self.default_dynamics_factor, on_progress
        )

    async def current_named_position(self) -> str:
        """
        Find the named position that matches the current joint angles.

        Returns:
          The position name, or an empty string if the robot is not at a named position.
        """

        joints = (await self.read_state()).joint_positions
        for name, target in self.named_positions.items():
            if max(abs(a - b) for a, b in zip(joints, target, strict=True)) <= NAMED_POSITION_TOLERANCE:
                return name
        return ""

    async def stop(self) -> None:
        """Request the active motion to decelerate and stop; does nothing when idle."""

        if self._stop is not None:
            self._stop.set()

    async def recover_from_errors(self) -> None:
        """
        Clear robot errors and the unknown-outcome block after the operator has inspected the robot.

        Raises:
          CommandRejected: If a motion is active.
        """

        if self.motion_active:
            msg = "Stop the active motion before recovering from errors."
            raise CommandRejected(msg)
        await self._backend().recover_from_errors()
        if not self.simulation_mode:
            self.control_outcome_unknown = False

    async def read_gripper(self) -> GripperState:
        """
        Read the gripper state from the active backend.

        Returns:
          The gripper state snapshot.
        """

        return await self._backend().read_gripper()

    async def gripper_move(self, width: float, speed: float) -> None:
        """
        Move the fingers to a width.

        Args:
          width: Target opening in metres.
          speed: Finger speed in metres per second.

        Raises:
          GripperFailed: If the gripper did not reach the width.
        """

        async with self._gripper() as backend:
            validate_gripper_command(width, speed, max_width=(await backend.read_gripper()).max_width)
            if not await backend.gripper_move(width, speed):
                msg = f"The gripper did not reach {width:.4f} m; something may be blocking the fingers."
                raise GripperFailed(msg)

    async def gripper_grasp(
        self, width: float, speed: float, force: float, epsilon_inner: float, epsilon_outer: float
    ) -> bool:
        """
        Grasp an object of known width.

        Args:
          width: Expected object width in metres.
          speed: Finger speed in metres per second.
          force: Grasping force in newtons.
          epsilon_inner: Accepted deviation below the expected width in metres.
          epsilon_outer: Accepted deviation above the expected width in metres.

        Returns:
          Whether an object within the tolerances is held.

        Raises:
          CommandRejected: If a tolerance is negative.
        """

        if not (epsilon_inner >= 0.0 and epsilon_outer >= 0.0):
            msg = "Grasp tolerances must be non-negative."
            raise CommandRejected(msg)
        async with self._gripper() as backend:
            validate_gripper_command(width, speed, force, max_width=(await backend.read_gripper()).max_width)
            return await backend.gripper_grasp(width, speed, force, epsilon_inner, epsilon_outer)

    async def grip(self) -> None:
        """
        Close on an object of unknown width with the configured speed and force.

        Raises:
          GripperFailed: If the fingers closed without holding an object.
        """

        async with self._gripper() as backend:
            max_width = (await backend.read_gripper()).max_width
            grasped = await backend.gripper_grasp(0.0, self.gripper_speed, self.gripper_force, 0.0, max_width)
            if not grasped or (await backend.read_gripper()).width < GRIP_EMPTY_WIDTH:
                msg = "The gripper closed without detecting an object."
                raise GripperFailed(msg)

    async def release(self) -> None:
        """Open the fingers fully with the configured speed."""

        async with self._gripper() as backend:
            max_width = (await backend.read_gripper()).max_width
            if not await backend.gripper_move(max_width, self.gripper_speed):
                msg = "The gripper did not open fully."
                raise GripperFailed(msg)

    async def gripper_homing(self) -> None:
        """
        Calibrate the finger range by opening and closing fully.

        Raises:
          GripperFailed: If homing did not succeed.
        """

        async with self._gripper() as backend:
            if not await backend.gripper_homing():
                msg = "Gripper homing did not succeed."
                raise GripperFailed(msg)

    async def gripper_stop(self) -> None:
        """
        Abort the current gripper action; bypasses the gripper guard.

        Raises:
          CommandRejected: If the gripper is disabled in the current mode.
        """

        if not self.gripper_enabled:
            msg = "Gripper actions are disabled for the physical robot. Set enable_gripper after on-site review."
            raise CommandRejected(msg)
        await self._backend().gripper_stop()

    def _backend(self) -> Backend:
        if self.simulation_mode:
            return self.simulator
        if self._real is None:
            msg = "The robot connection is closed; restart real mode through the SimulationController."
            raise DeviceUnavailable(msg)
        return self._real

    async def _connect_real(self) -> Backend:
        if not self.robot_host:
            msg = "Configure robot_host with the FCI address of the FR3 control unit before using real mode."
            raise DeviceUnavailable(msg)
        backend = self._real_backend_factory()
        await backend.open()
        return backend

    def _validate_pose(self, pose: list[float]) -> list[float]:
        if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
            msg = "Pose must contain seven finite values: x, y, z, qx, qy, qz, qw."
            raise CommandRejected(msg)
        validate_position_in_workspace(pose[:3], self.minimum_tcp_height)
        try:
            quaternion_to_matrix(pose[3:])
        except ValueError as error:
            raise CommandRejected(str(error)) from error
        norm = math.sqrt(sum(value * value for value in pose[3:]))
        return [*pose[:3], *(value / norm for value in pose[3:])]

    @contextlib.asynccontextmanager
    async def _motion(self) -> collections.abc.AsyncIterator[tuple[Backend, RobotState, asyncio.Event]]:
        if not self.simulation_mode and not self.enable_motion:
            msg = "Motion is disabled for the physical robot. Set enable_motion only after on-site review."
            raise CommandRejected(msg)
        if not self.simulation_mode and self.control_outcome_unknown:
            msg = "A previous action has an unknown outcome. Inspect the robot, then call RecoverFromErrors."
            raise CommandRejected(msg)
        if self.motion_active or self._switching:
            msg = "Another motion is in progress; wait for it to finish or call Stop."
            raise CommandRejected(msg)

        backend = self._backend()
        stop = self._stop = asyncio.Event()
        try:
            state = await backend.read_state()
            if state.has_errors:
                msg = f"The robot reports errors (mode {state.robot_mode}); call RecoverFromErrors first."
                raise CommandRejected(msg)
            try:
                yield backend, state, stop
            except (OutcomeUnknown, DeviceUnavailable) as error:
                if self.simulation_mode:
                    raise
                self.control_outcome_unknown = True
                if isinstance(error, OutcomeUnknown):
                    raise
                msg = f"The connection failed during a motion request ({error}); its outcome is unknown."
                raise OutcomeUnknown(msg) from error
        finally:
            self._stop = None

    @contextlib.asynccontextmanager
    async def _gripper(self) -> collections.abc.AsyncIterator[Backend]:
        if not self.gripper_enabled:
            msg = "Gripper actions are disabled for the physical robot. Set enable_gripper after on-site review."
            raise CommandRejected(msg)
        if self._gripper_busy or self._switching:
            msg = "Another gripper action is in progress."
            raise CommandRejected(msg)

        backend = self._backend()
        self._gripper_busy = True
        try:
            yield backend
        finally:
            self._gripper_busy = False
