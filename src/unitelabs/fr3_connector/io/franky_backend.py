import asyncio
import collections.abc
import math
import types

import typing_extensions as typing

from .backend import GripperState, ProgressCallback, RobotState
from .errors import DeviceUnavailable, GripperFailed, MotionFailed, MotionInterrupted, OutcomeUnknown

if typing.TYPE_CHECKING:
    import franky

T = typing.TypeVar("T")

INSTALL_HINT = (
    "Install the optional robot dependencies with `uv sync --extra robot` on the Linux workstation that is "
    "connected to the FR3 control unit."
)


class FrankyBackend:
    """
    Control a physical FR3 through the Franka Control Interface (FCI) using franky.

    franky plans time-optimal trajectories with Ruckig and runs the 1 kHz libfranka control loop in its own thread,
    so FCI requires a real-time Linux kernel on this host unless `realtime_config` is "ignore". All blocking franky
    calls run in worker threads; the event loop is never blocked.
    """

    def __init__(
        self,
        host: str,
        *,
        realtime_config: str = "enforce",
        gripper_enabled: bool = False,
        poll_interval: float = 0.02,
    ) -> None:
        self.host = host
        self.realtime_config = realtime_config
        self.gripper_enabled = gripper_enabled
        self.poll_interval = poll_interval
        self._franky: types.ModuleType | None = None
        self._robot: franky.Robot | None = None
        self._gripper: franky.Gripper | None = None

    async def open(self) -> None:
        """
        Connect to the robot and, if enabled, the gripper without commanding motion.

        Raises:
          DeviceUnavailable: If franky is missing or the FCI connection fails.
        """

        if not self.host:
            msg = "Configure robot_host with the FCI address of the FR3 control unit before using real mode."
            raise DeviceUnavailable(msg)
        try:
            import franky
        except ImportError as error:
            msg = f"The franky-control package is not available. {INSTALL_HINT}"
            raise DeviceUnavailable(msg) from error

        self._franky = franky
        realtime = franky.RealtimeConfig.Ignore if self.realtime_config == "ignore" else franky.RealtimeConfig.Enforce
        try:
            self._robot = await asyncio.to_thread(franky.Robot, self.host, realtime_config=realtime)
            if self.gripper_enabled:
                self._gripper = await asyncio.to_thread(franky.Gripper, self.host)
        except self._errors() as error:
            self._robot = self._gripper = None
            msg = f"Cannot connect to the FR3 at {self.host}: {error}"
            raise DeviceUnavailable(msg) from error

    async def close(self) -> None:
        """Release the franky handles; libfranka closes its connections when they are garbage collected."""

        self._robot = self._gripper = None

    async def read_state(self) -> RobotState:
        """
        Read the measured arm state.

        Returns:
          Joint state, end-effector pose, robot mode and error flag.

        Raises:
          DeviceUnavailable: If the robot is not connected or the state cannot be read.
        """

        robot = self._require_robot()

        def read() -> RobotState:
            state = robot.state
            pose = robot.current_pose.end_effector_pose
            quaternion = [float(value) for value in pose.quaternion]
            if quaternion[3] < 0:
                quaternion = [-value for value in quaternion]
            return RobotState(
                joint_positions=[float(value) for value in state.q],
                joint_velocities=[float(value) for value in state.dq],
                end_effector_pose=[*(float(value) for value in pose.translation), *quaternion],
                robot_mode=getattr(state.robot_mode, "name", str(state.robot_mode)),
                has_errors=bool(robot.has_errors),
            )

        return await self._call(read)

    async def move_joints(
        self, target: list[float], dynamics_factor: float, stop: asyncio.Event, on_progress: ProgressCallback
    ) -> None:
        """
        Execute a joint-space point-to-point motion.

        Args:
          target: Absolute joint angles in radians.
          dynamics_factor: Fraction of the velocity, acceleration and jerk limits.
          stop: Set to decelerate and end the motion early.
          on_progress: Receives progress updates.
        """

        franky = self._require_franky()
        robot = self._require_robot()

        def remaining() -> float:
            return max(abs(a - b) for a, b in zip(robot.current_joint_positions, target, strict=True))

        await self._execute(
            franky.JointMotion(target), franky.JointStopMotion, dynamics_factor, remaining, stop, on_progress
        )

    async def move_cartesian(
        self, target: list[float], dynamics_factor: float, stop: asyncio.Event, on_progress: ProgressCallback
    ) -> None:
        """
        Execute a Cartesian motion to an absolute end-effector pose.

        Args:
          target: Pose [x, y, z, qx, qy, qz, qw] in the base frame.
          dynamics_factor: Fraction of the velocity, acceleration and jerk limits.
          stop: Set to decelerate and end the motion early.
          on_progress: Receives progress updates.
        """

        franky = self._require_franky()
        robot = self._require_robot()
        motion = franky.CartesianMotion(franky.Affine(target[:3], target[3:]), franky.ReferenceType.Absolute)

        def remaining() -> float:
            return math.dist(robot.current_pose.end_effector_pose.translation, target[:3])

        await self._execute(motion, franky.CartesianStopMotion, dynamics_factor, remaining, stop, on_progress)

    async def recover_from_errors(self) -> None:
        """
        Run libfranka automatic error recovery.

        Raises:
          MotionFailed: If the robot rejects recovery.
          DeviceUnavailable: If the robot is not connected.
        """

        robot = self._require_robot()
        try:
            await asyncio.to_thread(robot.recover_from_errors)
        except self._network_errors() as error:
            raise DeviceUnavailable(str(error)) from error
        except self._errors() as error:
            raise MotionFailed(str(error)) from error

    async def read_gripper(self) -> GripperState:
        """
        Read the Franka Hand state.

        Returns:
          Finger opening, maximum opening and grasp state.

        Raises:
          DeviceUnavailable: If the gripper is not connected or cannot be read.
        """

        gripper = self._require_gripper()
        return await self._call(
            lambda: GripperState(
                width=float(gripper.width), max_width=float(gripper.max_width), is_grasped=bool(gripper.is_grasped)
            )
        )

    async def gripper_move(self, width: float, speed: float) -> bool:
        """
        Move the fingers to a width.

        Args:
          width: Target opening in metres.
          speed: Finger speed in metres per second.

        Returns:
          Whether the width was reached.
        """

        gripper = self._require_gripper()
        return await self._gripper_call(gripper.move, width, speed)

    async def gripper_grasp(
        self, width: float, speed: float, force: float, epsilon_inner: float, epsilon_outer: float
    ) -> bool:
        """
        Grasp an object.

        Args:
          width: Expected object width in metres.
          speed: Finger speed in metres per second.
          force: Grasping force in newtons.
          epsilon_inner: Accepted deviation below the expected width.
          epsilon_outer: Accepted deviation above the expected width.

        Returns:
          Whether an object within the tolerances is held.
        """

        gripper = self._require_gripper()
        return await self._gripper_call(
            lambda: gripper.grasp(width, speed, force, epsilon_inner=epsilon_inner, epsilon_outer=epsilon_outer)
        )

    async def gripper_homing(self) -> bool:
        """
        Calibrate the finger range.

        Returns:
          Whether homing succeeded.
        """

        return await self._gripper_call(self._require_gripper().homing)

    async def gripper_stop(self) -> None:
        """Abort the running gripper action."""

        await self._gripper_call(self._require_gripper().stop)

    async def _execute(
        self,
        motion: object,
        stop_motion: collections.abc.Callable[[], object],
        dynamics_factor: float,
        remaining: collections.abc.Callable[[], float],
        stop: asyncio.Event,
        on_progress: ProgressCallback,
    ) -> None:
        robot = self._require_robot()
        if stop.is_set():
            msg = "Stop was requested before the motion started; the robot did not move."
            raise MotionInterrupted(msg)

        initial = await self._call(remaining)
        started = stopping = False
        try:

            def start() -> None:
                robot.relative_dynamics_factor = dynamics_factor
                robot.move(motion, asynchronous=True)

            await asyncio.to_thread(start)
            started = True
            while not await asyncio.to_thread(robot.join_motion, self.poll_interval):
                if stop.is_set() and not stopping:
                    # A stop motion of the same control type preempts the motion and decelerates smoothly.
                    await asyncio.to_thread(robot.move, stop_motion(), asynchronous=True)
                    stopping = True
                if initial > 0 and not stopping:
                    on_progress(min(max(1.0 - await asyncio.to_thread(remaining) / initial, 0.0), 1.0), None)
        except asyncio.CancelledError:
            if started and not stopping:
                await asyncio.to_thread(self._halt, stop_motion)
            raise
        except self._network_errors() as error:
            msg = f"Lost the FCI connection during the motion ({error}); the robot state is unknown."
            raise OutcomeUnknown(msg) from error
        except self._errors() as error:
            msg = f"The robot aborted the motion: {error}"
            raise MotionFailed(msg) from error

        if stopping:
            msg = "Motion stopped on request before reaching the target."
            raise MotionInterrupted(msg)
        on_progress(1.0, 0.0)

    def _halt(self, stop_motion: collections.abc.Callable[[], object]) -> None:
        robot = self._require_robot()
        try:
            robot.move(stop_motion(), asynchronous=True)
            robot.join_motion()
        except self._errors():
            robot.stop()

    async def _gripper_call(self, function: collections.abc.Callable[..., T], *args: object) -> T:
        try:
            return await asyncio.to_thread(function, *args)
        except self._network_errors() as error:
            raise DeviceUnavailable(str(error)) from error
        except self._errors() as error:
            raise GripperFailed(str(error)) from error

    async def _call(self, function: collections.abc.Callable[[], T]) -> T:
        try:
            return await asyncio.to_thread(function)
        except self._errors() as error:
            raise DeviceUnavailable(str(error)) from error

    def _errors(self) -> tuple[type[BaseException], ...]:
        franky = self._franky
        if franky is None:
            return (RuntimeError,)
        return (franky.Exception, getattr(franky, "GripperException", RuntimeError), RuntimeError)

    def _network_errors(self) -> tuple[type[BaseException], ...]:
        franky = self._require_franky()
        return (franky.NetworkException, franky.ProtocolException)

    def _require_franky(self) -> types.ModuleType:
        if self._franky is None:
            msg = "The robot connection is closed."
            raise DeviceUnavailable(msg)
        return self._franky

    def _require_robot(self) -> "franky.Robot":
        if self._robot is None:
            msg = "The robot connection is closed."
            raise DeviceUnavailable(msg)
        return self._robot

    def _require_gripper(self) -> "franky.Gripper":
        if self._gripper is None:
            msg = "The gripper is not connected; set enable_gripper to use the Franka Hand in real mode."
            raise DeviceUnavailable(msg)
        return self._gripper
