import asyncio
import collections.abc
import itertools
import math

import numpy as np

from . import kinematics
from .backend import GripperState, ProgressCallback, RobotState
from .errors import CommandRejected, MotionInterrupted
from .limits import (
    HAND_MAX_WIDTH,
    JOINT_ACCELERATION_MAX,
    JOINT_COUNT,
    JOINT_VELOCITY_MAX,
    READY_POSE,
    ROTATION_ACCELERATION_MAX,
    ROTATION_VELOCITY_MAX,
    TRANSLATION_ACCELERATION_MAX,
    TRANSLATION_VELOCITY_MAX,
)

STEP_SECONDS = 0.02
"""Simulated control period used for interpolation and progress updates."""

MAX_PATH_SAMPLES = 200
"""Upper bound for inverse-kinematics samples along one Cartesian motion."""


class SimulatedFr3:
    """
    Model FR3 and Franka Hand kinematics without dynamics, contacts or collisions.

    Motions follow a minimum-jerk profile whose duration respects the scaled velocity and acceleration limits.
    Cartesian motions follow a straight TCP line solved with nominal inverse kinematics.
    """

    def __init__(self, *, time_scale: float = 1.0, object_width: float = 0.0) -> None:
        self.time_scale = time_scale
        """Wall-clock seconds per simulated second; zero completes motions immediately."""

        self.object_width = object_width
        """Width in metres of the object a grasp closes on; zero means no object between the fingers."""

        self.joint_positions = list(READY_POSE)
        self.joint_velocities = [0.0] * JOINT_COUNT
        self.robot_mode = "Idle"
        self.has_errors = False
        self.gripper_width = HAND_MAX_WIDTH
        self.gripper_grasped = False

    async def open(self) -> None:
        """Nothing to connect in simulation."""

    async def close(self) -> None:
        """Nothing to disconnect in simulation."""

    async def read_state(self) -> RobotState:
        """
        Read the simulated arm state.

        Returns:
          The current joint state and the nominal Franka Hand TCP pose.
        """

        return RobotState(
            joint_positions=list(self.joint_positions),
            joint_velocities=list(self.joint_velocities),
            end_effector_pose=kinematics.matrix_to_pose(kinematics.forward_kinematics(self.joint_positions)),
            robot_mode=self.robot_mode,
            has_errors=self.has_errors,
        )

    async def move_joints(
        self, target: list[float], dynamics_factor: float, stop: asyncio.Event, on_progress: ProgressCallback
    ) -> None:
        """
        Interpolate synchronously in joint space.

        Args:
          target: Absolute joint angles in radians.
          dynamics_factor: Fraction of the joint velocity and acceleration limits.
          stop: Set to end the motion early.
          on_progress: Receives progress updates.
        """

        start = list(self.joint_positions)
        duration = max(
            kinematics.minimum_jerk_duration(
                abs(goal - origin), velocity * dynamics_factor, acceleration * dynamics_factor
            )
            for origin, goal, velocity, acceleration in zip(
                start, target, JOINT_VELOCITY_MAX, JOINT_ACCELERATION_MAX, strict=True
            )
        )

        def sample(fraction: float) -> list[float]:
            return [origin + fraction * (goal - origin) for origin, goal in zip(start, target, strict=True)]

        await self._follow(sample, duration, stop, on_progress)

    async def move_cartesian(
        self, target: list[float], dynamics_factor: float, stop: asyncio.Event, on_progress: ProgressCallback
    ) -> None:
        """
        Move the TCP along a straight line while interpolating orientation.

        Args:
          target: Absolute pose [x, y, z, qx, qy, qz, qw] in the base frame.
          dynamics_factor: Fraction of the Cartesian and joint velocity and acceleration limits.
          stop: Set to end the motion early.
          on_progress: Receives progress updates.

        Raises:
          CommandRejected: If no inverse-kinematics solution follows the line; the robot does not move.
        """

        start = list(self.joint_positions)
        origin = kinematics.forward_kinematics(start)
        goal = kinematics.pose_to_matrix(target[:3], target[3:])
        distance = float(np.linalg.norm(goal[:3, 3] - origin[:3, 3]))
        angle = float(np.linalg.norm(kinematics.rotation_vector(goal[:3, :3] @ origin[:3, :3].T)))
        duration = max(
            kinematics.minimum_jerk_duration(
                distance, TRANSLATION_VELOCITY_MAX * dynamics_factor, TRANSLATION_ACCELERATION_MAX * dynamics_factor
            ),
            kinematics.minimum_jerk_duration(
                angle, ROTATION_VELOCITY_MAX * dynamics_factor, ROTATION_ACCELERATION_MAX * dynamics_factor
            ),
        )
        steps = min(max(math.ceil(duration / STEP_SECONDS), 1), MAX_PATH_SAMPLES)
        path = await asyncio.to_thread(kinematics.plan_linear_path, start, goal, steps)
        if path is None:
            msg = "The target pose cannot be reached along a straight line from the current configuration."
            raise CommandRejected(msg)

        # Stretch the motion if the joints would otherwise exceed their scaled velocity limits at peak speed.
        waypoints = [start, *path]
        steepest = max(
            abs(b - a) / (velocity * dynamics_factor)
            for previous, current in itertools.pairwise(waypoints)
            for a, b, velocity in zip(previous, current, JOINT_VELOCITY_MAX, strict=True)
        )
        duration = max(duration, kinematics.MINIMUM_JERK_PEAK_VELOCITY * steps * steepest)

        def sample(fraction: float) -> list[float]:
            position = fraction * steps
            index = min(int(position), steps - 1)
            weight = position - index
            return [a + weight * (b - a) for a, b in zip(waypoints[index], waypoints[index + 1], strict=True)]

        await self._follow(sample, duration, stop, on_progress)

    async def recover_from_errors(self) -> None:
        """Clear the simulated error state."""

        self.has_errors = False
        self.robot_mode = "Idle"

    async def read_gripper(self) -> GripperState:
        """
        Read the simulated Franka Hand state.

        Returns:
          The finger opening and grasp state.
        """

        return GripperState(width=self.gripper_width, max_width=HAND_MAX_WIDTH, is_grasped=self.gripper_grasped)

    async def gripper_move(self, width: float, speed: float) -> bool:
        """
        Move the fingers; the simulated object only affects grasps.

        Args:
          width: Target opening in metres.
          speed: Finger speed in metres per second.

        Returns:
          Always True.
        """

        await self._finger_delay(width, speed)
        self.gripper_width = width
        self.gripper_grasped = False
        return True

    async def gripper_grasp(
        self, width: float, speed: float, force: float, epsilon_inner: float, epsilon_outer: float
    ) -> bool:
        """
        Close until the simulated object, or fully if there is none.

        Args:
          width: Expected object width in metres.
          speed: Finger speed in metres per second.
          force: Grasping force in newtons; not modelled.
          epsilon_inner: Accepted deviation below the expected width.
          epsilon_outer: Accepted deviation above the expected width.

        Returns:
          Whether the final opening lies within the expected width tolerances.
        """

        del force
        contact = self.object_width if 0.0 < self.object_width < self.gripper_width else 0.0
        await self._finger_delay(contact, speed)
        self.gripper_width = contact
        self.gripper_grasped = width - epsilon_inner <= contact <= width + epsilon_outer
        return self.gripper_grasped

    async def gripper_homing(self) -> bool:
        """
        Open the fingers fully.

        Returns:
          Always True.
        """

        await self._finger_delay(HAND_MAX_WIDTH, 0.05)
        self.gripper_width = HAND_MAX_WIDTH
        self.gripper_grasped = False
        return True

    async def gripper_stop(self) -> None:
        """Do nothing; simulated finger moves are atomic."""

    async def _finger_delay(self, width: float, speed: float) -> None:
        if self.time_scale > 0:
            await asyncio.sleep(abs(width - self.gripper_width) / speed * self.time_scale)

    async def _follow(
        self,
        sample: collections.abc.Callable[[float], list[float]],
        duration: float,
        stop: asyncio.Event,
        on_progress: ProgressCallback,
    ) -> None:
        if stop.is_set():
            msg = "Stop was requested before the motion started; the robot did not move."
            raise MotionInterrupted(msg)

        wall_duration = duration * self.time_scale
        if wall_duration <= 0:
            self.joint_positions = sample(1.0)
            on_progress(1.0, 0.0)
            return

        loop = asyncio.get_running_loop()
        started = previous_time = loop.time()
        self.robot_mode = "Move"
        try:
            while True:
                now = loop.time()
                fraction = min((now - started) / wall_duration, 1.0)
                previous = self.joint_positions
                self.joint_positions = sample(kinematics.minimum_jerk(fraction))
                elapsed = (now - previous_time) / self.time_scale
                if elapsed > 0:
                    self.joint_velocities = [
                        (b - a) / elapsed for a, b in zip(previous, self.joint_positions, strict=True)
                    ]
                previous_time = now
                on_progress(fraction, (1.0 - fraction) * wall_duration)
                if fraction >= 1.0:
                    return
                if stop.is_set():
                    msg = f"Motion stopped at {fraction:.0%} of its duration before reaching the target."
                    raise MotionInterrupted(msg)
                await asyncio.sleep(min(STEP_SECONDS * self.time_scale, wall_duration))
        finally:
            self.robot_mode = "Idle"
            self.joint_velocities = [0.0] * JOINT_COUNT
