import typing_extensions as typing

from unitelabs.cdk import sila

from ...io import Fr3Protocol
from ...io.limits import HAND_MAX_FORCE, HAND_MAX_SPEED, HAND_MAX_WIDTH

Width = typing.Annotated[
    float,
    sila.constraints.Unit(label="m", components=[sila.constraints.UnitComponent("Meter")]),
    sila.constraints.MinimalInclusive(0.0),
    sila.constraints.MaximalInclusive(HAND_MAX_WIDTH),
]
Tolerance = typing.Annotated[
    float,
    sila.constraints.Unit(label="m", components=[sila.constraints.UnitComponent("Meter")]),
    sila.constraints.MinimalInclusive(0.0),
]
Speed = typing.Annotated[
    float,
    sila.constraints.Unit(
        label="m/s",
        components=[sila.constraints.UnitComponent("Meter"), sila.constraints.UnitComponent("Second", -1)],
    ),
    sila.constraints.MinimalExclusive(0.0),
    sila.constraints.MaximalInclusive(HAND_MAX_SPEED),
]
Force = typing.Annotated[
    float,
    sila.constraints.Unit(
        label="N",
        components=[
            sila.constraints.UnitComponent("Kilogram"),
            sila.constraints.UnitComponent("Meter"),
            sila.constraints.UnitComponent("Second", -2),
        ],
    ),
    sila.constraints.MinimalExclusive(0.0),
    sila.constraints.MaximalInclusive(HAND_MAX_FORCE),
]


class FrankaHandController(sila.Feature):
    """
    Control the Franka Hand parallel gripper with explicit width, speed and force.

    Widths are the distance between the fingers in metres. A grasp succeeds when the fingers stop within the
    given tolerances around the expected object width.
    """

    def __init__(self, protocol: Fr3Protocol) -> None:
        super().__init__(originator="io.unitelabs", category="robot", version="1.0", maturity_level="Draft")
        self.protocol = protocol

    @sila.UnobservableProperty()
    async def get_width(self) -> float:
        """
        Read the current finger opening in metres.

        Raises:
          DeviceUnavailable: If the gripper is not connected.
        """

        return (await self.protocol.read_gripper()).width

    @sila.UnobservableProperty()
    async def get_max_width(self) -> float:
        """
        Read the maximum finger opening in metres measured by the last homing.

        Raises:
          DeviceUnavailable: If the gripper is not connected.
        """

        return (await self.protocol.read_gripper()).max_width

    @sila.UnobservableProperty()
    async def get_is_grasped(self) -> bool:
        """
        Report whether an object is currently held.

        Raises:
          DeviceUnavailable: If the gripper is not connected.
        """

        return (await self.protocol.read_gripper()).is_grasped

    @sila.UnobservableProperty()
    async def get_gripper_enabled(self) -> bool:
        """Report whether gripper actions are permitted in the current backend."""

        return self.protocol.gripper_enabled

    @sila.UnobservableCommand()
    async def move_fingers(self, width: Width, speed: Speed) -> None:
        """
        Move the fingers to a width without grasping.

        Args:
          Width: Target finger opening in metres.
          Speed: Finger speed in metres per second.

        Raises:
          CommandRejected: If disabled, invalid or another gripper action is active.
          DeviceUnavailable: If the gripper is not connected.
          GripperFailed: If the width was not reached.
        """

        await self.protocol.gripper_move(width, speed)

    @sila.UnobservableCommand()
    async def grasp(
        self, width: Width, speed: Speed, force: Force, epsilon_inner: Tolerance, epsilon_outer: Tolerance
    ) -> bool:
        """
        Close on an object and hold it with a constant force.

        Args:
          Width: Expected object width in metres.
          Speed: Finger speed in metres per second.
          Force: Grasping force in newtons.
          EpsilonInner: Accepted deviation below the expected width in metres.
          EpsilonOuter: Accepted deviation above the expected width in metres.

        Returns:
          Grasped: Whether an object within the tolerances is held.

        Raises:
          CommandRejected: If disabled, invalid or another gripper action is active.
          DeviceUnavailable: If the gripper is not connected.
          GripperFailed: If the gripper reported an error.
        """

        return await self.protocol.gripper_grasp(width, speed, force, epsilon_inner, epsilon_outer)

    @sila.UnobservableCommand()
    async def homing(self) -> None:
        """
        Calibrate the finger range by opening and closing fully. Remove any object first.

        Raises:
          CommandRejected: If disabled or another gripper action is active.
          DeviceUnavailable: If the gripper is not connected.
          GripperFailed: If homing did not succeed.
        """

        await self.protocol.gripper_homing()

    @sila.UnobservableCommand()
    async def stop(self) -> None:
        """
        Abort the running gripper action.

        Raises:
          CommandRejected: If gripper actions are disabled.
          DeviceUnavailable: If the gripper is not connected.
          GripperFailed: If the gripper reported an error.
        """

        await self.protocol.gripper_stop()
