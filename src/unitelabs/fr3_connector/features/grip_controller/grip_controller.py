import typing_extensions as typing

from unitelabs.cdk.features.robot.grip_controller import GripControllerBase

from ...io import Fr3Protocol


class GripController(GripControllerBase):
    """Grip and release labware with the Franka Hand using the configured speed and force."""

    def __init__(self, protocol: Fr3Protocol) -> None:
        super().__init__()
        self.protocol = protocol

    @typing.override
    async def grip(self) -> None:
        """
        Close the gripper on an object of unknown width.

        Raises:
          CommandRejected: If disabled or another gripper action is active.
          DeviceUnavailable: If the gripper is not connected.
          GripperFailed: If the fingers closed without holding an object.
        """

        await self.protocol.grip()

    @typing.override
    async def release(self) -> None:
        """
        Open the gripper fully.

        Raises:
          CommandRejected: If disabled or another gripper action is active.
          DeviceUnavailable: If the gripper is not connected.
          GripperFailed: If the gripper did not open.
        """

        await self.protocol.release()
