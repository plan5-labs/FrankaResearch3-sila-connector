import datetime

import typing_extensions as typing

from unitelabs.cdk import sila
from unitelabs.cdk.features.robot.movement_controller import MovementControllerBase, PositionIndex, TargetPosition

from ...io import Fr3Protocol
from ...io.errors import CommandRejected


class MovementController(MovementControllerBase):
    """
    Move the arm to named joint configurations, e.g. Home or positions taught for a workcell.

    Each named position has exactly one sub-position. Motions use the configured default dynamics factor.
    """

    def __init__(self, protocol: Fr3Protocol) -> None:
        super().__init__()
        self.protocol = protocol

    @typing.override
    async def get_available_positions(self) -> list[TargetPosition]:
        """Get all named positions of the arm, each with one sub-position."""

        return [
            TargetPosition(position=name, sub_position=PositionIndex(position_index=1))
            for name in self.protocol.named_positions
        ]

    @typing.override
    async def get_current_position(self) -> TargetPosition:
        """
        Get the named position the arm is at, or an empty position name if it is elsewhere.

        Raises:
          DeviceUnavailable: If the robot state cannot be read.
        """

        return TargetPosition(
            position=await self.protocol.current_named_position(), sub_position=PositionIndex(position_index=1)
        )

    @typing.override
    async def move_to(self, target_position: TargetPosition, *, status: sila.Status) -> None:
        """
        Move the arm in joint space to a named position.

        Args:
          TargetPosition: A named position from AvailablePositions with sub-position 1.

        Raises:
          CommandRejected: If the position is unknown, motion is disabled or another motion is active.
          DeviceUnavailable: If the robot state cannot be read.
          MotionInterrupted: If Stop ended the motion before the target was reached.
          MotionFailed: If the robot aborted the motion; inspect it and call RecoverFromErrors.
          OutcomeUnknown: If the connection was lost during the motion. Do not retry blindly.
        """

        if target_position.sub_position.position_index != 1:
            msg = f"Position {target_position.position!r} has only sub-position 1."
            raise CommandRejected(msg)

        def report(fraction: float, remaining: float | None) -> None:
            status.update(
                progress=fraction,
                remaining_time=datetime.timedelta(seconds=remaining) if remaining is not None else None,
            )

        await self.protocol.move_to_named_position(target_position.position, on_progress=report)
