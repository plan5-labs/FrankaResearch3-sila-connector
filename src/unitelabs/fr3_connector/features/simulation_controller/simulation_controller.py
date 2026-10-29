import typing_extensions as typing

from unitelabs.cdk.features.core.simulation_controller import (
    SimulationControllerBase,
    StartRealModeFailed,
    StartSimulationModeFailed,
)

from ...io import Fr3Protocol
from ...io.errors import CommandRejected, DeviceUnavailable


class SimulationController(SimulationControllerBase):
    """Switch between the configured FR3 and the kinematic simulator while no action is running."""

    def __init__(self, protocol: Fr3Protocol) -> None:
        super().__init__()
        self.protocol = protocol

    @typing.override
    async def simulation_mode(self) -> bool:
        """Report the active backend."""
        return self.protocol.simulation_mode

    @typing.override
    async def start_simulation_mode(self) -> None:
        """
        Use the kinematic simulator while no motion or gripper action is running.

        Raises:
          StartSimulationModeFailed: If an action is still running.
        """
        try:
            await self.protocol.set_simulation_mode(True)
        except (CommandRejected, DeviceUnavailable) as error:
            raise StartSimulationModeFailed(str(error)) from error

    @typing.override
    async def start_real_mode(self) -> None:
        """
        Connect to the configured FR3 through FCI while no simulated action is running.

        Raises:
          StartRealModeFailed: If an action is running or the robot cannot be reached.
        """
        try:
            await self.protocol.set_simulation_mode(False)
        except (CommandRejected, DeviceUnavailable) as error:
            raise StartRealModeFailed(str(error)) from error
