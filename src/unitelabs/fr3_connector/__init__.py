import collections.abc
from importlib.metadata import version

from unitelabs.cdk import Connector
from unitelabs.cdk.features.core.lock_controller import LockController

from .config import Fr3ConnectorConfig
from .features.franka_hand_controller import FrankaHandController
from .features.grip_controller import GripController
from .features.movement_controller import MovementController
from .features.robot_controller import RobotController
from .features.simulation_controller import SimulationController
from .io import Fr3Protocol

__version__ = version("unitelabs-fr3-connector")


async def create_app(config: Fr3ConnectorConfig) -> collections.abc.AsyncGenerator[Connector, None]:
    """Create the connector and manage its connection lifecycle."""
    app = Connector(config)
    protocol = Fr3Protocol(
        simulation_mode=config.simulation_mode,
        robot_host=config.robot_host,
        realtime_config=config.realtime_config,
        enable_motion=config.enable_motion,
        enable_gripper=config.enable_gripper,
        default_dynamics_factor=config.default_dynamics_factor,
        max_dynamics_factor=config.max_dynamics_factor,
        gripper_speed=config.gripper_speed,
        gripper_force=config.gripper_force,
        named_positions=config.named_positions,
        minimum_tcp_height=config.minimum_tcp_height,
        simulation_time_scale=config.simulation_time_scale,
        simulated_object_width=config.simulated_object_width,
    )
    app.register(LockController())
    app.register(SimulationController(protocol))
    app.register(RobotController(protocol))
    app.register(MovementController(protocol))
    app.register(FrankaHandController(protocol))
    app.register(GripController(protocol))
    try:
        await protocol.open()
        yield app
    finally:
        await protocol.close()
