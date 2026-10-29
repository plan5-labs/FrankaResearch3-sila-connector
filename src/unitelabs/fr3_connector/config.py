import dataclasses
from importlib.metadata import version

import typing_extensions as typing

from unitelabs.cdk import ConnectorBaseConfig, SiLAServerConfig

__version__ = version("unitelabs-fr3-connector")


@dataclasses.dataclass
class Fr3ConnectorConfig(ConnectorBaseConfig):
    """Configure the Franka Research 3 connector."""

    simulation_mode: bool = True
    """Run against the kinematic simulator. Disable to connect to the configured robot through FCI."""

    robot_host: str = ""
    """FCI IPv4 address or hostname of the FR3 control unit. Required for real mode."""

    realtime_config: typing.Literal["enforce", "ignore"] = "enforce"
    """Require a real-time kernel for the 1 kHz control loop. Use "ignore" only for franky-sim or bench tests."""

    enable_motion: bool = False
    """Permit arm motion on the physical robot after on-site review. Simulation always permits motion."""

    enable_gripper: bool = False
    """Connect to and permit actions of the Franka Hand on the physical robot. Simulation always permits them."""

    default_dynamics_factor: float = 0.05
    """Fraction of the velocity, acceleration and jerk limits for commands without an explicit factor."""

    max_dynamics_factor: float = 0.2
    """Largest relative dynamics factor any command may request."""

    gripper_speed: float = 0.05
    """Finger speed in metres per second for Grip and Release."""

    gripper_force: float = 20.0
    """Grasping force in newtons for Grip."""

    named_positions: dict[str, list[float]] = dataclasses.field(default_factory=dict)
    """Additional joint configurations in radians by name; "Home" defaults to Franka's ready pose."""

    minimum_tcp_height: float = 0.0
    """Lowest permitted Cartesian target height in metres above the robot base."""

    simulation_time_scale: float = 1.0
    """Wall-clock seconds per simulated second. Zero completes simulated motions immediately."""

    simulated_object_width: float = 0.03
    """Width in metres of the object a simulated grasp closes on; zero means the fingers close on nothing."""

    sila_server: SiLAServerConfig = dataclasses.field(
        default_factory=lambda: SiLAServerConfig(
            hostname="127.0.0.1",
            port=50052,
            name="Franka Research 3 (Simulation)",
            type="Robot",
            description="Franka Research 3 robot arm and Franka Hand; backend selected by configuration.",
            version=str(__version__),
            vendor_url="https://github.com/plan5-labs/FrankaResearch3-sila-connector",
        )
    )
