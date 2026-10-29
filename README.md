# Franka Research 3 SiLA Connector

Control a [Franka Research 3](https://franka.de/) (FR3) arm and Franka Hand through **SiLA 2**, built with the [UniteLabs CDK](https://gitlab.com/unitelabs/cdk/python-cdk). The connector serves standard SiLA features (`SimulationController`, `LockController`, `MovementController`, `GripController`) next to FR3-specific joint, Cartesian and gripper control.

A kinematic simulator is the default backend, so you can develop and test clients on macOS, Linux or Windows without a robot. The hardware backend uses [franky](https://github.com/TimSchneider42/franky) over the Franka Control Interface (FCI). It needs a Linux workstation with a real-time kernel.

The project layout follows the [UR5e SiLA connector](https://github.com/AccelerationConsortium/ur5e-sila-connector): `io/` owns robot communication, limits and simulation, and `features/` exposes thin SiLA methods.

## Quick start: no robot required

Prerequisites: Git, [uv](https://docs.astral.sh/uv/getting-started/installation/) and Python 3.12. uv can install Python for you.

```bash
git clone https://github.com/plan5-labs/FrankaResearch3-sila-connector.git
cd FrankaResearch3-sila-connector
uv sync --all-extras
```

**Terminal 1: simulated connector**

```bash
uv run python -m unitelabs.fr3_connector --config-path simulation.example.json
```

**Terminal 2: demo client**

```bash
uv run python examples/demo_client.py --port 50052
```

The demo reads the robot state, moves to `Home`, lowers the TCP 10 cm in a straight line, grips and releases, then stops a joint motion part-way and returns to `Home`. Every call goes over real SiLA network requests. Any SiLA 2 client works.

## SiLA features

| Feature | Identifier | Purpose |
| --- | --- | --- |
| RobotController | `io.unitelabs/robot/RobotController/v1` | Joint/TCP state; observable `MoveToJointPositions`, `MoveJoint`, `MoveToPose`, `MoveLinear`; `Stop`; `RecoverFromErrors` |
| MovementController | `io.unitelabs/robot/MovementController/v1` | Move to named joint configurations (`Home` plus the ones you configure) |
| FrankaHandController | `io.unitelabs/robot/FrankaHandController/v1` | `MoveFingers`, `Grasp` with width/speed/force/tolerances, `Homing`, `Stop`; width and grasp state |
| GripController | `io.unitelabs/robot/GripController/v1` | Generic `Grip` / `Release` with configured speed and force |
| SimulationController | `org.silastandard/core/SimulationController/v1` | Switch between simulator and robot while idle |
| LockController | `org.silastandard/core/LockController/v2` | Reserve the connector for one client |

Units are SI: radians, metres, seconds and newtons. Poses use the Franka Hand TCP in the robot base frame as `X, Y, Z, Qx, Qy, Qz, Qw`. Motion commands are SiLA *observable* commands, so clients receive progress and remaining-time updates. See [Architecture](docs/ARCHITECTURE.md) for the full command reference and safety gates.

## Connecting a physical FR3

Start read-only with [`real.example.json`](real.example.json), which disables motion and gripper actions, and enable motion only after an on-site review. [Hardware setup](docs/HARDWARE.md) covers the real-time kernel, network, Desk/FCI activation, franky version matching and the configuration switches.

```bash
uv sync --extra robot   # Linux x86_64 only
uv run python -m unitelabs.fr3_connector --config-path real.example.json
```

## What is supported

| Capability | Status |
| --- | --- |
| Simulation through the same SiLA features | Implemented; automated tests over the network |
| Joint, relative-joint, Cartesian pose and straight-line motions | Implemented; simulator uses nominal FK/IK and minimum-jerk timing |
| Stop with smooth deceleration (franky stop motions) | Implemented; tested against an offline franky stand-in |
| Franka Hand move/grasp/homing/stop, generic grip/release | Implemented |
| Fail-closed handling of lost connections during motion | Implemented; motions blocked until `RecoverFromErrors` |
| Hardware backend (franky / FCI) | Implemented. **Not yet validated on a physical FR3 or franky-sim** |
| Collision checking, force control, impedance control, Desk API (brakes, FCI activation) | Not implemented |

The simulator models kinematics only. It does not model dynamics, contacts or collisions, and passing tests do not establish collision safety.

## Development

```bash
uv run pytest -q
uv run ruff format --check src/
uv run ruff check src/
```

See [Contributing](CONTRIBUTING.md).
