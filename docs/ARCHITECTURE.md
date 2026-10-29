# SiLA and architecture

```text
SiLA 2 client (UniteLabs platform, SiLA Browser, scripts)
  -> SiLA features (features/*): thin methods, parameter constraints, declared errors
  -> Fr3Protocol (io/protocol.py): configuration gates, limits, workspace envelope, single-motion guard
  -> Backend
       SimulatedFr3 (io/simulation.py): nominal kinematics, minimum-jerk timing, simulated Franka Hand
       FrankyBackend (io/franky_backend.py): franky -> libfranka -> FCI (1 kHz, real-time Linux)
```

`Fr3Protocol` applies every rule before it delegates, so simulation and hardware reject the same requests. Backends execute validated requests and report progress through a callback, which features forward as SiLA execution info.

## Robot model

- Limits come from the FR3 interface specification (`io/limits.py`). Commanded joint targets stay 0.01 rad inside the position limits. Velocity and acceleration limits scale the simulated motion timing.
- Forward and inverse kinematics (`io/kinematics.py`) use Franka's published modified DH parameters with the Franka Hand TCP: 0.1034 m beyond the flange, rotated −45° about Z. At the ready pose `[0, −π/4, 0, −3π/4, 0, π/2, π/4]`, the TCP is at (0.307, 0, 0.487) m and points down.
- Inverse kinematics uses damped least squares near the current configuration. The simulator follows Cartesian motions as straight TCP lines with slerped orientation and rejects a target before moving if any sample has no nearby solution.
- On hardware, the FR3 controller and franky (Ruckig) plan the motion. The connector checks only the coarse envelope (0.855 m reach from the shoulder, configurable minimum height) before sending Cartesian targets.

## Commands

### RobotController

| Element | Kind | Notes |
| --- | --- | --- |
| `JointPositions`, `JointVelocities` | property | 7 values, rad and rad/s |
| `EndEffectorPose` | property | `Pose` structure: X, Y, Z in metres; Qx, Qy, Qz, Qw |
| `RobotMode`, `HasErrors` | property | libfranka robot mode, e.g. `Idle`, `Move`, `Reflex` |
| `MotionEnabled`, `MotionActive`, `ControlOutcomeUnknown` | property | Gate and guard state |
| `MaxDynamicsFactor`, `LowerJointLimits`, `UpperJointLimits` | property | Configured cap and FR3 limits |
| `MoveToJointPositions(JointPositions, RelativeDynamicsFactor)` | observable | Synchronized point-to-point joint motion |
| `MoveJoint(Joint, Delta, RelativeDynamicsFactor)` | observable | Rotate joint 1–7 by `Delta` rad |
| `MoveToPose(Pose, RelativeDynamicsFactor)` | observable | Absolute Cartesian target; the quaternion is normalized |
| `MoveLinear(Dx, Dy, Dz, RelativeDynamicsFactor)` | observable | Straight TCP line in the base frame, orientation held |
| `Stop` | unobservable | Ends the active motion; that motion then fails with `MotionInterrupted` |
| `RecoverFromErrors` | unobservable | libfranka error recovery; also clears `ControlOutcomeUnknown` |

`RelativeDynamicsFactor` scales the velocity, acceleration and jerk limits. SiLA constraints limit it to (0, 1], and the connector rejects values above `max_dynamics_factor`.

### Other features

- **MovementController**: `AvailablePositions` lists `Home` plus every `named_positions` entry, each with sub-position 1. `CurrentPosition` reports the matching name, or an empty name when every joint is more than 0.01 rad from all named positions. `MoveTo` moves in joint space at `default_dynamics_factor`.
- **FrankaHandController**: `Width`, `MaxWidth`, `IsGrasped`, `GripperEnabled`; `MoveFingers(Width, Speed)`, `Grasp(Width, Speed, Force, EpsilonInner, EpsilonOuter) -> Grasped`, `Homing`, `Stop`. A grasp succeeds when the fingers stop within `[Width − EpsilonInner, Width + EpsilonOuter]`.
- **GripController**: `Grip` closes on an object of unknown width at `gripper_speed` and `gripper_force`. It fails with `GripperFailed` if the fingers close fully. `Release` opens fully.
- **SimulationController**: switching is refused while a motion or gripper action runs. A failed connection keeps the current mode.

## Defined execution errors

| Error | Meaning |
| --- | --- |
| `CommandRejected` | Invalid, disabled by configuration, outside limits, another motion active, or robot has errors. Nothing moved. |
| `DeviceUnavailable` | Not connected, or state could not be read. |
| `MotionInterrupted` | `Stop` ended the motion early; the robot is stationary short of the target. |
| `MotionFailed` | The robot aborted the motion (reflex, limit violation). Inspect, then `RecoverFromErrors`. |
| `OutcomeUnknown` | The connection dropped mid-motion. Real motion stays blocked until `RecoverFromErrors`; inspect the robot first. |
| `GripperFailed` | The gripper did not reach its target or reported an error. |

## Concurrency

- One arm motion at a time; a second motion request is rejected rather than queued.
- `Stop` bypasses the guard and sets the active motion's stop event. On hardware, the backend sends a franky `JointStopMotion` or `CartesianStopMotion` of the same control type, so the robot decelerates smoothly.
- If the server cancels a motion, for example during shutdown, the backend sends the stop motion before it lets the cancellation propagate.
- Gripper actions have their own guard and can run while the arm moves. Gripper `Stop` bypasses the gripper guard.
- All blocking franky calls run in worker threads; the event loop never blocks on the robot.

## Configuration

| Key | Default | Meaning |
| --- | --- | --- |
| `simulation_mode` | `true` | Start with the simulator |
| `robot_host` | `""` | FCI address of the FR3 control unit |
| `realtime_config` | `"enforce"` | `"ignore"` only for franky-sim or bench tests |
| `enable_motion` / `enable_gripper` | `false` | Permit arm / Franka Hand actions on hardware |
| `default_dynamics_factor` / `max_dynamics_factor` | `0.05` / `0.2` | Default for named moves; hard cap for all moves |
| `gripper_speed` / `gripper_force` | `0.05` / `20.0` | For `Grip` / `Release` |
| `named_positions` | `{}` | Extra joint configurations (rad) by name |
| `minimum_tcp_height` | `0.0` | Lowest Cartesian target height in metres |
| `simulation_time_scale` | `1.0` | Wall-clock seconds per simulated second; `0` completes simulated motions immediately |
| `simulated_object_width` | `0.03` | Object width a simulated grasp closes on; `0` for none |

SiLA listens on loopback without TLS in the example configurations. Do not expose these endpoints to a network without enabling TLS and access control.
