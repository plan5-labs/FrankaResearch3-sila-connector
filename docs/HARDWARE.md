# Hardware setup

The hardware backend has been tested only against an offline franky stand-in. It has not yet been validated on a physical FR3 or with franky-sim. Qualify it on site, at low dynamics factors, with the external enabling device in reach before using it in a workflow.

## Host requirements

- **Linux x86_64 with a real-time kernel (PREEMPT_RT).** libfranka runs a 1 kHz control loop and, with `realtime_config: "enforce"`, refuses to start without real-time scheduling. Follow the real-time kernel setup in [Franka's documentation](https://frankarobotics.github.io/docs/) and grant your user real-time permissions.
- **Direct Ethernet** from the workstation to the FR3 control unit, not the arm base. Avoid switches and Wi-Fi in the FCI path.
- Python 3.12 and uv.

```bash
uv sync --extra robot
uv run python -c "import franky; print('franky', franky.__file__)"
```

### franky and firmware versions

The `franky-control` wheels on PyPI target a particular robot server version. If connecting fails with a server version mismatch, install the wheel for your firmware from franky's index, as described in the [franky FAQ](https://timschneider42.github.io/franky/getting-started/faq.html):

```bash
# Replace 10 with your robot server version
uv pip install --pre franky-control --extra-index-url "https://timschneider42.github.io/franky/whl/by-robot-server-version/10/"
```

## Bring-up sequence

1. Power the control unit and wait until the arm's lights show it is ready.
2. Open Desk in a browser at the robot's address. Unlock the joints and **activate FCI**. The connector does not unlock brakes or activate FCI for you.
3. Start the connector **read-only**:

   ```bash
   uv run python -m unitelabs.fr3_connector --config-path real.example.json
   ```

   Set `robot_host` to your FCI address first. The example uses `172.16.0.2`.
4. Check `SimulationMode` is `false`, and that `JointPositions`, `EndEffectorPose` and `RobotMode` match the robot. `MotionEnabled` must be `false`.
5. Review [`motion.example.json`](../motion.example.json): its named positions, `max_dynamics_factor` (start at 0.05–0.1), `minimum_tcp_height` for your table, and gripper force. Then restart with it.
6. Run one small `MoveJoint` at a factor of 0.05, then exercise `Stop` mid-motion before trying Cartesian motions.

## Recovering

- `HasErrors` is `true` or `RobotMode` is `Reflex`: inspect the cell, clear the cause, then call `RecoverFromErrors`.
- `ControlOutcomeUnknown` is `true`: the connection dropped during a motion, and the connector does not know whether it completed. Inspect the robot, then call `RecoverFromErrors`. The connector never retries a motion on its own.
- `StartRealMode` fails: check the cable, that FCI is active in Desk, and that no other FCI client (for example another connector) holds the robot.

## Safety notes

- `Stop` is a controlled software stop, not an emergency stop. Keep the external enabling or emergency stop device within reach.
- The envelope check (0.855 m reach, minimum height) is coarse. It does not know about fixtures, labware or people. The FR3's own safety configuration in Desk remains authoritative.
- Configure the end effector and payload in Desk. The connector's poses assume the Franka Hand TCP, and the robot reports poses for the end effector configured in Desk.
