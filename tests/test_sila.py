import asyncio
import math

import pytest

from unitelabs.fr3_connector import create_app
from unitelabs.fr3_connector.config import Fr3ConnectorConfig

from .conftest import GRIP, HAND, LOCK, MOVEMENT, ROBOT, SIMULATION

READY = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
PREGRASP = [0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785]

READ_ONLY = {"simulation_mode": False, "robot_host": "192.0.2.10", "realtime_config": "ignore"}
MOTION = {**READ_ONLY, "enable_motion": True, "enable_gripper": True}


def error_name(info):
    return type(info.value).__name__


async def test_features_are_served(sila_client):
    for identifier in (ROBOT, HAND, GRIP, MOVEMENT, SIMULATION, LOCK):
        await sila_client.load_feature(identifier)
    robot = await sila_client.load_feature(ROBOT)
    assert set(robot.commands) == {
        "MoveToJointPositions", "MoveJoint", "MoveToPose", "MoveLinear", "Stop", "RecoverFromErrors",
    }


async def test_simulated_state(sila_client):
    robot = await sila_client.load_feature(ROBOT)
    assert await robot.properties["JointPositions"].read() == pytest.approx(READY)
    pose = await robot.properties["EndEffectorPose"].read()
    assert [pose["X"], pose["Y"], pose["Z"]] == pytest.approx([0.3069, 0.0, 0.4869], abs=1e-4)
    assert await robot.properties["RobotMode"].read() == "Idle"
    assert await robot.properties["HasErrors"].read() is False
    assert await robot.properties["MotionEnabled"].read() is True
    assert await robot.properties["MaxDynamicsFactor"].read() == pytest.approx(0.2)
    assert len(await robot.properties["LowerJointLimits"].read()) == 7


async def test_joint_motion_reports_progress(sila_client, observe):
    robot = await sila_client.load_feature(ROBOT)
    _, progress = await observe(
        robot.commands["MoveToJointPositions"], {"JointPositions": PREGRASP, "RelativeDynamicsFactor": 0.1}
    )
    assert progress[-1] == pytest.approx(1.0)
    assert await robot.properties["JointPositions"].read() == pytest.approx(PREGRASP)
    await observe(robot.commands["MoveJoint"], {"Joint": 7, "Delta": -0.2, "RelativeDynamicsFactor": 0.1})
    assert (await robot.properties["JointPositions"].read())[6] == pytest.approx(PREGRASP[6] - 0.2)


async def test_cartesian_motion(sila_client, observe):
    robot = await sila_client.load_feature(ROBOT)
    before = await robot.properties["EndEffectorPose"].read()
    await observe(robot.commands["MoveLinear"], {"Dx": 0.0, "Dy": 0.1, "Dz": -0.1, "RelativeDynamicsFactor": 0.1})
    after = await robot.properties["EndEffectorPose"].read()
    assert [after["Y"], after["Z"]] == pytest.approx([before["Y"] + 0.1, before["Z"] - 0.1], abs=1e-4)
    target = {"X": 0.45, "Y": -0.1, "Z": 0.35, "Qx": 1.0, "Qy": 0.0, "Qz": 0.0, "Qw": 0.0}
    await observe(robot.commands["MoveToPose"], {"Pose": target, "RelativeDynamicsFactor": 0.1})
    reached = await robot.properties["EndEffectorPose"].read()
    assert [reached[k] for k in target] == pytest.approx(list(target.values()), abs=1e-3)


async def test_defined_errors_and_constraints(sila_client, observe):
    robot = await sila_client.load_feature(ROBOT)
    unreachable = {"X": 1.5, "Y": 0.0, "Z": 0.3, "Qx": 1.0, "Qy": 0.0, "Qz": 0.0, "Qw": 0.0}
    with pytest.raises(Exception) as info:
        await observe(robot.commands["MoveToPose"], {"Pose": unreachable, "RelativeDynamicsFactor": 0.1})
    assert error_name(info) == "CommandRejected"
    with pytest.raises(Exception) as info:
        await observe(robot.commands["MoveJoint"], {"Joint": 1, "Delta": 0.1, "RelativeDynamicsFactor": 0.5})
    assert error_name(info) == "CommandRejected"
    with pytest.raises(Exception) as info:
        await robot.commands["MoveJoint"].execute({"Joint": 8, "Delta": 0.1, "RelativeDynamicsFactor": 0.1})
    assert error_name(info) == "ValidationError"
    with pytest.raises(Exception) as info:
        await robot.commands["MoveToJointPositions"].execute({"JointPositions": [0.0], "RelativeDynamicsFactor": 0.1})
    assert error_name(info) == "ValidationError"


@pytest.mark.parametrize("sila_client", [{"simulation_time_scale": 1.0}], indirect=True)
async def test_stop_interrupts_motion(sila_client, observe):
    robot = await sila_client.load_feature(ROBOT)
    motion = asyncio.create_task(
        observe(robot.commands["MoveJoint"], {"Joint": 1, "Delta": 1.0, "RelativeDynamicsFactor": 0.05})
    )
    await asyncio.sleep(0.5)
    assert await robot.properties["MotionActive"].read() is True
    await robot.commands["Stop"].execute({})
    with pytest.raises(Exception) as info:
        await motion
    assert error_name(info) == "MotionInterrupted"
    assert 0.0 < (await robot.properties["JointPositions"].read())[0] < 1.0
    assert await robot.properties["MotionActive"].read() is False


async def test_movement_controller_named_positions(sila_client, observe):
    movement = await sila_client.load_feature(MOVEMENT)
    available = await movement.properties["AvailablePositions"].read()
    assert [position["Position"] for position in available] == ["Home"]
    assert (await movement.properties["CurrentPosition"].read())["Position"] == "Home"
    robot = await sila_client.load_feature(ROBOT)
    await observe(robot.commands["MoveJoint"], {"Joint": 1, "Delta": 0.3, "RelativeDynamicsFactor": 0.1})
    assert (await movement.properties["CurrentPosition"].read())["Position"] == ""
    await observe(movement.commands["MoveTo"], {"TargetPosition": {"Position": "Home", "SubPosition": 1}})
    assert (await movement.properties["CurrentPosition"].read())["Position"] == "Home"
    with pytest.raises(Exception) as info:
        await observe(movement.commands["MoveTo"], {"TargetPosition": {"Position": "Nowhere", "SubPosition": 1}})
    assert error_name(info) == "CommandRejected"


async def test_gripper(sila_client):
    grip = await sila_client.load_feature(GRIP)
    hand = await sila_client.load_feature(HAND)
    await grip.commands["Grip"].execute({})
    assert await hand.properties["IsGrasped"].read() is True
    assert await hand.properties["Width"].read() == pytest.approx(0.03)
    await grip.commands["Release"].execute({})
    assert await hand.properties["Width"].read() == pytest.approx(0.08)
    grasp = {"Width": 0.03, "Speed": 0.05, "Force": 20.0, "EpsilonInner": 0.005, "EpsilonOuter": 0.005}
    assert (await hand.commands["Grasp"].execute(grasp))["Grasped"] is True
    await hand.commands["MoveFingers"].execute({"Width": 0.06, "Speed": 0.05})
    assert await hand.properties["Width"].read() == pytest.approx(0.06)
    with pytest.raises(Exception) as info:
        await hand.commands["MoveFingers"].execute({"Width": 0.06, "Speed": 0.5})
    assert error_name(info) == "ValidationError"


async def test_real_mode_requires_configured_host():
    app = create_app(Fr3ConnectorConfig(simulation_mode=False))
    with pytest.raises(Exception, match="Configure robot_host"):
        await anext(app)
    await app.aclose()


async def test_start_real_mode_fails_without_host(sila_client):
    simulation = await sila_client.load_feature(SIMULATION)
    with pytest.raises(Exception) as info:
        await simulation.commands["StartRealMode"].execute({})
    assert error_name(info) == "StartRealModeFailed"
    assert await simulation.properties["SimulationMode"].read() is True


@pytest.mark.parametrize("sila_client", [READ_ONLY], indirect=True)
async def test_real_mode_is_read_only_by_default(sila_client, observe):
    simulation = await sila_client.load_feature(SIMULATION)
    robot = await sila_client.load_feature(ROBOT)
    grip = await sila_client.load_feature(GRIP)
    assert await simulation.properties["SimulationMode"].read() is False
    assert await robot.properties["MotionEnabled"].read() is False
    assert await robot.properties["JointPositions"].read() == pytest.approx(READY)
    with pytest.raises(Exception) as info:
        await observe(robot.commands["MoveJoint"], {"Joint": 1, "Delta": 0.1, "RelativeDynamicsFactor": 0.05})
    assert error_name(info) == "CommandRejected"
    with pytest.raises(Exception) as info:
        await grip.commands["Grip"].execute({})
    assert error_name(info) == "CommandRejected"
    await simulation.commands["StartSimulationMode"].execute({})
    await observe(robot.commands["MoveJoint"], {"Joint": 1, "Delta": 0.1, "RelativeDynamicsFactor": 0.05})


@pytest.mark.parametrize("sila_client", [MOTION], indirect=True)
async def test_real_motion_through_franky(sila_client, observe):
    robot = await sila_client.load_feature(ROBOT)
    hand = await sila_client.load_feature(HAND)
    await observe(
        robot.commands["MoveToJointPositions"], {"JointPositions": PREGRASP, "RelativeDynamicsFactor": 0.05}
    )
    assert await robot.properties["JointPositions"].read() == pytest.approx(PREGRASP)
    grasp = {"Width": 0.025, "Speed": 0.05, "Force": 20.0, "EpsilonInner": 0.005, "EpsilonOuter": 0.005}
    assert (await hand.commands["Grasp"].execute(grasp))["Grasped"] is True


@pytest.mark.parametrize("sila_client", [READ_ONLY], indirect=True)
async def test_switching_back_to_real_mode(sila_client):
    simulation = await sila_client.load_feature(SIMULATION)
    await simulation.commands["StartSimulationMode"].execute({})
    assert await simulation.properties["SimulationMode"].read() is True
    await simulation.commands["StartRealMode"].execute({})
    assert await simulation.properties["SimulationMode"].read() is False
