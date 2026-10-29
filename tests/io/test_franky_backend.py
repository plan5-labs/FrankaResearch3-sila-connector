import asyncio
import builtins
from pathlib import Path
import sys

import pytest

from unitelabs.fr3_connector.io import Fr3Protocol
from unitelabs.fr3_connector.io.errors import (
    CommandRejected,
    DeviceUnavailable,
    GripperFailed,
    MotionFailed,
    MotionInterrupted,
    OutcomeUnknown,
)
from unitelabs.fr3_connector.io.franky_backend import FrankyBackend

from ..conftest import FAKE_FRANKY

PREGRASP = [0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785]


def ignore(fraction, remaining):
    pass


async def connected(fake_franky, **kwargs):
    backend = FrankyBackend("192.0.2.10", poll_interval=0.005, **kwargs)
    await backend.open()
    return backend, fake_franky.Robot.instances[-1]


async def test_open_selects_realtime_mode_and_reads_state(fake_franky):
    backend, robot = await connected(fake_franky, realtime_config="ignore")
    assert robot.fci_hostname == "192.0.2.10"
    assert robot.realtime_config is fake_franky.RealtimeConfig.Ignore
    state = await backend.read_state()
    assert state.joint_positions == robot.q
    assert state.end_effector_pose == [0.3069, 0.0, 0.4869, 1.0, 0.0, 0.0, 0.0]
    assert state.robot_mode == "Idle"
    assert not state.has_errors
    assert not fake_franky.Gripper.instances


async def test_open_reports_missing_franky(monkeypatch):
    real_import = builtins.__import__

    def without_franky(name, *args, **kwargs):
        if name == "franky":
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", without_franky)
    with pytest.raises(DeviceUnavailable, match="uv sync --extra robot"):
        await FrankyBackend("192.0.2.10").open()


async def test_open_reports_connection_failure(fake_franky):
    fake_franky.Robot.behavior["fail_connect"] = True
    with pytest.raises(DeviceUnavailable, match="timed out"):
        await FrankyBackend("192.0.2.10").open()


async def test_joint_motion_uses_requested_dynamics(fake_franky):
    backend, robot = await connected(fake_franky)
    progress = []
    await backend.move_joints(PREGRASP, 0.07, asyncio.Event(), lambda f, r: progress.append(f))
    assert robot.q == PREGRASP
    assert robot.factors == [0.07]
    assert isinstance(robot.moves[0], fake_franky.JointMotion)
    assert progress[-1] == 1.0


async def test_cartesian_motion_targets_absolute_affine(fake_franky):
    backend, robot = await connected(fake_franky)
    target = [0.4, 0.1, 0.3, 1.0, 0.0, 0.0, 0.0]
    await backend.move_cartesian(target, 0.05, asyncio.Event(), ignore)
    motion = robot.moves[0]
    assert isinstance(motion, fake_franky.CartesianMotion)
    assert motion.reference_type is fake_franky.ReferenceType.Absolute
    assert motion.target.translation == target[:3]
    assert motion.target.quaternion == target[3:]


@pytest.mark.parametrize(
    ("method", "stop_type"), [("move_joints", "JointStopMotion"), ("move_cartesian", "CartesianStopMotion")]
)
async def test_stop_sends_matching_stop_motion(fake_franky, method, stop_type):
    backend, robot = await connected(fake_franky)
    fake_franky.Robot.motion_seconds = 5.0
    stop = asyncio.Event()
    target = PREGRASP if method == "move_joints" else [0.4, 0.1, 0.3, 1.0, 0.0, 0.0, 0.0]
    motion = asyncio.create_task(getattr(backend, method)(target, 0.05, stop, ignore))
    await asyncio.sleep(0.05)
    stop.set()
    with pytest.raises(MotionInterrupted):
        await asyncio.wait_for(motion, timeout=2)
    assert type(robot.moves[-1]).__name__ == stop_type
    assert robot.q != PREGRASP


async def test_reflex_raises_motion_failed(fake_franky):
    backend, _ = await connected(fake_franky)
    fake_franky.Robot.behavior["reflex"] = True
    with pytest.raises(MotionFailed, match="reflex"):
        await backend.move_joints(PREGRASP, 0.05, asyncio.Event(), ignore)
    assert (await backend.read_state()).has_errors


async def test_network_loss_raises_outcome_unknown(fake_franky):
    backend, _ = await connected(fake_franky)
    fake_franky.Robot.behavior["network"] = True
    with pytest.raises(OutcomeUnknown):
        await backend.move_joints(PREGRASP, 0.05, asyncio.Event(), ignore)


async def test_cancellation_stops_the_robot(fake_franky):
    backend, robot = await connected(fake_franky)
    fake_franky.Robot.motion_seconds = 5.0
    motion = asyncio.create_task(backend.move_joints(PREGRASP, 0.05, asyncio.Event(), ignore))
    await asyncio.sleep(0.05)
    motion.cancel()
    with pytest.raises(asyncio.CancelledError):
        await motion
    assert isinstance(robot.moves[-1], fake_franky.JointStopMotion)


async def test_gripper_calls(fake_franky):
    backend, _ = await connected(fake_franky, gripper_enabled=True)
    gripper = fake_franky.Gripper.instances[-1]
    assert (await backend.read_gripper()).width == 0.08
    assert await backend.gripper_move(0.04, 0.05)
    assert await backend.gripper_grasp(0.025, 0.05, 20.0, 0.005, 0.01)
    await backend.gripper_stop()
    assert gripper.calls == [("move", 0.04, 0.05), ("grasp", 0.025, 0.05, 20.0, 0.005, 0.01), ("stop",)]


async def test_gripper_errors_are_mapped(fake_franky):
    backend, _ = await connected(fake_franky, gripper_enabled=True)
    fake_franky.Gripper.behavior["fault"] = True
    with pytest.raises(GripperFailed, match="fault"):
        await backend.gripper_homing()


async def test_gripper_must_be_enabled(fake_franky):
    backend, _ = await connected(fake_franky)
    with pytest.raises(DeviceUnavailable, match="enable_gripper"):
        await backend.read_gripper()


async def test_protocol_over_franky_backend(fake_franky):
    protocol = Fr3Protocol(simulation_mode=False, robot_host="192.0.2.10", enable_motion=True)
    await protocol.open()
    robot = fake_franky.Robot.instances[-1]
    await protocol.move_to_joint_positions(PREGRASP, 0.05)
    assert robot.q == PREGRASP
    fake_franky.Robot.behavior["reflex"] = True
    with pytest.raises(MotionFailed):
        await protocol.move_joint(1, 0.1, 0.05)
    fake_franky.Robot.behavior.clear()
    with pytest.raises(CommandRejected, match="RecoverFromErrors"):
        await protocol.move_joint(1, 0.1, 0.05)
    await protocol.recover_from_errors()
    assert robot.recoveries == 1
    await protocol.move_joint(1, 0.1, 0.05)
    assert robot.q[0] == pytest.approx(0.1)
    await protocol.close()


def test_installed_franky_provides_the_api_used_by_the_backend():
    """Runs only where the real franky wheel imports (Linux x86_64); guards against API drift."""

    sys.modules.pop("franky", None)
    franky = pytest.importorskip("franky")
    if Path(franky.__file__).resolve().is_relative_to(FAKE_FRANKY.resolve()):
        pytest.skip("only the offline stand-in is importable")
    for name in (
        "Robot", "Gripper", "Affine", "JointMotion", "CartesianMotion", "JointStopMotion", "CartesianStopMotion",
        "ReferenceType", "RealtimeConfig", "Exception", "NetworkException", "ProtocolException", "GripperException",
    ):
        assert hasattr(franky, name), name
    assert franky.ReferenceType.Absolute is not None
    assert {franky.RealtimeConfig.Enforce, franky.RealtimeConfig.Ignore}
    for name in (
        "move", "join_motion", "recover_from_errors", "relative_dynamics_factor", "state", "current_pose",
        "current_joint_positions", "has_errors",
    ):
        assert hasattr(franky.Robot, name), name
    for name in ("move", "grasp", "homing", "stop", "width", "max_width", "is_grasped"):
        assert hasattr(franky.Gripper, name), name
    affine = franky.Affine([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
    assert list(affine.translation) == pytest.approx([0.1, 0.2, 0.3])
    assert list(affine.quaternion) == pytest.approx([0.0, 0.0, 0.0, 1.0])
    sys.modules.pop("franky", None)
