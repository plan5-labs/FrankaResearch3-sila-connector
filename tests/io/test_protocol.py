import asyncio
import math

import numpy as np
import pytest

from unitelabs.fr3_connector.io import Fr3Protocol, kinematics
from unitelabs.fr3_connector.io.errors import (
    CommandRejected,
    DeviceUnavailable,
    GripperFailed,
    MotionInterrupted,
    OutcomeUnknown,
)
from unitelabs.fr3_connector.io.limits import READY_POSE
from unitelabs.fr3_connector.io.simulation import SimulatedFr3

PREGRASP = [0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785]


def simulated(**kwargs):
    return Fr3Protocol(simulation_time_scale=0.0, **kwargs)


class LosingBackend(SimulatedFr3):
    """Loses its connection in the middle of every motion."""

    async def move_joints(self, target, dynamics_factor, stop, on_progress):
        msg = "connection reset"
        raise DeviceUnavailable(msg)


def real(backend=None, **kwargs):
    backend = backend or SimulatedFr3(time_scale=0.0)
    kwargs.setdefault("robot_host", "192.0.2.10")
    return Fr3Protocol(simulation_mode=False, real_backend_factory=lambda: backend, **kwargs), backend


async def test_joint_motion_reaches_target_and_reports_progress():
    protocol = simulated()
    progress = []
    await protocol.move_to_joint_positions(PREGRASP, 0.1, lambda fraction, remaining: progress.append(fraction))
    assert (await protocol.read_state()).joint_positions == pytest.approx(PREGRASP)
    assert progress[-1] == 1.0
    assert not protocol.motion_active


async def test_joint_motion_validation():
    protocol = simulated(max_dynamics_factor=0.2)
    with pytest.raises(CommandRejected, match="Joint 4"):
        await protocol.move_to_joint_positions([0, 0, 0, 0, 0, 1.5, 0], 0.1)
    with pytest.raises(CommandRejected, match=r"at most 0\.2"):
        await protocol.move_to_joint_positions(PREGRASP, 0.5)
    with pytest.raises(CommandRejected, match="Joint must be"):
        await protocol.move_joint(8, 0.1, 0.1)
    with pytest.raises(CommandRejected, match="Joint 1"):
        await protocol.move_joint(1, 3.0, 0.1)
    assert (await protocol.read_state()).joint_positions == pytest.approx(READY_POSE)


async def test_relative_joint_move_changes_only_that_joint():
    protocol = simulated()
    await protocol.move_joint(7, -0.25, 0.1)
    joints = (await protocol.read_state()).joint_positions
    assert joints[6] == pytest.approx(READY_POSE[6] - 0.25)
    assert joints[:6] == pytest.approx(READY_POSE[:6])


async def test_linear_move_keeps_orientation():
    protocol = simulated()
    before = (await protocol.read_state()).end_effector_pose
    await protocol.move_linear([0.05, -0.1, -0.15], 0.1)
    after = (await protocol.read_state()).end_effector_pose
    assert after[:3] == pytest.approx([before[0] + 0.05, before[1] - 0.1, before[2] - 0.15], abs=1e-4)
    assert abs(float(np.dot(after[3:], before[3:]))) == pytest.approx(1.0, abs=1e-6)


async def test_pose_motion_reaches_rotated_target():
    protocol = simulated()
    yaw = math.radians(30)
    target = kinematics.pose_to_matrix([0.4, 0.1, 0.3], [math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0])
    pose = kinematics.matrix_to_pose(target)
    await protocol.move_to_pose([*pose[:3], *(2 * value for value in pose[3:])], 0.1)
    reached = kinematics.forward_kinematics((await protocol.read_state()).joint_positions)
    assert np.abs(kinematics.pose_error(target, reached)).max() < 1e-3


async def test_pose_motion_rejects_targets_outside_envelope():
    protocol = simulated(minimum_tcp_height=0.05)
    with pytest.raises(CommandRejected, match="below"):
        await protocol.move_to_pose([0.4, 0.0, 0.0, 1, 0, 0, 0], 0.1)
    with pytest.raises(CommandRejected, match="reach"):
        await protocol.move_to_pose([1.2, 0.0, 0.3, 1, 0, 0, 0], 0.1)
    with pytest.raises(CommandRejected, match="non-zero"):
        await protocol.move_to_pose([0.4, 0.0, 0.3, 0, 0, 0, 0], 0.1)


async def test_stop_interrupts_motion_and_frees_the_guard():
    protocol = Fr3Protocol(simulation_time_scale=1.0)
    motion = asyncio.create_task(protocol.move_joint(1, 1.0, 0.05))
    await asyncio.sleep(0.2)
    assert protocol.motion_active
    with pytest.raises(CommandRejected, match="in progress"):
        await protocol.move_joint(2, 0.1, 0.05)
    with pytest.raises(CommandRejected, match="Wait for the active motion"):
        await protocol.set_simulation_mode(False)
    await protocol.stop()
    with pytest.raises(MotionInterrupted):
        await motion
    reached = (await protocol.read_state()).joint_positions[0]
    assert 0.0 < reached < 1.0
    assert not protocol.motion_active
    await protocol.move_joint(1, -reached, 0.2)


async def test_named_positions():
    protocol = simulated(named_positions={"Pregrasp": PREGRASP})
    assert list(protocol.named_positions) == ["Home", "Pregrasp"]
    assert await protocol.current_named_position() == "Home"
    await protocol.move_to_named_position("Pregrasp")
    assert await protocol.current_named_position() == "Pregrasp"
    with pytest.raises(CommandRejected, match="Unknown position"):
        await protocol.move_to_named_position("Nowhere")


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError, match="Joint 4"):
        Fr3Protocol(named_positions={"Bad": [0.0] * 7})
    with pytest.raises(ValueError, match="default_dynamics_factor"):
        Fr3Protocol(default_dynamics_factor=0.5, max_dynamics_factor=0.2)


async def test_simulated_gripper():
    protocol = simulated(simulated_object_width=0.03)
    await protocol.grip()
    state = await protocol.read_gripper()
    assert state.width == pytest.approx(0.03)
    assert state.is_grasped
    await protocol.release()
    assert (await protocol.read_gripper()).width == pytest.approx(0.08)
    assert await protocol.gripper_grasp(0.03, 0.05, 20.0, 0.005, 0.005)
    await protocol.gripper_move(0.08, 0.05)
    assert not await protocol.gripper_grasp(0.05, 0.05, 20.0, 0.005, 0.005)
    with pytest.raises(CommandRejected, match="width"):
        await protocol.gripper_move(0.1, 0.05)


async def test_grip_without_object_fails():
    protocol = simulated(simulated_object_width=0.0)
    with pytest.raises(GripperFailed, match="without detecting"):
        await protocol.grip()


async def test_real_mode_is_read_only_by_default():
    protocol, backend = real()
    await protocol.open()
    assert not protocol.motion_enabled and not protocol.gripper_enabled
    assert (await protocol.read_state()).joint_positions == pytest.approx(READY_POSE)
    with pytest.raises(CommandRejected, match="enable_motion"):
        await protocol.move_to_joint_positions(PREGRASP, 0.05)
    with pytest.raises(CommandRejected, match="enable_gripper"):
        await protocol.grip()
    assert backend.joint_positions == pytest.approx(READY_POSE)


async def test_real_mode_requires_host():
    protocol = Fr3Protocol(simulation_mode=False)
    with pytest.raises(DeviceUnavailable, match="robot_host"):
        await protocol.open()
    simulation = simulated()
    with pytest.raises(DeviceUnavailable, match="robot_host"):
        await simulation.set_simulation_mode(False)
    assert simulation.simulation_mode


async def test_real_motion_and_mode_switching():
    protocol, backend = real(enable_motion=True)
    await protocol.open()
    await protocol.move_to_joint_positions(PREGRASP, 0.05)
    assert backend.joint_positions == pytest.approx(PREGRASP)
    await protocol.set_simulation_mode(True)
    assert protocol.simulator.joint_positions == pytest.approx(READY_POSE)
    await protocol.close()


async def test_robot_errors_block_motion_until_recovered():
    protocol, backend = real(enable_motion=True)
    await protocol.open()
    backend.has_errors = True
    with pytest.raises(CommandRejected, match="RecoverFromErrors"):
        await protocol.move_to_joint_positions(PREGRASP, 0.05)
    await protocol.recover_from_errors()
    await protocol.move_to_joint_positions(PREGRASP, 0.05)


async def test_lost_connection_marks_outcome_unknown_and_fails_closed():
    protocol, _ = real(LosingBackend(time_scale=0.0), enable_motion=True)
    await protocol.open()
    with pytest.raises(OutcomeUnknown, match="outcome is unknown"):
        await protocol.move_to_joint_positions(PREGRASP, 0.05)
    assert protocol.control_outcome_unknown
    with pytest.raises(CommandRejected, match="unknown outcome"):
        await protocol.move_to_joint_positions(PREGRASP, 0.05)
    await protocol.recover_from_errors()
    assert not protocol.control_outcome_unknown
