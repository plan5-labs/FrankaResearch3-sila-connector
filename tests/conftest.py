import asyncio
import json
import os
from pathlib import Path
import signal
import socket
import sys

import pytest
from sila.client import Client, ClientConfig

FAKE_FRANKY = Path(__file__).parent / "fake_franky"

ROBOT = "io.unitelabs/robot/RobotController/v1"
HAND = "io.unitelabs/robot/FrankaHandController/v1"
GRIP = "io.unitelabs/robot/GripController/v1"
MOVEMENT = "io.unitelabs/robot/MovementController/v1"
SIMULATION = "org.silastandard/core/SimulationController/v1"
LOCK = "org.silastandard/core/LockController/v2"


@pytest.fixture
async def sila_client(tmp_path, request):
    """Start the connector in a subprocess; `request.param` holds configuration overrides."""

    overrides = getattr(request, "param", None) or {}
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = tmp_path / "local.json"
    config.write_text(json.dumps({
        "simulation_mode": True,
        "simulation_time_scale": 0.0,
        "sila_server": {"hostname": "127.0.0.1", "port": port, "tls": False},
        "cloud_server_endpoint": [], "discovery": None,
        **overrides,
    }))
    # The fake franky shadows any installed franky, so real mode never reaches a physical robot.
    env = {**os.environ, "PYTHONPATH": os.pathsep.join([str(FAKE_FRANKY), str(Path("src").resolve())])}
    with (tmp_path / "server.log").open("w+") as log:
        process = await asyncio.create_subprocess_exec(
            sys.executable, "-m", "unitelabs.fr3_connector", "--config-path", str(config),
            stdout=log, stderr=log, env=env,
        )
        client = Client(ClientConfig("127.0.0.1", port, tls=False, default_timeout=5000))
        try:
            skipped = await asyncio.wait_for(client.open(), timeout=15)
            assert skipped == []
            yield client
        finally:
            await client.close()
            expected_returncode = 0
            if process.returncode is None:
                if sys.platform == "win32":
                    process.terminate()
                    expected_returncode = 1
                else:
                    process.send_signal(signal.SIGINT)
                try:
                    await asyncio.wait_for(process.wait(), timeout=10)
                except TimeoutError:
                    process.kill()
                    await process.wait()
            log.seek(0)
            output = log.read()
            assert process.returncode == expected_returncode, output
            assert "Duplicate handler" not in output, output
            assert "UserWarning" not in output, output


@pytest.fixture
def observe():
    """Execute an observable command, wait for it to finish, and return its responses and progress values."""

    async def run(command, parameters):
        execution = await command.execute(parameters)
        progress = []
        async for status in await execution.subscribe_status():
            if status.progress is not None:
                progress.append(status.progress.value)
        return await execution.get_responses(), progress

    return run


@pytest.fixture
def fake_franky(monkeypatch):
    """Import the offline franky stand-in for in-process backend tests."""

    monkeypatch.syspath_prepend(str(FAKE_FRANKY))
    monkeypatch.delitem(sys.modules, "franky", raising=False)
    import franky

    franky.Robot.instances.clear()
    franky.Robot.behavior.clear()
    franky.Robot.motion_seconds = 0.05
    franky.Gripper.instances.clear()
    franky.Gripper.behavior.clear()
    yield franky
    sys.modules.pop("franky", None)
