"""
Exercise a running FR3 connector over SiLA: read state, move, stop, grip.

Start the simulated connector first:

    uv run python -m unitelabs.fr3_connector --config-path simulation.example.json

Then run:

    uv run python examples/demo_client.py --port 50052

Against a physical robot, review every motion below for your cell before running it.
"""

import argparse
import asyncio

from sila.client import Client, ClientConfig

ROBOT = "io.unitelabs/robot/RobotController/v1"
MOVEMENT = "io.unitelabs/robot/MovementController/v1"
GRIP = "io.unitelabs/robot/GripController/v1"
SIMULATION = "org.silastandard/core/SimulationController/v1"


async def observe(command, parameters):
    execution = await command.execute(parameters)
    async for status in await execution.subscribe_status():
        if status.progress is not None:
            print(f"  progress {status.progress.value:6.1%}", end="\r")
    print()
    return await execution.get_responses()


async def main(host: str, port: int, factor: float) -> None:
    client = Client(ClientConfig(host, port, tls=False, default_timeout=10000))
    await client.open()
    try:
        simulation = await client.load_feature(SIMULATION)
        robot = await client.load_feature(ROBOT)
        movement = await client.load_feature(MOVEMENT)
        grip = await client.load_feature(GRIP)

        print("Simulation mode:", await simulation.properties["SimulationMode"].read())
        print("Robot mode:", await robot.properties["RobotMode"].read())
        print("Joints [rad]:", [round(q, 3) for q in await robot.properties["JointPositions"].read()])
        print("TCP pose:", await robot.properties["EndEffectorPose"].read())
        if not await robot.properties["MotionEnabled"].read():
            print("Motion is disabled in this configuration; stopping after the read-only checks.")
            return

        print("Moving to Home")
        await observe(movement.commands["MoveTo"], {"TargetPosition": {"Position": "Home", "SubPosition": 1}})

        print("Lowering the TCP by 10 cm along a straight line")
        offset = {"Dx": 0.0, "Dy": 0.0, "Dz": -0.1, "RelativeDynamicsFactor": factor}
        await observe(robot.commands["MoveLinear"], offset)

        print("Gripping and releasing")
        await grip.commands["Grip"].execute({})
        await grip.commands["Release"].execute({})

        print("Rotating joint 1 and stopping half-way")
        motion = asyncio.create_task(
            observe(robot.commands["MoveJoint"], {"Joint": 1, "Delta": 0.5, "RelativeDynamicsFactor": factor})
        )
        await asyncio.sleep(1.0)
        await robot.commands["Stop"].execute({})
        try:
            await motion
        except Exception as error:  # noqa: BLE001
            print(f"  ended with {type(error).__name__}: {error}")

        print("Returning to Home")
        await observe(movement.commands["MoveTo"], {"TargetPosition": {"Position": "Home", "SubPosition": 1}})
    finally:
        await client.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50052)
    parser.add_argument("--dynamics-factor", type=float, default=0.05)
    arguments = parser.parse_args()
    asyncio.run(main(arguments.host, arguments.port, arguments.dynamics_factor))
