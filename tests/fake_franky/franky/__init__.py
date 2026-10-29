"""
Offline stand-in for the franky attributes this connector uses.

It is put first on the import path by the tests, so the real franky (if installed) is never contacted. Motions
complete on a background thread after `Robot.motion_seconds`; class-level `behavior` flags inject failures.
"""

import builtins
import enum
import threading
from typing import ClassVar

READY = [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483]


class Exception(builtins.Exception):  # noqa: A001
    """Base class of libfranka errors."""


class CommandException(Exception):
    """The robot rejected a command."""


class ControlException(Exception):
    """The control loop aborted, e.g. after a reflex."""


class NetworkException(Exception):
    """The connection to the robot failed."""


class ProtocolException(Exception):
    """The robot sent an unexpected message."""


class GripperException(builtins.Exception):
    """The gripper reported an error."""


class RealtimeConfig(enum.Enum):
    Enforce = 0
    Ignore = 1


class ReferenceType(enum.Enum):
    Absolute = 0
    Relative = 1


class RobotMode(enum.Enum):
    Idle = 0
    Move = 1
    Reflex = 2


class Affine:
    def __init__(self, translation=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
        self.translation = [float(value) for value in translation]
        self.quaternion = [float(value) for value in quaternion]


class JointMotion:
    def __init__(self, target):
        self.target = [float(value) for value in target]


class CartesianMotion:
    def __init__(self, target, reference_type=ReferenceType.Absolute):
        self.target = target
        self.reference_type = reference_type


class JointStopMotion:
    pass


class CartesianStopMotion:
    pass


class _State:
    def __init__(self, q, mode):
        self.q = list(q)
        self.dq = [0.0] * 7
        self.robot_mode = mode


class _Pose:
    def __init__(self, affine):
        self.end_effector_pose = affine


class Robot:
    instances: ClassVar[list["Robot"]] = []
    behavior: ClassVar[dict[str, bool]] = {}
    motion_seconds = 0.05

    def __init__(self, fci_hostname, realtime_config=RealtimeConfig.Enforce):
        if self.behavior.get("fail_connect"):
            msg = f"Connection to {fci_hostname} timed out"
            raise NetworkException(msg)
        self.fci_hostname = fci_hostname
        self.realtime_config = realtime_config
        self.relative_dynamics_factor = 1.0
        self.has_errors = False
        self.q = list(READY)
        self.pose = Affine([0.3069, 0.0, 0.4869], [1.0, 0.0, 0.0, 0.0])
        self.moves = []
        self.factors = []
        self.recoveries = 0
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._done.set()
        self._stopping = threading.Event()
        self._error = None
        Robot.instances.append(self)

    @property
    def state(self):
        return _State(self.q, RobotMode.Reflex if self.has_errors else RobotMode.Idle)

    @property
    def current_joint_positions(self):
        return list(self.q)

    @property
    def current_pose(self):
        return _Pose(self.pose)

    def move(self, motion, asynchronous=False):
        self.moves.append(motion)
        if isinstance(motion, (JointStopMotion, CartesianStopMotion)):
            self._stopping.set()
            return
        self.factors.append(self.relative_dynamics_factor)
        self._done.clear()
        self._stopping.clear()
        threading.Thread(target=self._run, args=(motion,), daemon=True).start()
        if not asynchronous:
            self.join_motion()

    def _run(self, motion):
        if not self._stopping.wait(self.motion_seconds):
            with self._lock:
                if self.behavior.get("reflex"):
                    self.has_errors = True
                    self._error = ControlException("cartesian_reflex: collision detected")
                elif self.behavior.get("network"):
                    self._error = NetworkException("UDP receive: Timeout")
                elif isinstance(motion, JointMotion):
                    self.q = list(motion.target)
                else:
                    self.pose = motion.target
        self._done.set()

    def join_motion(self, timeout=None):
        finished = self._done.wait(timeout)
        with self._lock:
            error, self._error = self._error, None
        if error is not None:
            raise error
        return finished

    def recover_from_errors(self):
        self.recoveries += 1
        self.has_errors = False

    def stop(self):
        self._stopping.set()


class Gripper:
    instances: ClassVar[list["Gripper"]] = []
    behavior: ClassVar[dict[str, bool]] = {}

    def __init__(self, fci_hostname):
        self.fci_hostname = fci_hostname
        self.width = 0.08
        self.max_width = 0.08
        self.is_grasped = False
        self.calls = []
        Gripper.instances.append(self)

    def move(self, width, speed):
        self.calls.append(("move", width, speed))
        self.width = width
        self.is_grasped = False
        return True

    def grasp(self, width, speed, force, epsilon_inner=0.005, epsilon_outer=0.005):
        self.calls.append(("grasp", width, speed, force, epsilon_inner, epsilon_outer))
        self.width = 0.025
        self.is_grasped = width - epsilon_inner <= self.width <= width + epsilon_outer
        return self.is_grasped

    def homing(self):
        self.calls.append(("homing",))
        if self.behavior.get("fault"):
            msg = "Gripper fault"
            raise CommandException(msg)
        self.width = self.max_width
        return True

    def stop(self):
        self.calls.append(("stop",))
        return True
