class CommandRejected(Exception):
    """The request is invalid, disabled by configuration, or not permitted in the current robot state."""


class DeviceUnavailable(Exception):
    """The robot or gripper connection is closed or cannot be established."""


class MotionInterrupted(Exception):
    """A stop request ended the motion before it reached its target; the robot is stationary."""


class MotionFailed(Exception):
    """The robot aborted the motion, e.g. after a reflex or limit violation; inspect it and recover from errors."""


class GripperFailed(Exception):
    """The gripper did not complete the requested action."""


class OutcomeUnknown(Exception):
    """The connection was lost during an action; inspect the robot before sending another command."""
