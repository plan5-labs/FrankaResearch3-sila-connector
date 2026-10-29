"""Nominal FR3 kinematics for simulation and pre-motion checks; not a calibrated or collision-aware model."""

import math

import numpy as np

from .limits import JOINT_LIMIT_MARGIN, JOINT_POSITION_MAX, JOINT_POSITION_MIN

DH_PARAMETERS = (
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -math.pi / 2),
    (0.0, 0.316, math.pi / 2),
    (0.0825, 0.0, math.pi / 2),
    (-0.0825, 0.384, -math.pi / 2),
    (0.0, 0.0, math.pi / 2),
    (0.088, 0.0, math.pi / 2),
)
"""Modified (Craig) Denavit-Hartenberg parameters (a_{i-1}, d_i, alpha_{i-1}) in metres and radians."""

FLANGE_OFFSET = 0.107
"""Distance from the joint 7 frame to the flange in metres."""

HAND_TCP_OFFSET = 0.1034
"""Distance from the flange to the Franka Hand fingertip centre in metres."""

HAND_TCP_ROTATION = -math.pi / 4
"""Rotation of the Franka Hand frame about the flange Z axis in radians."""


def _dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def _flange_to_tcp() -> np.ndarray:
    ct, st = math.cos(HAND_TCP_ROTATION), math.sin(HAND_TCP_ROTATION)
    return np.array(
        [
            [ct, -st, 0.0, 0.0],
            [st, ct, 0.0, 0.0],
            [0.0, 0.0, 1.0, FLANGE_OFFSET + HAND_TCP_OFFSET],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


_TCP = _flange_to_tcp()


def joint_frames(q: list[float] | np.ndarray) -> tuple[list[np.ndarray], np.ndarray]:
    """
    Compute every joint frame and the Franka Hand TCP frame in the base frame.

    Args:
      q: Seven joint angles in radians.

    Returns:
      The seven joint frames and the TCP frame as 4x4 homogeneous transforms.
    """

    frames = []
    transform = np.eye(4)
    for (a, d, alpha), theta in zip(DH_PARAMETERS, q, strict=True):
        transform = transform @ _dh_transform(a, d, alpha, float(theta))
        frames.append(transform)
    return frames, transform @ _TCP


def forward_kinematics(q: list[float] | np.ndarray) -> np.ndarray:
    """
    Compute the nominal Franka Hand TCP pose.

    Args:
      q: Seven joint angles in radians.

    Returns:
      The TCP pose as a 4x4 homogeneous transform in the base frame.
    """

    return joint_frames(q)[1]


def jacobian(q: list[float] | np.ndarray) -> np.ndarray:
    """
    Compute the geometric TCP Jacobian.

    Args:
      q: Seven joint angles in radians.

    Returns:
      A 6x7 matrix mapping joint velocities to linear and angular TCP velocity in the base frame.
    """

    frames, tcp = joint_frames(q)
    position = tcp[:3, 3]
    columns = []
    for frame in frames:
        axis, origin = frame[:3, 2], frame[:3, 3]
        columns.append(np.concatenate([np.cross(axis, position - origin), axis]))
    return np.array(columns).T


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """
    Convert a rotation matrix to a unit quaternion.

    Args:
      rotation: A 3x3 rotation matrix.

    Returns:
      The quaternion as (x, y, z, w) with a non-negative w.
    """

    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        quaternion = np.array([s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        quaternion = np.array([(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        quaternion = np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s])
    quaternion /= np.linalg.norm(quaternion)
    return -quaternion if quaternion[3] < 0 else quaternion


def quaternion_to_matrix(quaternion: list[float] | np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix.

    Args:
      quaternion: The quaternion as (x, y, z, w); it is normalized first.

    Returns:
      A 3x3 rotation matrix.

    Raises:
      ValueError: If the quaternion has zero length or contains non-finite values.
    """

    q = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        msg = "Orientation quaternion must be finite and non-zero."
        raise ValueError(msg)
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_to_matrix(position: list[float] | np.ndarray, quaternion: list[float] | np.ndarray) -> np.ndarray:
    """
    Build a homogeneous transform from a position and an orientation quaternion.

    Args:
      position: Translation (x, y, z) in metres.
      quaternion: Orientation as (x, y, z, w).

    Returns:
      A 4x4 homogeneous transform.
    """

    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_matrix(quaternion)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def matrix_to_pose(transform: np.ndarray) -> list[float]:
    """
    Split a homogeneous transform into position and quaternion.

    Args:
      transform: A 4x4 homogeneous transform.

    Returns:
      [x, y, z, qx, qy, qz, qw] with the position in metres.
    """

    return [*transform[:3, 3].tolist(), *matrix_to_quaternion(transform[:3, :3]).tolist()]


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """
    Compute the axis-angle vector of a rotation.

    Args:
      rotation: A 3x3 rotation matrix.

    Returns:
      The rotation axis scaled by the angle in radians, with the angle in [0, pi].
    """

    quaternion = matrix_to_quaternion(rotation)
    vector, w = quaternion[:3], quaternion[3]
    sine = float(np.linalg.norm(vector))
    if sine < 1e-12:
        return np.zeros(3)
    return vector / sine * 2.0 * math.atan2(sine, w)


def pose_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """
    Compute the six-dimensional error that moves the current pose onto the target.

    Args:
      target: Desired 4x4 transform.
      current: Actual 4x4 transform.

    Returns:
      Position error in metres followed by the orientation error as a rotation vector in radians.
    """

    return np.concatenate(
        [target[:3, 3] - current[:3, 3], rotation_vector(target[:3, :3] @ current[:3, :3].T)],
    )


def inverse_kinematics(
    target: np.ndarray,
    seed: list[float] | np.ndarray,
    *,
    max_iterations: int = 300,
    position_tolerance: float = 1e-5,
    orientation_tolerance: float = 1e-4,
    damping: float = 0.02,
) -> list[float] | None:
    """
    Solve the nominal TCP inverse kinematics near a seed configuration.

    Uses damped least squares and keeps every joint inside its limits; the redundant degree of freedom stays close
    to the seed.

    Args:
      target: Desired TCP pose as a 4x4 transform in the base frame.
      seed: Starting joint configuration in radians.
      max_iterations: Iteration budget.
      position_tolerance: Accepted position error in metres.
      orientation_tolerance: Accepted orientation error in radians.
      damping: Damping factor for steps near singular configurations.

    Returns:
      A joint configuration reaching the target, or None if none was found near the seed.
    """

    lower = np.array(JOINT_POSITION_MIN) + JOINT_LIMIT_MARGIN
    upper = np.array(JOINT_POSITION_MAX) - JOINT_LIMIT_MARGIN
    q = np.clip(np.asarray(seed, dtype=float), lower, upper)
    regularization = damping**2 * np.eye(6)
    for _ in range(max_iterations):
        error = pose_error(target, forward_kinematics(q))
        if np.linalg.norm(error[:3]) < position_tolerance and np.linalg.norm(error[3:]) < orientation_tolerance:
            return q.tolist()
        j = jacobian(q)
        step = j.T @ np.linalg.solve(j @ j.T + regularization, error)
        largest = float(np.max(np.abs(step)))
        if largest > 0.2:
            step *= 0.2 / largest
        q = np.clip(q + step, lower, upper)
    return None


def slerp(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    """
    Interpolate between two unit quaternions along the shortest arc.

    Args:
      start: Quaternion (x, y, z, w) at fraction 0.
      end: Quaternion (x, y, z, w) at fraction 1.
      fraction: Interpolation parameter in [0, 1].

    Returns:
      The interpolated unit quaternion.
    """

    dot = float(np.dot(start, end))
    if dot < 0.0:
        end, dot = -end, -dot
    if dot > 0.9995:
        result = start + fraction * (end - start)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    return (math.sin((1 - fraction) * angle) * start + math.sin(fraction * angle) * end) / math.sin(angle)


def plan_linear_path(start: list[float], target: np.ndarray, steps: int) -> list[list[float]] | None:
    """
    Sample joint configurations along a straight TCP line with interpolated orientation.

    Args:
      start: Current joint configuration in radians.
      target: Desired TCP pose as a 4x4 transform.
      steps: Number of samples after the start configuration.

    Returns:
      The sampled joint configurations ending at the target, or None if a sample has no nearby IK solution.
    """

    origin = forward_kinematics(start)
    start_quaternion = matrix_to_quaternion(origin[:3, :3])
    end_quaternion = matrix_to_quaternion(target[:3, :3])
    path = []
    seed = list(start)
    for index in range(1, steps + 1):
        fraction = index / steps
        sample = np.eye(4)
        sample[:3, :3] = quaternion_to_matrix(slerp(start_quaternion, end_quaternion, fraction))
        sample[:3, 3] = origin[:3, 3] + fraction * (target[:3, 3] - origin[:3, 3])
        solution = inverse_kinematics(sample, seed)
        if solution is None:
            return None
        path.append(solution)
        seed = solution
    return path


MINIMUM_JERK_PEAK_VELOCITY = 1.875
"""Peak of the normalized minimum-jerk velocity profile."""

MINIMUM_JERK_PEAK_ACCELERATION = 10.0 * math.sqrt(3.0) / 3.0
"""Peak of the normalized minimum-jerk acceleration profile."""


def minimum_jerk_duration(distance: float, velocity: float, acceleration: float) -> float:
    """
    Compute the shortest rest-to-rest minimum-jerk duration within velocity and acceleration limits.

    Args:
      distance: Absolute travel distance.
      velocity: Velocity limit.
      acceleration: Acceleration limit.

    Returns:
      The duration in seconds.
    """

    if distance <= 0.0:
        return 0.0
    return max(
        MINIMUM_JERK_PEAK_VELOCITY * distance / velocity,
        math.sqrt(MINIMUM_JERK_PEAK_ACCELERATION * distance / acceleration),
    )


def minimum_jerk(fraction: float) -> float:
    """
    Evaluate the normalized minimum-jerk profile.

    Args:
      fraction: Normalized time in [0, 1].

    Returns:
      Normalized position in [0, 1] with zero velocity and acceleration at both ends.
    """

    t = min(max(fraction, 0.0), 1.0)
    return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)
