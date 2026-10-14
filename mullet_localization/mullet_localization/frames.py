"""Converts an estimator's ROS odometry into PX4's external-vision convention.

ROS (REP 103): the estimator's world frame is gravity-aligned and z-up with an arbitrary heading,
its body frame is FLU, and the twist is expressed in the body (child) frame. PX4: world FRD with an
arbitrary heading and z down, body FRD. Both flips are the same 180 degree rotation about x.

Quaternions are (w, x, y, z), Hamiltonian, body to world, as in px4_msgs/VehicleOdometry.q.
No ROS imports, so the tests run anywhere numpy does.
"""

from __future__ import annotations

import numpy as np


def quat_mul(a, b) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def rotate(q, v) -> np.ndarray:
    """Rotates v by the unit quaternion q."""
    u = np.asarray(q[1:], dtype=float)
    v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(u, v)
    return v + q[0] * t + np.cross(u, t)


def flu_to_frd(v) -> np.ndarray:
    return np.array([v[0], -v[1], -v[2]])


def flu_quat_to_frd(q) -> np.ndarray:
    """The same attitude with both the world and the body frame flipped about x."""
    return np.array([q[0], q[1], -q[2], -q[3]])


def to_px4(p, q, v_body, w_body, body_t_base, body_q_base):
    """Pose and twist of the estimator body -> pose and twist of base_link, PX4 frames.

    Args:
      p, q: estimator body pose in its world frame (FLU world, FLU body).
      v_body, w_body: linear and angular velocity of the estimator body, in the body frame.
      body_t_base, body_q_base: pose of base_link (PX4's body) in the estimator body frame.

    Returns:
      (position, q, velocity, angular_velocity): base_link position, attitude and velocity in
      world FRD (VehicleOdometry POSE_FRAME_FRD / VELOCITY_FRAME_FRD), angular velocity in body
      FRD. The lever arm is applied here, so EKF2_EV_POS_X/Y/Z must stay 0.
    """
    q = np.asarray(q, dtype=float)
    w_body = np.asarray(w_body, dtype=float)
    p_base = np.asarray(p, dtype=float) + rotate(q, body_t_base)
    q_base = quat_mul(q, body_q_base)
    v_base = rotate(q, np.asarray(v_body, dtype=float) + np.cross(w_body, body_t_base))
    w_base = rotate(quat_conj(body_q_base), w_body)
    return flu_to_frd(p_base), flu_quat_to_frd(q_base), flu_to_frd(v_base), flu_to_frd(w_base)


def variances(pose_cov, twist_cov):
    """Diagonal variances (position, orientation, velocity) from ROS 6x6 row-major covariances.

    The x flip keeps a diagonal diagonal. The extrinsic rotation and the body-to-world rotation of
    the velocity do not, so orientation and velocity use the largest axis for all three.
    ponytail: conservative max, rotate the full 3x3 blocks if EV fusion turns out over-cautious.
    """
    pose_cov = np.asarray(pose_cov, dtype=float).reshape(6, 6)
    twist_cov = np.asarray(twist_cov, dtype=float).reshape(6, 6)
    pos = np.diag(pose_cov)[:3]
    att = np.full(3, np.diag(pose_cov)[3:].max())
    vel = np.full(3, np.diag(twist_cov)[:3].max())
    return pos, att, vel
