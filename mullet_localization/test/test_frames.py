import math

import numpy as np

from mullet_localization.frames import (
    flu_quat_to_frd, flu_to_frd, quat_conj, quat_mul, rotate, to_px4, variances)

IDENTITY = [1.0, 0.0, 0.0, 0.0]
ZERO = [0.0, 0.0, 0.0]


def yaw(rad):
    return np.array([math.cos(rad / 2), 0.0, 0.0, math.sin(rad / 2)])


def test_flip_commutes_with_rotation():
    # Rotating then flipping must equal flipping the attitude and the vector, for any attitude.
    rng = np.random.default_rng(0)
    for _ in range(100):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        v = rng.normal(size=3)
        np.testing.assert_allclose(
            flu_to_frd(rotate(q, v)), rotate(flu_quat_to_frd(q), flu_to_frd(v)), atol=1e-12)


def test_yaw_left_is_negative_frd_yaw():
    # Facing +y (left) at (1, 2, 3) in FLU: PX4 sees (1, -2, -3), heading -90 degrees.
    p, q, v, w = to_px4([1, 2, 3], yaw(math.pi / 2), [1, 0, 0], ZERO, ZERO, IDENTITY)
    np.testing.assert_allclose(p, [1, -2, -3])
    np.testing.assert_allclose(q, yaw(-math.pi / 2), atol=1e-12)
    # 1 m/s forward in the body is 1 m/s along world +y, so -y in FRD.
    np.testing.assert_allclose(v, [0, -1, 0], atol=1e-12)


def test_lever_arm():
    # base_link 1 m ahead of the estimator body, yawing left at 1 rad/s, body not translating.
    p, q, v, w = to_px4(ZERO, IDENTITY, ZERO, [0, 0, 1], [1, 0, 0], IDENTITY)
    np.testing.assert_allclose(p, [1, 0, 0])
    np.testing.assert_allclose(v, [0, -1, 0], atol=1e-12)  # moving left
    np.testing.assert_allclose(w, [0, 0, -1], atol=1e-12)  # yaw rate about down


def test_extrinsic_rotation():
    # Estimator body rolled 90 degrees from base_link: its z is base_link's -y.
    roll = np.array([math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0])
    body_q_base = quat_conj(roll)
    p, q, v, w = to_px4(ZERO, roll, ZERO, [0, 0, 1], ZERO, body_q_base)
    np.testing.assert_allclose(q, IDENTITY, atol=1e-12)  # base_link level
    np.testing.assert_allclose(w, [0, 1, 0], atol=1e-12)  # FLU -y is FRD +y
    np.testing.assert_allclose(quat_mul(roll, body_q_base), IDENTITY, atol=1e-12)


def test_variances():
    pose = np.diag([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
    twist = np.diag([0.4, 0.5, 0.6, 0.0, 0.0, 0.0])
    pos, att, vel = variances(pose.ravel(), twist.ravel())
    np.testing.assert_allclose(pos, [1, 2, 3])
    np.testing.assert_allclose(att, [0.3] * 3)
    np.testing.assert_allclose(vel, [0.6] * 3)
