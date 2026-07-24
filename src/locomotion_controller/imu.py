"""IMU transforms used by the policy observation."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def get_gravity_orientation(quaternion: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = quaternion
    return np.asarray(
        (
            2.0 * (-qz * qx + qw * qy),
            -2.0 * (qz * qy + qw * qx),
            1.0 - 2.0 * (qw * qw + qz * qz),
        ),
        dtype=np.float32,
    )


def transform_torso_imu(
    waist_yaw: float,
    waist_yaw_velocity: float,
    quaternion: np.ndarray,
    angular_velocity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    waist_rotation = Rotation.from_euler("z", waist_yaw).as_matrix()
    torso_rotation = Rotation.from_quat(
        [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    ).as_matrix()
    pelvis_rotation = torso_rotation @ waist_rotation.T
    pelvis_angular_velocity = (
        waist_rotation @ angular_velocity
        - np.asarray([0.0, 0.0, waist_yaw_velocity])
    )
    pelvis_quaternion = Rotation.from_matrix(pelvis_rotation).as_quat()[
        [3, 0, 1, 2]
    ]
    return pelvis_quaternion, pelvis_angular_velocity
