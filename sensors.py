# sensors.py
# ─────────────────────────────────────────────────────────────────────────────
# IMU sensor simulation: gyroscope, accelerometer, magnetometer, barometer.
# ─────────────────────────────────────────────────────────────────────────────

import time
from datetime import datetime

import numpy as np
import pybullet as p

from config import IMU_NOISE_LEVEL, IMU_LOCAL_POSITION


class IMUSensor:
    """Simulates an IMU sensor attached to a PyBullet rigid body.

    Provides:
      - Gyroscope      – angular velocity (rad/s) with Gaussian noise
      - Accelerometer  – specific force including gravity (m/s²)
      - Magnetometer   – Earth's magnetic field projected into body frame (µT)
      - Barometer      – altitude derived from body Z position (m)
    """

    GRAVITY          = 9.81
    SEA_LEVEL_PRESSURE = 1013.25          # hPa (unused but kept for completeness)
    MAGNETIC_FIELD   = np.array([25.0, 0.0, 40.0])   # µT – simplified Earth field

    def __init__(
        self,
        body_id: int,
        local_position: list | None = None,
        noise_level: float = IMU_NOISE_LEVEL,
    ):
        self.body_id        = body_id
        self.local_position = local_position if local_position is not None else IMU_LOCAL_POSITION
        self.noise_level    = noise_level
        self._last_time     = time.time()

    # ── Public API ────────────────────────────────────────────────────────────

    def get_measurements(self) -> dict:
        """Return a dict of all sensor channels at the current simulation state."""
        current_time = time.time()
        self._last_time = current_time

        pos, orn     = p.getBasePositionAndOrientation(self.body_id)
        _, ang_vel   = p.getBaseVelocity(self.body_id)

        euler        = p.getEulerFromQuaternion(orn)
        roll, pitch, yaw = euler

        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

        # ── Gyroscope ─────────────────────────────────────────────────────────
        gyro = np.array(ang_vel) + np.random.normal(0, self.noise_level, 3)

        # ── Accelerometer (gravity in body frame, no translational acc here) ──
        gravity_body = rot.T @ np.array([0.0, 0.0, -self.GRAVITY])
        acc = gravity_body + np.random.normal(0, self.noise_level * 10, 3)

        # ── Magnetometer ──────────────────────────────────────────────────────
        mag = rot.T @ self.MAGNETIC_FIELD + np.random.normal(0, self.noise_level * 5, 3)

        # ── Barometer / altitude ──────────────────────────────────────────────
        altitude = pos[2] + np.random.normal(0, self.noise_level * 2)

        return {
            "timestamp": datetime.now(),
            # Gyroscope
            "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
            # Accelerometer
            "acc_x": acc[0],  "acc_y": acc[1],  "acc_z": acc[2],
            # Magnetometer
            "mag_x": mag[0],  "mag_y": mag[1],  "mag_z": mag[2],
            # Orientation (Euler)
            "roll": roll, "pitch": pitch, "yaw": yaw,
            # Orientation (Quaternion)
            "quat_x": orn[0], "quat_y": orn[1], "quat_z": orn[2], "quat_w": orn[3],
            # Barometer
            "altitude": altitude,
        }
