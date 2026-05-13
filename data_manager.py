# data_manager.py
# ─────────────────────────────────────────────────────────────────────────────
# Collects per-step sensor readings and exports them to Excel (or CSV fallback).
# ─────────────────────────────────────────────────────────────────────────────

import os
from datetime import datetime

import pandas as pd
import pybullet as p

from config import CONTAINER_LOAD, CONTAINER_TYPE, OUTPUT_DIR


class DataManager:
    """Accumulates simulation data and writes it to disk on request.

    Parameters
    ----------
    container_imu : IMUSensor
        The sensor attached to the container body.
    spreader_id, container_id, trolley_id : int
        PyBullet body IDs used to read pose data each step.
    """

    def __init__(self, container_imu, spreader_id: int,
                 container_id: int, trolley_id: int):
        self.imu          = container_imu
        self.spreader_id  = spreader_id
        self.container_id = container_id
        self.trolley_id   = trolley_id

        self.container_type = CONTAINER_TYPE
        self.container_load = CONTAINER_LOAD

        self._rows: list[dict] = []

    # ── Per-step collection ───────────────────────────────────────────────────

    def collect(self, phase: str, attached: bool):
        """Append one data row for the current simulation step."""
        imu = self.imu.get_measurements()

        sp_pos, _  = p.getBasePositionAndOrientation(self.spreader_id)
        c_pos,  _  = p.getBasePositionAndOrientation(self.container_id)
        tr_pos, _  = p.getBasePositionAndOrientation(self.trolley_id)

        # Simplified cable-tension estimate
        cable_tension = self.container_load * 9.81
        if phase == "LIFTING":
            cable_tension *= 1.2
        elif phase == "LOWERING_FINAL":
            cable_tension *= 1.1

        self._rows.append({
            "timestamp":      imu["timestamp"],
            "phase":          phase,
            "attached":       attached,
            # Container pose
            "container_x":    c_pos[0],
            "container_y":    c_pos[1],
            "container_z":    c_pos[2],
            # Trolley / spreader
            "trolley_y":      tr_pos[1],
            "spreader_height": sp_pos[2],
            # Gyroscope
            "gyro_x": imu["gyro_x"], "gyro_y": imu["gyro_y"], "gyro_z": imu["gyro_z"],
            # Accelerometer
            "acc_x":  imu["acc_x"],  "acc_y":  imu["acc_y"],  "acc_z":  imu["acc_z"],
            # Magnetometer
            "mag_x":  imu["mag_x"],  "mag_y":  imu["mag_y"],  "mag_z":  imu["mag_z"],
            # Orientation
            "roll":  imu["roll"], "pitch": imu["pitch"], "yaw": imu["yaw"],
            "altitude": imu["altitude"],
            # Cable
            "cable_tension": cable_tension,
            # Metadata
            "container_type": self.container_type,
            "container_load": self.container_load,
            "imu_location_x": self.imu.local_position[0],
            "imu_location_y": self.imu.local_position[1],
            "imu_location_z": self.imu.local_position[2],
        })

    # ── Export ────────────────────────────────────────────────────────────────

    def save(self, filename: str | None = None) -> str | None:
        """Write collected rows to an Excel file (CSV fallback).

        Returns the path of the file written, or None if no data exists.
        """
        print(f"Saving dataset – {len(self._rows)} rows collected.")

        if not self._rows:
            print("Warning: no data to save.")
            return None

        if filename is None:
            ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"container_imu_data_{ts}.xlsx"

        output_dir = os.path.expanduser(OUTPUT_DIR)
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)

        df = pd.DataFrame(self._rows)

        try:
            df.to_excel(filepath, index=False)
            print(f"Saved {len(df)} rows → {filepath}")
            return filepath
        except Exception as exc:
            print(f"Excel export failed ({exc}), falling back to CSV.")
            csv_path = filepath.replace(".xlsx", ".csv")
            df.to_csv(csv_path, index=False)
            print(f"Saved {len(df)} rows → {csv_path}")
            return csv_path

    # ── Convenience ───────────────────────────────────────────────────────────

    @property
    def row_count(self) -> int:
        return len(self._rows)
