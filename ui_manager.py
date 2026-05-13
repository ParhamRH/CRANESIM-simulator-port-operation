# ui_manager.py
# ─────────────────────────────────────────────────────────────────────────────
# Creates and updates all PyBullet debug UI: parameter sliders, on-screen text,
# and the coloured waypoint-marker geometry.
# ─────────────────────────────────────────────────────────────────────────────

import pybullet as p

from config import CONTAINER_HEIGHT, WAYPOINTS


class UIManager:
    """Owns every debug-GUI element for the crane simulator."""

    def __init__(self):
        # ── Sliders ───────────────────────────────────────────────────────────
        self.auto_slider  = p.addUserDebugParameter("Start Automatic Cycle (0=Stop, 1=Start)", 0, 1, 0)
        self.speed_slider = p.addUserDebugParameter("Speed", 1, 10.0, 1)

        # ── Persistent text items (initialised to empty strings) ──────────────
        self._txt_phase   = p.addUserDebugText("", [0,    55, 48], textSize=1.5, textColorRGB=[1, 1, 1])
        self._txt_cable   = p.addUserDebugText("", [0.3,  55, 46.5], textSize=1.2, textColorRGB=[0, 0, 0])
        self._txt_trolley = p.addUserDebugText("", [0.6,  55, 45],   textSize=1.2, textColorRGB=[0, 0, 0])
        self._txt_gyro    = p.addUserDebugText("", [1.4,  55, 43.5], textSize=1.0, textColorRGB=[1, 0, 0])
        self._txt_acc     = p.addUserDebugText("", [1.5,  55, 42],   textSize=1.0, textColorRGB=[0, 0, 1])
        self._txt_rpy     = p.addUserDebugText("", [1.7,  55, 40.5], textSize=1.0, textColorRGB=[0, 1, 0])

        # Unused floating sensor texts (kept for compatibility)
        self._txt_f_gyro = p.addUserDebugText("", [30, -20, 34], textSize=1.0, textColorRGB=[1, 0, 0])
        self._txt_f_acc  = p.addUserDebugText("", [30, -20, 33], textSize=1.0, textColorRGB=[0, 1, 0])
        self._txt_f_rpy  = p.addUserDebugText("", [30, -20, 32], textSize=1.0, textColorRGB=[0, 0, 1])

        # ── Waypoint markers ──────────────────────────────────────────────────
        self._create_waypoint_markers()

    # ── Slider reads ──────────────────────────────────────────────────────────

    @property
    def auto_mode(self) -> bool:
        try:
            return p.readUserDebugParameter(self.auto_slider) > 0.5
        except Exception:
            return False

    @property
    def speed(self) -> int:
        try:
            return max(1, int(p.readUserDebugParameter(self.speed_slider)))
        except Exception:
            return 1

    # ── Status update ─────────────────────────────────────────────────────────

    def update_status(self, phase: str, attached: bool,
                      spreader_id: int, trolley_id: int,
                      waypoint_index: int, imu_data: dict):
        """Refresh every on-screen text element."""
        sp_pos, _ = p.getBasePositionAndOrientation(spreader_id)
        tr_pos, _ = p.getBasePositionAndOrientation(trolley_id)
        cable_len  = abs(tr_pos[2] - sp_pos[2])
        status_str = "ATTACHED" if attached else "DETACHED"

        p.addUserDebugText(
            f"Phase: {phase} | Status: {status_str}",
            [0, 55, 48], textColorRGB=[1, 1, 1], textSize=1.5,
            replaceItemUniqueId=self._txt_phase)

        p.addUserDebugText(
            f"Cable Length: {cable_len:.1f} m | Spreader Height: {sp_pos[2]:.1f} m",
            [0.3, 55, 46.5], textColorRGB=[0, 0, 0], textSize=1.2,
            replaceItemUniqueId=self._txt_cable)

        p.addUserDebugText(
            f"Trolley Y: {tr_pos[1]:.1f} | Waypoint: {waypoint_index + 1}/{len(WAYPOINTS)}",
            [0.6, 55, 45], textColorRGB=[0, 0, 0], textSize=1.2,
            replaceItemUniqueId=self._txt_trolley)

        # IMU readings
        p.addUserDebugText(
            f"Gyro: X={imu_data['gyro_x']:.3f}  Y={imu_data['gyro_y']:.3f}  Z={imu_data['gyro_z']:.3f} rad/s",
            [1.4, 55, 43.5], textColorRGB=[1, 0, 0], textSize=1.0,
            replaceItemUniqueId=self._txt_gyro)

        p.addUserDebugText(
            f"Acc: X={imu_data['acc_x']:.2f}  Y={imu_data['acc_y']:.2f}  Z={imu_data['acc_z']:.2f} m/s²",
            [1.5, 55, 42], textColorRGB=[0, 0, 1], textSize=1.0,
            replaceItemUniqueId=self._txt_acc)

        p.addUserDebugText(
            f"RPY: R={imu_data['roll']:.3f}  P={imu_data['pitch']:.3f}  Y={imu_data['yaw']:.3f} rad",
            [1.7, 55, 40.5], textColorRGB=[0, 1, 0], textSize=1.0,
            replaceItemUniqueId=self._txt_rpy)

    def show_camera_label(self, view_name: str):
        p.addUserDebugText(
            f"Camera: {view_name.upper()}",
            [0, 0, 45], textSize=1.5, textColorRGB=[1, 1, 0], lifeTime=2)

    # ── Waypoint markers ──────────────────────────────────────────────────────

    def _create_waypoint_markers(self):
        colors  = [[0.5, 0, 0.5, 0.5], [1, 0.5, 0.7, 0.5]]
        labels  = ["WP1", "WP2"]
        z_levels = [0.1, 24.6]   # WP1 at ground, WP2 elevated to ship deck

        for i, waypoint_y in enumerate(WAYPOINTS):
            marker_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[8, 2, 0.1],
                rgbaColor=colors[i],
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=marker_visual,
                basePosition=[0, waypoint_y, z_levels[i]],
                baseOrientation=[0, 0, 0, 1],
            )
            p.addUserDebugText(
                labels[i],
                [0, waypoint_y, z_levels[i]],
                textColorRGB=colors[i][:3],
                textSize=1.5,
                replaceItemUniqueId=200 + i,
            )
