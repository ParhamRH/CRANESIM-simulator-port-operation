# camera_manager.py
# ─────────────────────────────────────────────────────────────────────────────
# Manages both the interactive debug camera and the synthetic (RGB/Depth/Seg)
# off-screen camera used for dataset generation.
# ─────────────────────────────────────────────────────────────────────────────

import pybullet as p

from config import (
    CAMERA_FAR, CAMERA_FOV, CAMERA_HEIGHT, CAMERA_NEAR,
    CAMERA_POSITION, CAMERA_TARGET, CAMERA_UPDATE_INTERVAL, CAMERA_WIDTH,
    DEBUG_CAM_DISTANCE, DEBUG_CAM_PITCH, DEBUG_CAM_TARGET, DEBUG_CAM_YAW,
)


class CameraManager:
    """Wraps all camera functionality for the crane simulator."""

    # Predefined debug-viewer presets
    VIEWS = {
        "top":   {"distance": 80, "yaw":   0, "pitch": -89, "target": [0, 20, 15]},
        "front": {"distance": 70, "yaw":   0, "pitch": -20, "target": [0, 20, 15]},
        "side":  {"distance": 70, "yaw":  90, "pitch": -20, "target": [0, 20, 15]},
        "iso":   {"distance": 80, "yaw":  45, "pitch": -30, "target": [0, 20, 15]},
        "crane": {"distance": 50, "yaw": 135, "pitch": -25, "target": [-10, 5, 20]},
    }
    VIEW_CYCLE = ["iso", "front", "side", "top", "crane"]

    def __init__(self):
        # Synthetic camera state
        self.cam_position = list(CAMERA_POSITION)
        self.cam_target   = list(CAMERA_TARGET)
        self._frame_count = 0
        self._view_idx    = 0

        # Enable buffer previews in the GUI
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,       1)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,     1)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)

        # Set initial debug-viewer position
        p.resetDebugVisualizerCamera(
            cameraDistance=DEBUG_CAM_DISTANCE,
            cameraYaw=DEBUG_CAM_YAW,
            cameraPitch=DEBUG_CAM_PITCH,
            cameraTargetPosition=DEBUG_CAM_TARGET,
        )

    # ── Synthetic camera ──────────────────────────────────────────────────────

    def tick(self):
        """Call once per simulation step; updates the synthetic camera periodically."""
        self._frame_count += 1
        if self._frame_count >= CAMERA_UPDATE_INTERVAL:
            self._render_synthetic()
            self._frame_count = 0

    def _render_synthetic(self):
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.cam_position,
            cameraTargetPosition=self.cam_target,
            cameraUpVector=[0, 0, 1],
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=CAMERA_FOV,
            aspect=CAMERA_WIDTH / CAMERA_HEIGHT,
            nearVal=CAMERA_NEAR,
            farVal=CAMERA_FAR,
        )
        p.getCameraImage(
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_TINY_RENDERER,
        )

    # ── Debug viewer controls ─────────────────────────────────────────────────

    def cycle_view(self):
        """Advance to the next preset view and return its name."""
        self._view_idx = (self._view_idx + 1) % len(self.VIEW_CYCLE)
        name = self.VIEW_CYCLE[self._view_idx]
        self.set_named_view(name)
        return name

    def set_named_view(self, name: str):
        if name not in self.VIEWS:
            return
        v = self.VIEWS[name]
        p.resetDebugVisualizerCamera(
            cameraDistance=v["distance"],
            cameraYaw=v["yaw"],
            cameraPitch=v["pitch"],
            cameraTargetPosition=v["target"],
        )

    def apply_keyboard_delta(self, yaw_delta=0, pitch_delta=0,
                             target_delta=None, cam_z_delta=0):
        """Nudge the debug camera based on keyboard input deltas."""
        cam = p.getDebugVisualizerCamera()
        yaw    = cam[8]  + yaw_delta
        pitch  = cam[9]  + pitch_delta
        dist   = cam[10]
        target = list(cam[11])

        if target_delta:
            target[0] += target_delta[0]
            target[1] += target_delta[1]

        self.cam_position[2] += cam_z_delta   # synthetic camera height

        p.resetDebugVisualizerCamera(
            cameraDistance=dist,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=target,
        )
