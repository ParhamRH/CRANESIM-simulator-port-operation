# simulator.py
# ─────────────────────────────────────────────────────────────────────────────
# ContainerSimulator: wires every module together and owns the main loop.
# ─────────────────────────────────────────────────────────────────────────────

import time

import pybullet as p
import pybullet_data

from config import (
    CONTAINER_LENGTH, CONTAINER_WIDTH, GRAVITY,
    IMU_LOCAL_POSITION, IMU_NOISE_LEVEL, SIMULATION_HZ,
    SPREADER_ANGULAR_DAMPING, SPREADER_LINEAR_DAMPING,
    CONTAINER_LINEAR_DAMPING, CONTAINER_ANGULAR_DAMPING,
)
from sensors           import IMUSensor
from crane_objects     import (load_plane, load_crane, load_ship,
                                create_container, create_trolley,
                                create_spreader, create_spreader_trolley_constraint)
from physics_controller import PhysicsController
from camera_manager    import CameraManager
from ui_manager        import UIManager
from data_manager      import DataManager


class ContainerSimulator:
    """Top-level orchestrator for the Panamax crane simulation."""

    def __init__(self):
        # ── PyBullet setup ────────────────────────────────────────────────────
        self._physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, GRAVITY)

        # ── Scene objects ─────────────────────────────────────────────────────
        self.plane_id     = load_plane()
        self.ship_id      = load_ship()
        self.crane_id     = load_crane()
        self.container_id = create_container()
        self.trolley_id   = create_trolley()
        self.spreader_id  = create_spreader()

        # Spreader ↔ trolley prismatic constraint
        self._spreader_trolley_constraint = create_spreader_trolley_constraint(
            self.trolley_id, self.spreader_id)

        # Fine-tune dynamics
        p.changeDynamics(self.spreader_id,  -1,
                         linearDamping=SPREADER_LINEAR_DAMPING,
                         angularDamping=SPREADER_ANGULAR_DAMPING)
        p.changeDynamics(self.container_id, -1,
                         linearDamping=CONTAINER_LINEAR_DAMPING,
                         angularDamping=CONTAINER_ANGULAR_DAMPING)

        # ── Sub-systems ───────────────────────────────────────────────────────
        self.camera  = CameraManager()
        self.ui      = UIManager()

        self.imu = IMUSensor(
            body_id=self.container_id,
            local_position=IMU_LOCAL_POSITION,
            noise_level=IMU_NOISE_LEVEL,
        )

        self.physics = PhysicsController(
            trolley_id=self.trolley_id,
            spreader_id=self.spreader_id,
            container_id=self.container_id,
        )

        self.data = DataManager(
            container_imu=self.imu,
            spreader_id=self.spreader_id,
            container_id=self.container_id,
            trolley_id=self.trolley_id,
        )

        print("Simulation ready. Set 'Start Automatic Cycle' slider to 1 to begin.")

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        self._print_controls()

        consecutive_errors = 0
        running = True

        try:
            while running:
                try:
                    keys = p.getKeyboardEvents()

                    # Quit
                    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                        self.data.save()
                        running = False
                        break

                    # Manual save
                    if ord('z') in keys and keys[ord('z')] & p.KEY_WAS_TRIGGERED:
                        self.data.save()
                        print("Dataset saved manually.")

                    # ── Camera keyboard controls ──────────────────────────────
                    yaw_delta = pitch_delta = 0
                    target_delta = [0, 0]
                    cam_z_delta = 0

                    if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                        name = self.camera.cycle_view()
                        self.ui.show_camera_label(name)

                    if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
                        yaw_delta -= 5
                    if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:
                        yaw_delta += 5
                    if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:
                        pitch_delta -= 5
                    if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
                        pitch_delta += 5
                    if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:
                        cam_z_delta += 1
                    if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:
                        cam_z_delta -= 1
                    if p.B3G_LEFT_ARROW  in keys and keys[p.B3G_LEFT_ARROW]  & p.KEY_IS_DOWN:
                        target_delta[0] -= 0.9
                    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                        target_delta[0] += 0.9
                    if p.B3G_UP_ARROW    in keys and keys[p.B3G_UP_ARROW]    & p.KEY_IS_DOWN:
                        target_delta[1] += 0.9
                    if p.B3G_DOWN_ARROW  in keys and keys[p.B3G_DOWN_ARROW]  & p.KEY_IS_DOWN:
                        target_delta[1] -= 0.9

                    if any([yaw_delta, pitch_delta, target_delta[0],
                            target_delta[1], cam_z_delta]):
                        self.camera.apply_keyboard_delta(
                            yaw_delta, pitch_delta, target_delta, cam_z_delta)

                    # ── Simulation tick ───────────────────────────────────────
                    auto_mode = self.ui.auto_mode
                    speed     = self.ui.speed

                    self.physics.update_cables()

                    if auto_mode:
                        self.physics.step(speed)
                        self.data.collect(self.physics.phase, self.physics.attached)
                    else:
                        self.physics.reset()

                    # Read latest IMU data for the UI (1 call, reused)
                    imu_data = self.imu.get_measurements()

                    self.ui.update_status(
                        phase=self.physics.phase,
                        attached=self.physics.attached,
                        spreader_id=self.spreader_id,
                        trolley_id=self.trolley_id,
                        waypoint_index=self.physics.current_waypoint_index,
                        imu_data=imu_data,
                    )

                    self.camera.tick()
                    p.stepSimulation()
                    time.sleep(1.0 / SIMULATION_HZ)

                    consecutive_errors = 0

                except Exception as exc:
                    consecutive_errors += 1
                    print(f"Loop error #{consecutive_errors}: {exc}")
                    if consecutive_errors > 10:
                        print("Too many consecutive errors – stopping.")
                        break
                    time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            self.data.save()
            p.disconnect()

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _print_controls():
        print("\n" + "=" * 60)
        print("  PANAMAX CRANE CONTAINER LOADING SIMULATOR")
        print("=" * 60)
        print("  Slider  – Start / Stop automatic cycle")
        print("  C       – Cycle camera views")
        print("  J / L   – Rotate camera left / right")
        print("  I / K   – Rotate camera up / down")
        print("  Arrows  – Pan camera")
        print("  U / H   – Move synthetic camera up / down")
        print("  Z       – Save dataset now")
        print("  Q       – Quit (auto-saves)")
        print("=" * 60 + "\n")
