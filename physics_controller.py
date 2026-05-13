# physics_controller.py
# ─────────────────────────────────────────────────────────────────────────────
# All motion logic: spreader height control, trolley movement, cable visuals,
# and the full automatic-cycle state machine.
# ─────────────────────────────────────────────────────────────────────────────

import pybullet as p

from config import (
    ATTACH_WAIT_STEPS, CONSTRAINT_MAX_FORCE,
    CONTAINER_HEIGHT, CONTAINER_LENGTH, CONTAINER_WIDTH,
    INITIAL_SPREADER_HEIGHT, LIFTING_TARGET_HEIGHT,
    LOWERING_FINAL_HEIGHT,
    SPREADER_DAMPING_COEFF, SPREADER_FORCE_ATTACHED, SPREADER_FORCE_FREE,
    STABILIZE_STEPS, WAYPOINTS,
)


class PhysicsController:
    """Drives all crane motion and manages the automatic loading cycle.

    Parameters
    ----------
    trolley_id, spreader_id, container_id : int
        PyBullet body IDs created by crane_objects.py.
    """

    def __init__(self, trolley_id: int, spreader_id: int, container_id: int):
        self.trolley_id   = trolley_id
        self.spreader_id  = spreader_id
        self.container_id = container_id

        # State
        self.phase                  = "WAITING"
        self.phase_timer            = 0
        self.attached               = False
        self.container_constraint   = None
        self.target_spreader_height = INITIAL_SPREADER_HEIGHT
        self.current_waypoint_index = 0
        self.waypoint_timer         = 0
        self.waypoint_reached       = False

        # Cable line visuals (4 corners)
        self.cable_visuals = [
            p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0.2, 0.2, 0.2], lineWidth=3)
            for _ in range(4)
        ]

    # ── Height control ────────────────────────────────────────────────────────

    def set_spreader_height(self, target_height: float, speed_factor: float) -> bool:
        """Apply forces to move the spreader toward *target_height*.

        Returns True when within 0.2 m of the target.
        """
        pos, _ = p.getBasePositionAndOrientation(self.spreader_id)
        diff   = target_height - pos[2]

        if abs(diff) > 0.2:
            multiplier = SPREADER_FORCE_ATTACHED if self.attached else SPREADER_FORCE_FREE
            force_z    = diff * multiplier * speed_factor
            p.applyExternalForce(self.spreader_id, -1, [0, 0, force_z],    [0, 0, 0], p.WORLD_FRAME)

            velocity    = p.getBaseVelocity(self.spreader_id)[0][2]
            damping     = -velocity * SPREADER_DAMPING_COEFF
            p.applyExternalForce(self.spreader_id, -1, [0, 0, damping],    [0, 0, 0], p.WORLD_FRAME)
            return False
        return True

    # ── Trolley movement ──────────────────────────────────────────────────────

    def move_trolley_to_waypoint(self, waypoint_index: int, speed: float) -> bool:
        """Kinematically slide the trolley toward *waypoint_index*.

        Returns True when within 0.05 m.
        """
        if waypoint_index >= len(WAYPOINTS):
            return True

        target_y = float(WAYPOINTS[waypoint_index])
        pos, orn = p.getBasePositionAndOrientation(self.trolley_id)
        current_y = pos[1]

        if abs(current_y - target_y) > 0.05:
            step  = min(speed * 0.01, abs(target_y - current_y))
            new_y = current_y + step * (1 if target_y > current_y else -1)
            p.resetBasePositionAndOrientation(self.trolley_id, [pos[0], new_y, pos[2]], orn)
            return False
        return True

    # ── Cable visuals ─────────────────────────────────────────────────────────

    def update_cables(self):
        """Redraw the four corner cables between trolley and spreader."""
        spreader_pos, spreader_orn = p.getBasePositionAndOrientation(self.spreader_id)
        trolley_pos,  _            = p.getBasePositionAndOrientation(self.trolley_id)

        hx = CONTAINER_LENGTH / 2 + 0.3
        hy = CONTAINER_WIDTH  / 2 + 0.3
        corners = [[ hx,  hy, 0], [-hx,  hy, 0], [ hx, -hy, 0], [-hx, -hy, 0]]
        trolley_attach_z = trolley_pos[2] - 0.5

        for i, corner in enumerate(corners):
            world_pt = p.multiplyTransforms(spreader_pos, spreader_orn, corner, [0, 0, 0, 1])[0]
            trolley_pt = [trolley_pos[0] + corner[0], trolley_pos[1] + corner[1], trolley_attach_z]
            p.addUserDebugLine(trolley_pt, world_pt, [0.2, 0.2, 0.2],
                               lineWidth=3, replaceItemUniqueId=self.cable_visuals[i])

    # ── Synchronise spreader / container with trolley during MOVING ───────────

    def _sync_to_trolley(self):
        trolley_pos, _ = p.getBasePositionAndOrientation(self.trolley_id)
        sp_pos, sp_orn = p.getBasePositionAndOrientation(self.spreader_id)
        y_diff = trolley_pos[1] - sp_pos[1]

        if abs(y_diff) > 0.01:
            p.resetBasePositionAndOrientation(
                self.spreader_id, [sp_pos[0], trolley_pos[1], sp_pos[2]], sp_orn)

            if self.attached:
                c_pos, c_orn = p.getBasePositionAndOrientation(self.container_id)
                p.resetBasePositionAndOrientation(
                    self.container_id,
                    [c_pos[0], c_pos[1] + y_diff, c_pos[2]],
                    c_orn)

    # ── Constraint helpers ────────────────────────────────────────────────────

    def _attach_container(self):
        self.container_constraint = p.createConstraint(
            self.spreader_id,  -1,
            self.container_id, -1,
            p.JOINT_FIXED,
            [0, 0, -0.3],
            [0, 0, CONTAINER_HEIGHT / 2],
            [0, 0, 0],
        )
        p.changeConstraint(self.container_constraint, maxForce=CONSTRAINT_MAX_FORCE)
        self.attached = True

    def _detach_container(self):
        if self.container_constraint is not None:
            try:
                p.removeConstraint(self.container_constraint)
            except Exception:
                pass
            self.container_constraint = None
        self.attached = False

    def _rebuild_constraint_clean(self):
        """Re-create the constraint with perfectly aligned frames (used in STABILIZING)."""
        self._detach_container()
        self.container_constraint = p.createConstraint(
            self.spreader_id,  -1,
            self.container_id, -1,
            p.JOINT_FIXED,
            [0, 0, -0.6],
            [0, 0, CONTAINER_HEIGHT / 2],
            [0, 0, 0, 1],
            [0, 0, 0, 1],
        )
        p.changeConstraint(self.container_constraint, maxForce=100_000_000)
        self.attached = True

    # ── Main state machine ────────────────────────────────────────────────────

    def step(self, speed: float):
        """Advance the automatic cycle by one simulation step."""
        self.phase_timer += 1
        spreader_pos, _ = p.getBasePositionAndOrientation(self.spreader_id)

        # ── WAITING ──────────────────────────────────────────────────────────
        if self.phase == "WAITING":
            self.phase                  = "POSITIONING"
            self.phase_timer            = 0
            self.target_spreader_height = INITIAL_SPREADER_HEIGHT
            self.current_waypoint_index = 0
            self.waypoint_timer         = 0
            self.waypoint_reached       = False
            print("Starting cycle – ensuring trolley at start position …")

        # ── POSITIONING ───────────────────────────────────────────────────────
        elif self.phase == "POSITIONING":
            self.set_spreader_height(INITIAL_SPREADER_HEIGHT, speed)
            if self.move_trolley_to_waypoint(0, speed * 2):
                self.phase                  = "LOWERING"
                self.phase_timer            = 0
                self.target_spreader_height = CONTAINER_HEIGHT + 1.0
                print("Lowering spreader to container …")

        # ── LOWERING ──────────────────────────────────────────────────────────
        elif self.phase == "LOWERING":
            if self.set_spreader_height(self.target_spreader_height, speed * 2):
                self.phase       = "ATTACHING"
                self.phase_timer = 0
                print(f"Spreader at {spreader_pos[2]:.1f} m, attaching …")

        # ── ATTACHING ─────────────────────────────────────────────────────────
        elif self.phase == "ATTACHING":
            self.set_spreader_height(self.target_spreader_height, speed * 2)

            if not self.attached and self.phase_timer > ATTACH_WAIT_STEPS:
                if abs(spreader_pos[2] - (CONTAINER_HEIGHT + 1.0)) < 0.5:
                    self._attach_container()
                    self.phase                  = "LIFTING"
                    self.phase_timer            = 0
                    self.target_spreader_height = LIFTING_TARGET_HEIGHT
                    print("Container attached! Lifting …")

        # ── LIFTING ───────────────────────────────────────────────────────────
        elif self.phase == "LIFTING":
            if self.set_spreader_height(self.target_spreader_height, speed * 1.5):
                self.phase                  = "MOVING"
                self.phase_timer            = 0
                self.waypoint_timer         = 0
                self.waypoint_reached       = False
                print("Container lifted, moving to ship …")

        # ── MOVING ────────────────────────────────────────────────────────────
        elif self.phase == "MOVING":
            self.set_spreader_height(self.target_spreader_height, speed)
            self._sync_to_trolley()

            if self.current_waypoint_index < len(WAYPOINTS) - 1:
                next_wp = self.current_waypoint_index + 1
                if self.move_trolley_to_waypoint(next_wp, speed * 1.5):
                    self.current_waypoint_index = next_wp

                if self.current_waypoint_index >= len(WAYPOINTS) - 1:
                    self.phase       = "STABILIZING"
                    self.phase_timer = 0
                    print("Reached destination, stabilizing …")

        # ── STABILIZING ───────────────────────────────────────────────────────
        elif self.phase == "STABILIZING":
            self.set_spreader_height(self.target_spreader_height, speed)

            # Zero out any residual velocities and level everything.
            p.resetBaseVelocity(self.spreader_id,  [0, 0, 0], [0, 0, 0])
            p.resetBaseVelocity(self.container_id, [0, 0, 0], [0, 0, 0])

            sp_pos, _ = p.getBasePositionAndOrientation(self.spreader_id)
            c_pos,  _ = p.getBasePositionAndOrientation(self.container_id)
            p.resetBasePositionAndOrientation(self.spreader_id,  sp_pos, [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(self.container_id, c_pos,  [0, 0, 0, 1])

            self._rebuild_constraint_clean()

            if self.phase_timer > STABILIZE_STEPS:
                self.phase                  = "LOWERING_FINAL"
                self.phase_timer            = 0
                self.target_spreader_height = LOWERING_FINAL_HEIGHT + CONTAINER_HEIGHT
                print("Stabilised – lowering container onto ship …")

        # ── LOWERING_FINAL ────────────────────────────────────────────────────
        elif self.phase == "LOWERING_FINAL":
            if self.set_spreader_height(self.target_spreader_height, speed):
                self.phase       = "COMPLETE"
                self.phase_timer = 0
                print("Container placed.")

        # ── COMPLETE ─────────────────────────────────────────────────────────
        # (Extend here to add DETACHING → LIFTING_EMPTY → RETURNING cycle.)

    # ── Reset / stop ─────────────────────────────────────────────────────────

    def reset(self):
        """Called when auto-mode is switched off; releases the container."""
        self.phase = "WAITING"
        self._detach_container()
        p.resetBaseVelocity(self.trolley_id, [0, 0, 0], [0, 0, 0])
