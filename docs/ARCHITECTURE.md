# Architecture & Design Notes

This document explains the reasoning behind the module structure and key design decisions in CraneSim.

---

## Module Dependency Graph

```
main.py
  └── simulator.py
        ├── config.py           (imported by all modules)
        ├── sensors.py
        ├── crane_objects.py
        ├── physics_controller.py
        ├── camera_manager.py
        ├── ui_manager.py
        └── data_manager.py
```

`config.py` is the only cross-cutting dependency. All other modules are leaf nodes — they import config but not each other. `simulator.py` is the only file that imports from multiple modules and wires them together.

---

## Design Principles

### 1. Single source of truth (`config.py`)
Every numeric constant — geometry, forces, damping, paths — lives in `config.py`. No file ever hardcodes a number. This means changing a waypoint, a mass, or an output path requires editing exactly one line in one file.

### 2. Stateless factories (`crane_objects.py`)
Body-creation functions return a single integer (PyBullet body ID) and carry no state. This makes them trivially testable and reusable for multi-container scenarios.

### 3. Separation of physics and presentation
`PhysicsController` knows nothing about sliders or text labels. `UIManager` knows nothing about forces or constraints. `DataManager` knows nothing about the GUI. This separation means any layer can be replaced (e.g. swap the PyBullet GUI for a web dashboard) without touching the physics logic.

### 4. IMU as a pure sensor
`IMUSensor` reads from PyBullet but adds noise and returns a plain dict. It has no concept of phases or constraints — it just reports what the body is doing. This matches how a real IMU would behave.

---

## State Machine (PhysicsController)

The loading cycle is implemented as an explicit string-keyed state machine rather than coroutines or threads. This keeps it:

- **Debuggable** — the current phase is always a readable string visible in the GUI
- **Deterministic** — one step per call, no hidden concurrency
- **Extensible** — adding a new phase is adding an `elif` block

State transitions happen only inside `PhysicsController.step()`. No other module ever writes `self.phase`.

### Phase durations

| Phase | Exit condition |
|-------|---------------|
| WAITING | Immediate (triggers on first step) |
| POSITIONING | Trolley at WP0 AND spreader at initial height |
| LOWERING | Spreader within 0.2 m of `CONTAINER_HEIGHT + 1.0` |
| ATTACHING | `ATTACH_WAIT_STEPS` elapsed AND spreader close enough |
| LIFTING | Spreader within 0.2 m of `LIFTING_TARGET_HEIGHT` |
| MOVING | Trolley reaches last waypoint |
| STABILIZING | `STABILIZE_STEPS` elapsed |
| LOWERING_FINAL | Spreader within 0.2 m of deck height |

---

## Force Model

The spreader is a dynamic body (non-zero mass). Height is controlled by applying a proportional + damping force each step:

```
F_z = (target - current) × multiplier × speed_factor
F_damping = -velocity_z × SPREADER_DAMPING_COEFF
```

This gives smooth, physically plausible motion rather than teleporting the spreader. The multiplier is larger when the container is attached (`SPREADER_FORCE_ATTACHED`) to account for the extra load.

The trolley is **kinematic** (mass = 0). It is moved by direct position resets (`resetBasePositionAndOrientation`), which is appropriate because a real crane trolley is driven by a motor at constant speed, not by free-body forces.

---

## Constraint Strategy

The spreader–container attachment uses `JOINT_FIXED` during normal operation. During the STABILIZING phase the constraint is removed and recreated with perfectly aligned frames. This prevents accumulated numerical drift from the MOVING phase (where positions are set manually) from causing a jerk when physics takes over in LOWERING_FINAL.

To enable realistic container swing, replace `JOINT_FIXED` with `JOINT_POINT2POINT` in `_attach_container()`. The STABILIZING step is still recommended to zero any residual velocity before lowering begins.

---

## Data Collection

`DataManager.collect()` is called once per step only when `auto_mode` is active. It reads poses and IMU data, appends a dict to an in-memory list, and never writes to disk mid-run (which would be slow). The full list is flushed to Excel on quit, keyboard shortcut `Z`, or any exception.

CSV is offered as a fallback because `openpyxl` can fail on some headless Linux environments.
