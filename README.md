#  CraneSim — Panamax Container Crane Physics Simulator

![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python)
![PyBullet](https://img.shields.io/badge/PyBullet-physics-orange)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Active-brightgreen)

A real-time, physics-based simulation of a **Panamax container crane** loading shipping containers from quay to vessel. Built with [PyBullet](https://pybullet.org), the simulator produces labelled IMU sensor datasets suitable for machine-learning research in industrial IoT, crane monitoring, and predictive maintenance.

---

##  Features

- Full 8-phase automatic loading cycle (position → lower → attach → lift → travel → stabilise → place)
- Simulated **IMU sensor** on the container — gyroscope, accelerometer, magnetometer, barometer
- Per-step dataset export to **Excel (.xlsx)** with phase labels, poses, and sensor readings
- Interactive **PyBullet GUI** with live on-screen telemetry
- Synthetic **RGB / Depth / Segmentation** camera output
- Modular architecture — every subsystem is its own file

---

##  Project Structure

```
CraneSim-Simulator-Port-Operation/
│
├── main.py                   # Entry point
├── simulator.py              # ContainerSimulator orchestrator & main loop
├── config.py                 # All constants (geometry, forces, paths …)
├── sensors.py                # IMUSensor — gyro / accel / mag / baro
├── crane_objects.py          # PyBullet body factories (container, trolley, spreader …)
├── physics_controller.py     # State machine, force control, trolley kinematics
├── camera_manager.py         # Debug viewer presets + synthetic camera renders
├── ui_manager.py             # GUI sliders, on-screen text, waypoint markers
├── data_manager.py           # Per-step data collection & Excel/CSV export
│
├── assets/
│   ├── meshes/               # crane.obj, ship.obj and any other 3-D assets
│   ├── images/               # Logo, diagrams used in documentation
│   └── screenshots/          # Simulation screenshots
│
├── output/
│   ├── excel/                # Generated .xlsx datasets (git-ignored)
│   └── logs/                 # Optional run logs (git-ignored)
│
├── docs/
│   ├── images/               # Figures used in docs/
│   ├── api/                  # Per-module API reference stubs
│   └── ARCHITECTURE.md       # Deep-dive design notes
│
├── tests/                    # Unit tests (pytest)
│
├── .github/
│   ├── workflows/            # CI pipeline (lint + tests)
│   └── ISSUE_TEMPLATE/       # Bug report & feature request templates
│
├── .gitignore
├── CONTRIBUTING.md
├── CHANGELOG.md
├── LICENSE
└── requirements.txt
```

---

##  Requirements

| Package | Purpose |
|---------|---------|
| `pybullet` | Physics engine & GUI |
| `numpy` | Vector / matrix maths |
| `pandas` | DataFrame & Excel export |
| `openpyxl` | `.xlsx` write support |

Install everything:

```bash
pip install -r requirements.txt
```

You also need `crane.obj` and `ship.obj` mesh files placed in **`assets/meshes/`**. Update `config.py` (`CRANE_OBJ`, `SHIP_OBJ`) if you store them elsewhere.

---

##  Quick Start

```bash
git clone https://github.com/ParhamRH/Simulator-Port-Operation.git
cd Simulator-Port-Operation
pip install -r requirements.txt

# Copy your mesh files
cp /path/to/crane.obj assets/meshes/
cp /path/to/ship.obj  assets/meshes/

python main.py
```

Once the GUI opens, drag the **"Start Automatic Cycle"** slider to **1**.

---

##  Controls

| Input | Action |
|-------|--------|
| `Slider → 1` | Start automatic loading cycle |
| `Slider → 0` | Stop & reset |
| `C` | Cycle camera presets (iso / front / side / top / crane) |
| `J` / `L` | Rotate debug camera left / right |
| `I` / `K` | Tilt debug camera up / down |
| Arrow keys | Pan camera target |
| `U` / `H` | Move synthetic camera up / down |
| `Z` | Save dataset to `output/excel/` immediately |
| `Q` | Quit (auto-saves on exit) |

---

## 🔄 Automatic Cycle Phases

```
WAITING
  └─► POSITIONING   (trolley returns to quay, spreader rises)
        └─► LOWERING        (spreader descends to container)
              └─► ATTACHING  (constraint locked)
                    └─► LIFTING         (container lifted to travel height)
                          └─► MOVING          (trolley traverses to ship)
                                └─► STABILIZING     (zero velocities, re-lock constraint)
                                      └─► LOWERING_FINAL  (container set onto deck)
                                            └─► COMPLETE
```

---

##  Dataset Output

Saved to `output/excel/container_imu_data_<YYYYMMDD_HHMMSS>.xlsx`.

| Column group | Columns |
|---|---|
| **Meta** | `timestamp`, `phase`, `attached` |
| **Positions** | `container_x/y/z`, `trolley_y`, `spreader_height` |
| **Gyroscope** | `gyro_x`, `gyro_y`, `gyro_z` (rad/s) |
| **Accelerometer** | `acc_x`, `acc_y`, `acc_z` (m/s²) |
| **Magnetometer** | `mag_x`, `mag_y`, `mag_z` (µT) |
| **Orientation** | `roll`, `pitch`, `yaw` (rad) |
| **Barometer** | `altitude` (m) |
| **Derived** | `cable_tension` (N) |
| **Container info** | `container_type`, `container_load`, `imu_location_x/y/z` |

---

##  Configuration

All tunable parameters are in **`config.py`** — no hunting through source files:

```python
# Example tweaks
WAYPOINTS        = [0, 47]      # Add intermediate stops
CONTAINER_LOAD   = 30_000       # kg
IMU_NOISE_LEVEL  = 0.05         # Increase sensor noise
LIFTING_TARGET_HEIGHT = 35      # m — raise travel clearance
```

---

##  Extending the Simulation

Ideas already scaffolded in the code comments:

| Feature | Where to add |
|---------|-------------|
| **Pendulum / swing** | `physics_controller.py → _attach_container()` — swap `JOINT_FIXED` for `JOINT_POINT2POINT` |
| **Wind disturbances** | New `apply_wind_force()` method in `PhysicsController` |
| **Variable loads** | Randomise `CONTAINER_LOAD` in `config.py` each cycle |
| **Control delay** | Add a `deque` input buffer in `PhysicsController.step()` |
| **Return cycle** | Uncomment `DETACHING → LIFTING_EMPTY → RETURNING` in `physics_controller.py` |
| **Multiple container types** | Extend `crane_objects.create_container()` with a `type` parameter |

---

##  Tests

```bash
pytest tests/
```

Test stubs live in `tests/`. Contributions welcome — see [CONTRIBUTING.md](CONTRIBUTING.md).



---

##  Acknowledgements

- [PyBullet](https://pybullet.org) — Erwin Coumans & Yunfei Bai
- [pybullet_data](https://github.com/bulletphysics/bullet3) — bundled URDF assets
