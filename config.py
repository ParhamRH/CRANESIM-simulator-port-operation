# config.py
# ─────────────────────────────────────────────────────────────────────────────
# Central configuration for the Panamax Crane Container Loading Simulator.
# Edit values here; nothing else needs to change.
# ─────────────────────────────────────────────────────────────────────────────

# ── Physics ──────────────────────────────────────────────────────────────────
GRAVITY = -9.81          # m/s²
SIMULATION_HZ = 240      # steps per second

# ── Container geometry (metres) ───────────────────────────────────────────────
CONTAINER_LENGTH = 12.19
CONTAINER_WIDTH  = 2.44
CONTAINER_HEIGHT = 2.59
CONTAINER_MASS   = 500   # kg (base rigid-body mass; payload separate)

# Container metadata
CONTAINER_TYPE = "40ft_standard"   # "20ft" | "40ft_high_cube" | …
CONTAINER_LOAD = 25_000            # kg – current cargo payload

# ── Crane geometry ────────────────────────────────────────────────────────────
CRANE_HEIGHT            = 39      # m – top of crane
INITIAL_SPREADER_HEIGHT = 10      # m – spreader start height

# ── Trolley waypoints (Y-axis positions along quay → ship) ───────────────────
WAYPOINTS = [0, 47]   # [quay position, ship position]

# ── Dynamics damping ─────────────────────────────────────────────────────────
SPREADER_LINEAR_DAMPING  = 3.0
SPREADER_ANGULAR_DAMPING = 3.0
CONTAINER_LINEAR_DAMPING = 0.5
CONTAINER_ANGULAR_DAMPING = 0.5
TROLLEY_LINEAR_DAMPING   = 5.0
TROLLEY_ANGULAR_DAMPING  = 5.0

# ── Force tuning ─────────────────────────────────────────────────────────────
SPREADER_FORCE_ATTACHED   = 15_000   # multiplier when container attached
SPREADER_FORCE_FREE       = 10_000   # multiplier when spreader is free
SPREADER_DAMPING_COEFF    = 2_000
CONSTRAINT_MAX_FORCE      = 1_000_000
TROLLEY_CONSTRAINT_FORCE  = 100_000

# ── Phase timings (simulation steps) ─────────────────────────────────────────
ATTACH_WAIT_STEPS      = 60    # steps to wait before locking the constraint
STABILIZE_STEPS        = 30    # steps in STABILIZING phase
LIFTING_TARGET_HEIGHT  = 33    # m – safe travel height
LOWERING_FINAL_HEIGHT  = 24.6  # m – ship deck height (spreader bottom)

# ── IMU sensor ───────────────────────────────────────────────────────────────
IMU_NOISE_LEVEL = 0.01
# Local position of the IMU on the container (top corner, ventilator-style)
IMU_LOCAL_POSITION = [
    CONTAINER_LENGTH / 2 - 0.3,
    CONTAINER_WIDTH  / 2 - 0.3,
    CONTAINER_HEIGHT / 2,
]

# ── Camera (synthetic) ────────────────────────────────────────────────────────
CAMERA_WIDTH    = 320
CAMERA_HEIGHT   = 240
CAMERA_FOV      = 60        # degrees
CAMERA_NEAR     = 0.1
CAMERA_FAR      = 100
CAMERA_POSITION = [0, -25, 20]
CAMERA_TARGET   = [0, 10, 17]
CAMERA_UPDATE_INTERVAL = 30  # frames between synthetic-camera refreshes

# ── Debug viewer (initial) ────────────────────────────────────────────────────
DEBUG_CAM_DISTANCE = 55
DEBUG_CAM_YAW      = 40
DEBUG_CAM_PITCH    = -30
DEBUG_CAM_TARGET   = [0, 30, 8]

# ── Mesh files & scales ───────────────────────────────────────────────────────
CRANE_OBJ       = "crane.obj"
CRANE_SCALE     = [3.5, 3.5, 4.0]
CRANE_POSITION  = [0, 5, 38.5]

SHIP_OBJ        = "ship.obj"
SHIP_SCALE      = [3.3, 3.3, 3.3]
SHIP_POSITION   = [2.3, 40, -13]

# ── Output ────────────────────────────────────────────────────────────────────
OUTPUT_DIR = "~/ContainerSimulation"
