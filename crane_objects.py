# crane_objects.py
# ─────────────────────────────────────────────────────────────────────────────
# Factory functions that create every PyBullet rigid body used in the scene:
# ground plane, crane mesh, ship mesh, container, trolley, spreader.
# ─────────────────────────────────────────────────────────────────────────────

import pybullet as p

from config import (
    CONTAINER_HEIGHT, CONTAINER_LENGTH, CONTAINER_MASS, CONTAINER_WIDTH,
    CRANE_HEIGHT, CRANE_OBJ, CRANE_POSITION, CRANE_SCALE,
    INITIAL_SPREADER_HEIGHT,
    SHIP_OBJ, SHIP_POSITION, SHIP_SCALE,
    TROLLEY_LINEAR_DAMPING, TROLLEY_ANGULAR_DAMPING,
    WAYPOINTS,
)


def load_plane() -> int:
    """Load the infinite ground plane."""
    return p.loadURDF("plane.urdf")


def load_crane() -> int:
    """Load the crane mesh (static, no collision response needed)."""
    orientation = p.getQuaternionFromEuler([1.570, 0, 1.570])
    visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=CRANE_OBJ,
        meshScale=CRANE_SCALE,
    )
    # Collision is intentionally disabled (-1) to avoid interference.
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual,
        basePosition=CRANE_POSITION,
        baseOrientation=orientation,
    )


def load_ship() -> int:
    """Load the ship mesh (static)."""
    orientation = p.getQuaternionFromEuler([1.570, 0, 0])
    visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=SHIP_OBJ,
        meshScale=SHIP_SCALE,
    )
    collision = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=SHIP_OBJ,
        meshScale=SHIP_SCALE,
    )
    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=SHIP_POSITION,
        baseOrientation=orientation,
    )


def create_container() -> int:
    """Create the shipping container at ground level at the first waypoint."""
    half = [CONTAINER_LENGTH / 2, CONTAINER_WIDTH / 2, CONTAINER_HEIGHT / 2]

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=[0.8, 0.2, 0.2, 1.0],
    )
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)

    return p.createMultiBody(
        baseMass=CONTAINER_MASS,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0, WAYPOINTS[0], CONTAINER_HEIGHT / 2],
        baseOrientation=[0, 0, 0, 1],
    )


def create_trolley() -> int:
    """Create the crane trolley (kinematic – mass set to 0 after creation)."""
    hx = CONTAINER_LENGTH / 2 + 0.3
    hy = CONTAINER_WIDTH  / 2 + 0.3
    hz = 0.5

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[hx, hy, hz],
        rgbaColor=[0.3, 0.3, 0.8, 1.0],
    )
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz])

    trolley_id = p.createMultiBody(
        baseMass=10_000,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0, WAYPOINTS[0], CRANE_HEIGHT],
        baseOrientation=[0, 0, 0, 1],
    )

    # Make kinematic so we can drive it via resetBasePositionAndOrientation.
    p.changeDynamics(
        trolley_id, -1,
        mass=0,
        linearDamping=TROLLEY_LINEAR_DAMPING,
        angularDamping=TROLLEY_ANGULAR_DAMPING,
    )
    return trolley_id


def create_spreader() -> int:
    """Create the container spreader, starting at INITIAL_SPREADER_HEIGHT."""
    hx = CONTAINER_LENGTH / 2 + 0.3
    hy = CONTAINER_WIDTH  / 2 + 0.3

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[hx, hy, 0.3],
        rgbaColor=[0.7, 0.7, 0.2, 1.0],
    )
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.3])

    return p.createMultiBody(
        baseMass=5_000,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0, WAYPOINTS[0], INITIAL_SPREADER_HEIGHT],
        baseOrientation=[0, 0, 0, 1],
    )


def create_spreader_trolley_constraint(trolley_id: int, spreader_id: int) -> int:
    """Attach spreader to trolley with a prismatic (vertical-only) joint."""
    constraint_id = p.createConstraint(
        trolley_id,  -1,
        spreader_id, -1,
        p.JOINT_PRISMATIC,
        [0, 0, 1],    # axis – Z only
        [0, 0, -5],   # parent frame offset
        [0, 0, 0],    # child frame offset
    )
    p.changeConstraint(constraint_id, maxForce=100_000)
    return constraint_id
