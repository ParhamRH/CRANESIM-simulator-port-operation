# =======================================
"""
1. Load Variations:
self.container_mass = random.uniform(5000, 40000)  # Empty to full

2. Control Delays and Uncertainties:
self.control_buffer = deque(maxlen=5)  # 5-step delay

3. Cable Tension and angles
4. environmental distribances (wind, ...)
5. Container swing
6. Multiple container type
7. Pendulum Dynamics:
    # Replace JOINT_FIXED with JOINT_POINT2POINT for cable flexibility
    self.container_constraint = p.createConstraint(
        self.spreaderId, -1,
        self.containerId, -1,
        p.JOINT_POINT2POINT,  # Allows swaying
        [0, 0, -0.6],
        [0, 0, self.container_height/2]
    )
8. Dynamic Disturbances:
    # Add wind forces
    def apply_wind_force(self):
        if self.attached and self.phase in ["MOVING", "LOWERING_FINAL"]:
            wind_force = [
                random.gauss(0, 100),  # X wind
                random.gauss(0, 100),  # Y wind  
                0
            ]
            p.applyExternalForce(self.containerId, -1, wind_force, [0,0,0], p.WORLD_FRAME)
"""
# =======================================

import pybullet as p
import pybullet_data
import time
import math

import numpy as np
import pandas as pd
from datetime import datetime
import os

class IMUSensor:
    """Simulates an IMU sensor with gyroscope, accelerometer, magnetometer, and barometer"""
    def __init__(self, body_id, local_position=[0, 0, 0], noise_level=0.01):
        self.body_id = body_id
        self.local_position = local_position
        self.noise_level = noise_level
        self.last_orientation = None
        self.last_time = time.time()
        self.gravity = 9.81
        self.sea_level_pressure = 1013.25  # hPa
        self.magnetic_field = np.array([25.0, 0.0, 40.0])  # μT
    
    def get_measurements(self):
        """Get all sensor measurements"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Get position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.body_id)
        linear_vel, angular_vel = p.getBaseVelocity(self.body_id)
        
        # Convert quaternion to euler angles
        euler = p.getEulerFromQuaternion(orn)
        roll, pitch, yaw = euler
        
        # Gyroscope (angular velocity in rad/s)
        gyro = np.array(angular_vel) + np.random.normal(0, self.noise_level, 3)
        
        # Accelerometer (includes gravity)
        rotation_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        gravity_body = rotation_matrix.T @ np.array([0, 0, -self.gravity])
        linear_acc = np.array([0, 0, 0])
        acc = gravity_body + linear_acc + np.random.normal(0, self.noise_level * 10, 3)
        
        # Magnetometer
        mag_body = rotation_matrix.T @ self.magnetic_field
        mag = mag_body + np.random.normal(0, self.noise_level * 5, 3)
        
        # Barometer
        altitude = pos[2]
        altitude_baro = altitude + np.random.normal(0, self.noise_level * 2)
        
        return {
            'timestamp': datetime.now(),
            'gyro_x': gyro[0], 'gyro_y': gyro[1], 'gyro_z': gyro[2],
            'acc_x': acc[0], 'acc_y': acc[1], 'acc_z': acc[2],
            'mag_x': mag[0], 'mag_y': mag[1], 'mag_z': mag[2],
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'quat_x': orn[0], 'quat_y': orn[1], 'quat_z': orn[2], 'quat_w': orn[3],
            'altitude': altitude_baro
        }

class ContainerSimulator:
    def __init__(self):
        """Initialize the container simulator"""
        # Connect to PyBullet
        self.physicsClient = p.connect(p.GUI)
        
        # Set up physics
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        
        # Setup camera
        p.resetDebugVisualizerCamera(
            cameraDistance=55,
            cameraYaw=40,
            cameraPitch=-30,
            cameraTargetPosition=[0, 30, 8]
        )
        
        # Enable GUI visualization of camera data
        self.setup_synthetic_camera()
        self.camera_update_counter = 0
        self.camera_update_interval = 30  # Update every 30 frames
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)

        # Load ground
        self.planeId = p.loadURDF("plane.urdf")
        
        # Container dimensions
        self.container_length = 12.19
        self.container_width = 2.44
        self.container_height = 2.59
        
        # Crane parameters
        self.crane_height = 39
        self.initial_spreader_height = 10
        
        # Waypoint configuration for trolley movement (Y-axis positions)
        # These represent the path from quay to ship
        self.waypoints = [
            0,     # Start position - at quay/crane (purple circle)
            # 31  #WP1
            47  #WP2
        ]
            # 25,    # Middle waypoint 2 (brown circle)  
            # 35     # Final destination - over ship (red circle)
        
        # Time to spend moving between waypoints (in simulation steps)
        self.waypoint_wait_time = 30  # Reduced for smoother movement
        
        # Waypoint tracking
        self.current_waypoint_index = 0
        self.waypoint_timer = 0
        self.waypoint_reached = False
        
        # Create simulation objects
        self.ship_id = self.load_ship_obj()
        self.crane_id = self.load_crane_obj()
        
        # Create container at ground level at first waypoint
        self.containerId = self.create_container()
        
        # Initialize IMU at top corner of container (ventilator position)
        # Standard container ventilator is typically at top corner
        imu_offset = [
            self.container_length/2 - 0.3,  # Near corner
            self.container_width/2 - 0.3,   # Near corner  
            self.container_height/2         # Top of container
        ]
        self.container_imu = IMUSensor(self.containerId, local_position=imu_offset)
        
        # Data collection setup
        self.sensor_data = []
        self.dataset = {
            'timestamp': [],
            'spreader_pose': [],
            'container_pose': [],
            'cable_tension': [],
            'imu_data': [],
            'control_inputs': [],
            'phase_labels': [],
            'imu_location': [],
            'container_type': [],
            'container_load': []
        }
        
        # Container specifications
        self.container_type = "40ft_standard"  # or "20ft", "40ft_high_cube", etc.
        self.container_load = 25000  # kg - current load

        # Create trolley at first waypoint
        self.trolleyId = self.create_trolley()
        
        # Create spreader high up, ready to descend
        self.spreaderId = self.create_spreader()
        
        # Create constraint between trolley and spreader
        self.spreader_constraint = p.createConstraint(
            self.trolleyId, -1,
            self.spreaderId, -1,
            p.JOINT_PRISMATIC,
            [0, 0, 1],  # Z-axis only (vertical movement)
            [0, 0, -5],  # Parent frame position
            [0, 0, 0]   # Child frame position
        )
        p.changeConstraint(self.spreader_constraint, maxForce=100000)


        
        
        # Cable attachment points for visual cables
        self.cable_attach_points = [
            [self.container_length/2, self.container_width/2, 0.3],
            [-self.container_length/2, self.container_width/2, 0.3],
            [self.container_length/2, -self.container_width/2, 0.3],
            [-self.container_length/2, -self.container_width/2, 0.3]
        ]
        
        # Create visual cables
        self.cable_visuals = []
        for i in range(4):
            cable_visual = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0.2, 0.2, 0.2], lineWidth=3)
            self.cable_visuals.append(cable_visual)
        
        # Create visual waypoint markers
        self.create_waypoint_markers()
        
        # Set dynamics
        p.changeDynamics(self.spreaderId, -1, linearDamping=3.0, angularDamping=3.0)
        p.changeDynamics(self.containerId, -1, linearDamping=0.5, angularDamping=0.5)
        p.changeDynamics(self.trolleyId, -1, linearDamping=5.0, angularDamping=5.0)
        
        # GUI controls
        self.auto_slider = p.addUserDebugParameter("Start Automatic Cycle (0=Stop, 1=Start)", 0, 1, 0)
        self.speed_slider = p.addUserDebugParameter("Speed", 1, 10.0, 1)

        # # # Parameter sliders
        # p.addUserDebugParameter("IMU Position", 0.1, 4.0, 1.0)
        # p.addUserDebugParameter("Duration (s)", 10, 120, 30)
        # p.addUserDebugParameter("Phase (1-6)", 1, 6, 1)
        # p.addUserDebugParameter("Cargo Weight", 1, 10000, 1)
        # p.addUserDebugParameter("Wind Speed", 0, 20, 0)
        # p.addUserDebugParameter("Quality Measure", 0, 100, 0)
        
        # # Sensor displays (read-only decorative)
        # p.addUserDebugParameter("Gyro X (rad/s)", -2, 2, 0)
        # p.addUserDebugParameter("Gyro Y (rad/s)", -2, 2, 0)
        # p.addUserDebugParameter("Gyro Z (rad/s)", -2, 2, 0)
        # p.addUserDebugParameter("Acc X (m/s²)", -20, 20, 0)
        # p.addUserDebugParameter("Acc Y (m/s²)", -20, 20, 0)
        # p.addUserDebugParameter("Acc Z (m/s²)", -20, 20, 0)
        # p.addUserDebugParameter("Roll (rad)", -1, 1, 0)
        # p.addUserDebugParameter("Pitch (rad)", -1, 1, 0)
        # p.addUserDebugParameter("Yaw (rad)", -1, 1, 0)
        
        # State variables
        self.phase = "WAITING"
        self.container_constraint = None
        self.attached = False
        self.phase_timer = 0
        self.target_spreader_height = self.initial_spreader_height

        # Initialize sensor text displays at fixed positions
        self.sensor_text_gyro = p.addUserDebugText("", [30, -20, 34], textSize=1.0, textColorRGB=[1, 0, 0])
        self.sensor_text_acc = p.addUserDebugText("", [30, -20, 33], textSize=1.0, textColorRGB=[0, 1, 0])
        self.sensor_text_rpy = p.addUserDebugText("", [30, -20, 32], textSize=1.0, textColorRGB=[0, 0, 1])
        
        print("Simulation started. Set 'Start Automatic Cycle' to 1 to begin.")
        print(f"Waypoints configured at Y positions: {self.waypoints}")
    
    
    def create_waypoint_markers(self):
        """Create visual markers for waypoints"""
        self.waypoint_markers = []
        colors = [[0.5, 0, 0.5, 0.5], [1, 0.5, 0.7, 0.5]]
        labels = ["WP1", "WP2"]
        
        for i, waypoint_y in enumerate(self.waypoints):
            # Set different Z heights for markers
            if i == 0:
                marker_z = 0.1  # First marker at ground level
            else:
                marker_z = 24.6  # Second marker elevated (adjust this value as needed)

            # Create a thin box marker at ground level
            marker_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[8, 2, 0.1],
                rgbaColor=colors[i]
            )
            
            marker_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=marker_visual,
                basePosition=[0, waypoint_y, marker_z],
                baseOrientation=[0, 0, 0, 1]
            )
            
            self.waypoint_markers.append(marker_id)
            
            # Add text label
            p.addUserDebugText(f"{labels[i]}", 
                             [0, waypoint_y, marker_z], 
                             textColorRGB=colors[i][:3], 
                             textSize=1.5,
                             replaceItemUniqueId=200 + i)
    
    def load_crane_obj(self):
        """Load crane mesh object"""
        mesh_scale = [3.5, 3.5, 4]
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName="crane.obj",
            meshScale=mesh_scale
        )
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName="crane.obj",
            meshScale=mesh_scale
        )
        orientation = p.getQuaternionFromEuler([1.570, 0, 1.570])
        crane_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[0, 5, 38.5],
            baseOrientation=orientation
        )
        return crane_id
    
    def load_ship_obj(self):
        """Load ship mesh object"""
        mesh_scale = [3.3, 3.3, 3.3]
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName="ship.obj",
            meshScale=mesh_scale
        )
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName="ship.obj",
            meshScale=mesh_scale
        )
        orientation = p.getQuaternionFromEuler([1.570, 0, 0])
        ship_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=1,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[2.3, 40, -13],  
            baseOrientation=orientation
        )
        return ship_id
    
    def create_container(self):
        """Create the shipping container at ground level"""
        container_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.container_length/2, self.container_width/2, self.container_height/2],
            rgbaColor=[0.8, 0.2, 0.2, 1]
        )
        
        container_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.container_length/2, self.container_width/2, self.container_height/2]
        )
        
        # Container starts at ground level at first waypoint
        containerId = p.createMultiBody(
            baseMass=500,
            baseCollisionShapeIndex=container_collision,
            baseVisualShapeIndex=container_visual,
            basePosition=[0, self.waypoints[0], self.container_height/2],  # At first waypoint, on ground
            baseOrientation=[0, 0, 0, 1]
        )
        
        return containerId
    
    def create_trolley(self):
        """Create the crane trolley at first waypoint"""
        # Match spreader footprint with a little extra thickness
        half_x = self.container_length / 2 + 0.3
        half_y = self.container_width / 2 + 0.3
        half_z = 0.5  # thicker than spreader for structure
        
        trolley_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[half_x, half_y, half_z],
            rgbaColor=[0.3, 0.3, 0.8, 1]
        )
        
        trolley_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[half_x, half_y, half_z]
        )
        
        # Trolley starts at first waypoint, high up
        trolleyId = p.createMultiBody(
            baseMass=10000,
            baseCollisionShapeIndex=trolley_collision,
            baseVisualShapeIndex=trolley_visual,
            basePosition=[0, self.waypoints[0], self.crane_height],  # At first waypoint
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Make it kinematic after creation
        p.changeDynamics(trolleyId, -1,  # Use trolleyId, not self.trolleyId
                        mass=0,  # Set mass to 0 makes it kinematic 100000
                        linearDamping=5.0,
                        angularDamping=5.0)
        
        return trolleyId
    
    def create_spreader(self):
        """Create the container spreader"""
        spreader_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.container_length/2 + 0.3, self.container_width/2 + 0.3, 0.3],
            rgbaColor=[0.7, 0.7, 0.2, 1]
        )
        
        spreader_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.container_length/2 + 0.3, self.container_width/2 + 0.3, 0.3]
        )
        
        # Spreader starts high, ready to descend
        spreaderId = p.createMultiBody(
            baseMass=5000,
            baseCollisionShapeIndex=spreader_collision,
            baseVisualShapeIndex=spreader_visual,
            basePosition=[0, self.waypoints[0], self.initial_spreader_height],  # At first waypoint, high
            baseOrientation=[0, 0, 0, 1]
        )
        
        return spreaderId
    

    def set_spreader_height(self, target_height, speed_factor):
        """Control spreader height using forces"""
        spreader_pos, _ = p.getBasePositionAndOrientation(self.spreaderId)
        current_height = spreader_pos[2]
        
        # Calculate height difference
        diff = target_height - current_height
        
        if abs(diff) > 0.2:
            # Apply force to move spreader up or down
            # Stronger force for lifting when attached
            force_multiplier = 15000 if self.attached else 10000
            force_z = diff * force_multiplier * speed_factor
            
            # Apply the force
            p.applyExternalForce(self.spreaderId, -1, [0, 0, force_z], [0, 0, 0], p.WORLD_FRAME)
            
            # Also apply small damping to prevent oscillation
            velocity = p.getBaseVelocity(self.spreaderId)[0][2]
            damping_force = -velocity * 2000
            p.applyExternalForce(self.spreaderId, -1, [0, 0, damping_force], [0, 0, 0], p.WORLD_FRAME)
            
            return False  # Not at target
        else:
            return True  # At target
    

    def move_trolley_to_waypoint(self, waypoint_index, speed):
        if waypoint_index >= len(self.waypoints):
            return True
        
        target_y = float(self.waypoints[waypoint_index])
        pos, orn = p.getBasePositionAndOrientation(self.trolleyId)
        current_y = pos[1]
        
        # For kinematic trolley (mass=0), use position-based movement
        if abs(current_y - target_y) > 0.05:
            step = min(speed * 0.01, abs(target_y - current_y))
            new_y = current_y + step * (1 if target_y > current_y else -1)
            p.resetBasePositionAndOrientation(
                self.trolleyId,
                [pos[0], new_y, pos[2]],
                orn
            )
            return False
        else:
            return True
    
    def update_cables(self):
        """Update visual cable positions at the corners of the spreader"""
        spreader_pos, spreader_orn = p.getBasePositionAndOrientation(self.spreaderId)
        trolley_pos, trolley_orn = p.getBasePositionAndOrientation(self.trolleyId)
        
        # Get spreader half-extents
        hx = self.container_length / 2 + 0.3
        hy = self.container_width  / 2 + 0.3
        
        # Corners relative to spreader center
        corners = [
            [ hx,  hy, 0],
            [-hx,  hy, 0],
            [ hx, -hy, 0],
            [-hx, -hy, 0]
        ]
        
        # trolley height: attach cables slightly below trolley base
        trolley_attach_z = trolley_pos[2] - 0.5
        
        for i, corner in enumerate(corners):
            # world coordinates of spreader corner
            world_point = p.multiplyTransforms(
                spreader_pos, spreader_orn,
                corner, [0, 0, 0, 1]
            )[0]
            
            # corresponding trolley attachment
            trolley_attach = [
                trolley_pos[0] + corner[0],
                trolley_pos[1] + corner[1],
                trolley_attach_z
            ]
            
            p.addUserDebugLine(
                trolley_attach,
                world_point,
                [0.2, 0.2, 0.2],
                lineWidth=3,
                replaceItemUniqueId=self.cable_visuals[i]
            )
    
    def update_status_display(self):
        """Update status text display"""
        spreader_pos, _ = p.getBasePositionAndOrientation(self.spreaderId)
        trolley_pos, _ = p.getBasePositionAndOrientation(self.trolleyId)
        
        # Calculate cable length for display
        cable_length = abs(trolley_pos[2] - spreader_pos[2])
        
        # Display status
        status = "ATTACHED" if self.attached else "DETACHED"
        p.addUserDebugText(f"Phase: {self.phase} | Status: {status}", 
                          [0, 55, 48], 
                          textColorRGB=[1, 1, 1], 
                          textSize=1.5,
                          replaceItemUniqueId=100)
        
        p.addUserDebugText(f"Cable Length: {cable_length:.1f}m | Spreader Height: {spreader_pos[2]:.1f}m", 
                          [0.3, 55, 46.5], 
                          textColorRGB=[0, 0, 0], 
                          textSize=1.2,
                          replaceItemUniqueId=101)
        
        waypoint_label = f"{self.current_waypoint_index + 1}/{len(self.waypoints)}"
        p.addUserDebugText(f"Trolley Y: {trolley_pos[1]:.1f} | Waypoint: {waypoint_label}", 
                          [0.6, 55, 45], 
                          textColorRGB=[0, 0, 0], 
                          textSize=1.2,
                          replaceItemUniqueId=102)
        
        imu_data = self.container_imu.get_measurements()

        # Add IMU sensor displays if available
        if hasattr(self, 'container_imu'):
            
            # Gyroscope data
            gyro_text = f"Gyro: X={imu_data['gyro_x']:.3f} Y={imu_data['gyro_y']:.3f} Z={imu_data['gyro_z']:.3f} rad/s"
            p.addUserDebugText(gyro_text,
                            [1.4, 55, 43.5],
                            textColorRGB=[1, 0, 0],
                            textSize=1.0,
                            replaceItemUniqueId=103)
            
            # Accelerometer data
            acc_text = f"Acc: X={imu_data['acc_x']:.2f} Y={imu_data['acc_y']:.2f} Z={imu_data['acc_z']:.2f} m/s²"
            p.addUserDebugText(acc_text,
                            [1.5, 55, 42],
                            textColorRGB=[0, 0, 1],
                            textSize=1.0,
                            replaceItemUniqueId=104)
            
            # Roll/Pitch/Yaw data

            rpy_text = f"RPY: R={imu_data['roll']:.3f} P={imu_data['pitch']:.3f} Y={imu_data['yaw']:.3f} rad"
            p.addUserDebugText(rpy_text,
                            [1.7, 55, 40.5],
                            textColorRGB=[0, 1, 0],
                            textSize=1.0,
                            replaceItemUniqueId=105)
    
    def handle_automatic_cycle(self, speed):
        """Handle the automatic crane cycle with proper sequencing"""
        self.phase_timer += 1
        spreader_pos, _ = p.getBasePositionAndOrientation(self.spreaderId)
        
        if self.phase == "WAITING":
            self.phase = "POSITIONING"
            self.phase_timer = 0
            self.target_spreader_height = self.initial_spreader_height
            self.current_waypoint_index = 0
            self.waypoint_timer = 0
            self.waypoint_reached = False
            print("Starting cycle - Ensuring trolley at start position...")
            
        elif self.phase == "POSITIONING":
            # Ensure trolley is at first waypoint and spreader is high
            self.set_spreader_height(self.initial_spreader_height, speed)
            
            # Make sure trolley is at first waypoint
            if self.move_trolley_to_waypoint(0, speed * 2):
                self.phase = "LOWERING"
                self.phase_timer = 0
                self.target_spreader_height = self.container_height + 1.0
                print("Lowering spreader to container...")
                
        elif self.phase == "LOWERING":
            # Lower spreader to container at quay
            if self.set_spreader_height(self.target_spreader_height, speed * 2):
                self.phase = "ATTACHING"
                self.phase_timer = 0
                print(f"Spreader at height {spreader_pos[2]:.1f}m, attaching...")
                
        elif self.phase == "ATTACHING":
            # Keep applying downward force to maintain contact
            self.set_spreader_height(self.target_spreader_height, speed * 2)
            
            if not self.attached and self.phase_timer > 60:
                # Check if spreader is close enough to container
                if abs(spreader_pos[2] - (self.container_height + 1.0)) < 0.5:
                    self.container_constraint = p.createConstraint(
                        self.spreaderId, -1,
                        self.containerId, -1,
                        p.JOINT_FIXED,
                        [0, 0, -0.3],
                        [0, 0, self.container_height/2],
                        [0, 0, 0]
                    )
                    p.changeConstraint(self.container_constraint, maxForce=1000000)
                    self.attached = True
                    self.phase = "LIFTING"
                    self.phase_timer = 0
                    self.target_spreader_height = 33  # Lift high for clearance
                    print("Container attached! Lifting...")
                
        elif self.phase == "LIFTING":
            # Lift spreader with container to safe height
            if self.set_spreader_height(self.target_spreader_height, speed * 1.5):
                self.phase = "MOVING"
                self.phase_timer = 0
                # Don't reset waypoint index - we're already at waypoint 0
                # and need to move to waypoint 1 next
                self.waypoint_timer = 0
                self.waypoint_reached = False
                print("Container lifted, starting movement to ship...")
                
        elif self.phase == "MOVING":
            self.set_spreader_height(self.target_spreader_height, speed)
            
            # Get trolley movement
            trolley_pos, _ = p.getBasePositionAndOrientation(self.trolleyId)
            
            # Move spreader AND container together to match trolley Y position
            spreader_pos, spreader_orn = p.getBasePositionAndOrientation(self.spreaderId)
            new_spreader_pos = [spreader_pos[0], trolley_pos[1], spreader_pos[2]]
            p.resetBasePositionAndOrientation(self.spreaderId, new_spreader_pos, spreader_orn)
            
            if self.attached:
                container_pos, container_orn = p.getBasePositionAndOrientation(self.containerId)
                new_container_pos = [container_pos[0], trolley_pos[1], container_pos[2]]
                p.resetBasePositionAndOrientation(self.containerId, new_container_pos, container_orn)
            
            # Check if we need to move to next waypoint
            if self.current_waypoint_index < len(self.waypoints) - 1:
                # Move to next waypoint
                next_waypoint = self.current_waypoint_index + 1
                
                if self.move_trolley_to_waypoint(next_waypoint, speed * 1.5):
                    # Successfully reached the waypoint
                    self.current_waypoint_index = next_waypoint
                    
                if self.current_waypoint_index >= len(self.waypoints) - 1:
                    # Instead of going directly to LOWERING_FINAL
                    self.phase = "STABILIZING"  # New transition phase
                    self.phase_timer = 0
                    print("Reached destination, stabilizing before lowering...")

        elif self.phase == "STABILIZING":
            """Transition phase to clean up after manual movement before physics-based lowering"""
            
            # Stop all manual position updates
            # Just maintain height with physics
            self.set_spreader_height(self.target_spreader_height, speed)
            
            # Clear any velocities
            p.resetBaseVelocity(self.spreaderId, [0,0,0], [0,0,0])
            p.resetBaseVelocity(self.containerId, [0,0,0], [0,0,0])
            
            # Reset orientations to perfectly level
            spreader_pos, _ = p.getBasePositionAndOrientation(self.spreaderId)
            container_pos, _ = p.getBasePositionAndOrientation(self.containerId)
            
            p.resetBasePositionAndOrientation(self.spreaderId, spreader_pos, [0,0,0,1])
            p.resetBasePositionAndOrientation(self.containerId, container_pos, [0,0,0,1])
            
            # Re-establish clean constraint
            if self.container_constraint:
                p.removeConstraint(self.container_constraint)
            
            self.container_constraint = p.createConstraint(
                self.spreaderId, -1,
                self.containerId, -1,
                p.JOINT_FIXED,
                [0, 0, -0.6],
                [0, 0, self.container_height/2],
                [0, 0, 0, 1],
                [0, 0, 0, 1]
            )
            p.changeConstraint(self.container_constraint, maxForce=100000000)
            
            # Wait for physics to stabilize (e.g., 30 steps)
            if self.phase_timer > 30:
                self.phase = "LOWERING_FINAL"
                self.phase_timer = 0
                self.target_spreader_height = 24.6 + self.container_height
                print("Stabilized, now lowering container...")
                
        elif self.phase == "LOWERING_FINAL":
            # Lower container onto ship
            if self.set_spreader_height(self.target_spreader_height, speed):
                self.phase = "COMPLETE"
                self.phase_timer = 0
                print("Container Placed")
                
        # elif self.phase == "DETACHING":
        #     # Maintain position while detaching
        #     self.set_spreader_height(self.target_spreader_height, speed * 0.3)
            
        #     if self.attached and self.container_constraint is not None and self.phase_timer > 60:
        #         p.removeConstraint(self.container_constraint)
        #         self.attached = False
        #         self.container_constraint = None
        #         self.phase = "LIFTING_EMPTY"
        #         self.phase_timer = 0
        #         self.target_spreader_height = 33  # Lift empty spreader
        #         print("Container placed!")
                
        # elif self.phase == "LIFTING_EMPTY":
        #     # Lift empty spreader to safe height
        #     if self.set_spreader_height(self.target_spreader_height, speed * 1.5):
        #         self.phase = "RETURNING"
        #         self.phase_timer = 0
        #         self.current_waypoint_index = len(self.waypoints) - 1
        #         self.waypoint_timer = 0
        #         self.waypoint_reached = False
        #         print("Returning to start position...")
            
        # elif self.phase == "RETURNING":
        #     # Keep spreader high while returning
        #     self.set_spreader_height(self.target_spreader_height, speed)
            
        #     # Return through waypoints in reverse
        #     if self.current_waypoint_index > 0:
        #         self.current_waypoint_index -= 1
                
        #         if self.move_trolley_to_waypoint(self.current_waypoint_index, speed * 2):
        #             # Check if back at start
        #             if self.current_waypoint_index <= 0:
        #                 if self.phase_timer > 120:
        #                     self.phase = "COMPLETE"
        #                     self.phase_timer = 0
        #                     print("Cycle complete! Ready for next container...")
        #     else:
        #         # At start position
        #         if self.phase_timer > 120:
        #             self.phase = "COMPLETE"
        #             self.phase_timer = 0
                    
        # elif self.phase == "COMPLETE":
        #     # Pause before restarting
        #     if self.phase_timer > 60:
        #         self.phase = "WAITING"
        #         self.phase_timer = 0
        #         print("Restarting cycle...")

    def set_camera_view(self, view_name):
        """Set camera to predefined views"""
        views = {
            'top': {'distance': 80, 'yaw': 0, 'pitch': -89, 'target': [0, 20, 15]},
            'front': {'distance': 70, 'yaw': 0, 'pitch': -20, 'target': [0, 20, 15]},
            'side': {'distance': 70, 'yaw': 90, 'pitch': -20, 'target': [0, 20, 15]},
            'iso': {'distance': 80, 'yaw': 45, 'pitch': -30, 'target': [0, 20, 15]},
            'crane': {'distance': 50, 'yaw': 135, 'pitch': -25, 'target': [-10, 5, 20]}
        }
        
        if view_name in views:
            v = views[view_name]
            p.resetDebugVisualizerCamera(
                cameraDistance=v['distance'],
                cameraYaw=v['yaw'],
                cameraPitch=v['pitch'],
                cameraTargetPosition=v['target']
            )

    def setup_synthetic_camera(self):
        """Setup synthetic camera for RGB, Depth and Segmentation views"""
        self.cam_width = 320
        self.cam_height = 240
        self.cam_fov = 60
        self.cam_aspect = self.cam_width / self.cam_height
        self.cam_near = 0.1
        self.cam_far = 100
        
        # Camera position (looking at the crane operation)
        self.cam_position = [0, -25, 20]
        self.cam_distance = 30  # Distance from target
        self.cam_target = [0, 10, 17]
        
    def update_synthetic_camera(self):
        """Update and display synthetic camera views"""
        # Calculate view and projection matrices
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.cam_position,
            cameraTargetPosition=self.cam_target,
            cameraUpVector=[0, 0, 1]
        )
        
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.cam_fov,
            aspect=self.cam_aspect,
            nearVal=self.cam_near,
            farVal=self.cam_far
        )
        
        # Get camera images
        _, _, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.cam_width,
            height=self.cam_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_TINY_RENDERER  # Faster than ER_BULLET_HARDWARE_OPENGL
        )



    def sync_spreader_to_trolley(self):
        """Keep spreader and container aligned with trolley during MOVING only"""
        if self.phase != "MOVING":
            return
        
        trolley_pos, _ = p.getBasePositionAndOrientation(self.trolleyId)
        spreader_pos, spreader_orn = p.getBasePositionAndOrientation(self.spreaderId)
        
        # Calculate Y movement needed
        y_diff = trolley_pos[1] - spreader_pos[1]
        
        if abs(y_diff) > 0.01:
            # Move spreader
            new_spreader_pos = [spreader_pos[0], trolley_pos[1], spreader_pos[2]]
            p.resetBasePositionAndOrientation(self.spreaderId, new_spreader_pos, spreader_orn)
            
            if self.attached and self.container_constraint is not None:
                container_pos, container_orn = p.getBasePositionAndOrientation(self.containerId)
                # Move container by SAME AMOUNT as spreader
                new_container_pos = [
                    container_pos[0],
                    container_pos[1] + y_diff,  # Add the same Y difference
                    container_pos[2]
                ]
                p.resetBasePositionAndOrientation(self.containerId, new_container_pos, container_orn)

    def collect_data_point(self):
        """Collect one data point for dataset"""
        
        # Get current measurements
        imu_reading = self.container_imu.get_measurements()
        
        # Get poses
        spreader_pos, spreader_orn = p.getBasePositionAndOrientation(self.spreaderId)
        container_pos, container_orn = p.getBasePositionAndOrientation(self.containerId)
        trolley_pos, _ = p.getBasePositionAndOrientation(self.trolleyId)
        
        # Calculate cable tension
        cable_tension = self.container_load * 9.81
        if self.phase == "LIFTING":
            cable_tension *= 1.2
        elif self.phase == "LOWERING_FINAL":
            cable_tension *= 1.1
        
        # Store data for Excel export
        self.sensor_data.append({
            'timestamp': imu_reading['timestamp'],
            'phase': self.phase,
            'attached': self.attached,
            'container_x': container_pos[0],
            'container_y': container_pos[1],
            'container_z': container_pos[2],
            'trolley_y': trolley_pos[1],
            'spreader_height': spreader_pos[2],
            'gyro_x': imu_reading['gyro_x'],
            'gyro_y': imu_reading['gyro_y'],
            'gyro_z': imu_reading['gyro_z'],
            'acc_x': imu_reading['acc_x'],
            'acc_y': imu_reading['acc_y'],
            'acc_z': imu_reading['acc_z'],
            'mag_x': imu_reading['mag_x'],
            'mag_y': imu_reading['mag_y'],
            'mag_z': imu_reading['mag_z'],
            'roll': imu_reading['roll'],
            'pitch': imu_reading['pitch'],
            'yaw': imu_reading['yaw'],
            'altitude': imu_reading['altitude'],
            'cable_tension': cable_tension,
            'container_type': self.container_type,
            'container_load': self.container_load,
            'imu_location_x': self.container_imu.local_position[0],
            'imu_location_y': self.container_imu.local_position[1],
            'imu_location_z': self.container_imu.local_position[2]
        })

    def save_dataset(self, filename=None):
        """Save sensor data to Excel"""
        print(f"Saving dataset - Total points collected: {len(self.sensor_data)}")
        
        if not self.sensor_data:
            print("Warning: No data collected to save!")
            return None
        
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"container_imu_data_{timestamp}.xlsx"
        
        df = pd.DataFrame(self.sensor_data)
        
        # Ensure directory exists
        output_dir = os.path.expanduser("~/ContainerSimulation")
        os.makedirs(output_dir, exist_ok=True)
        
        filepath = os.path.join(output_dir, filename)
        
        try:
            df.to_excel(filepath, index=False)
            print(f"Successfully saved {len(df)} rows to: {filepath}")
        except Exception as e:
            print(f"Error saving file: {e}")
            # Try CSV as fallback
            csv_path = filepath.replace('.xlsx', '.csv')
            df.to_csv(csv_path, index=False)
            print(f"Saved as CSV instead: {csv_path}")
        
        return filepath
    
    def run(self):
        """Main simulation loop"""
        print("\n" + "="*60)
        print("PANAMAX CRANE CONTAINER LOADING SIMULATOR")
        print("="*60)
        print("\nSimulating container movement from quay to ship")
        print("\nControls:")
        print("  Slider - Start/Stop automatic cycle")
        print("  C - Cycle camera views")
        print("  J/L - Rotate camera left/right")
        print("  I/K - Rotate camera up/down")
        print("  Arrow Keys - Pan camera")
        print("  U/H - Move RGB Camera Up/Down")
        print("  Z - Save dataset")
        print("  Q - Quit")
        print("="*60 + "\n")

        camera_views = ['iso', 'front', 'side', 'top', 'crane']
        current_view_idx = 0
        running = True
        consecutive_errors = 0
        max_errors = 10
        
        try:
            while running:
                try:
                    keys = p.getKeyboardEvents()
                    
                    # Quit
                    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                        self.save_dataset()
                        running = False
                        break

                    if ord('z') in keys and keys[ord('z')] & p.KEY_WAS_TRIGGERED:
                        self.save_dataset()
                        print("Dataset saved manually")                
                    
                    # Camera controls
                    cam = p.getDebugVisualizerCamera()
                    yaw = cam[8]
                    pitch = cam[9]
                    dist = cam[10]
                    target = list(cam[11])

                    # Update synthetic frequency
                    self.camera_update_counter += 1
                    if self.camera_update_counter >= self.camera_update_interval:
                        self.update_synthetic_camera()
                        self.camera_update_counter = 0
                    
                    if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                        current_view_idx = (current_view_idx + 1) % len(camera_views)
                        self.set_camera_view(camera_views[current_view_idx])
                        p.addUserDebugText(f"Camera: {camera_views[current_view_idx].upper()}", 
                                        [0, 0, 45], textSize=1.5, textColorRGB=[1, 1, 0], lifeTime=2)
                    
                    # J/L for yaw rotation
                    if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
                        yaw -= 5
                    if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:
                        yaw += 5

                    # I/K for pitch rotation  
                    if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:
                        pitch -= 5
                    if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
                        pitch += 5

                    # U/H for RGB Camera
                    if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:
                        self.cam_position[2] += 1
                    if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:
                        self.cam_position[2] -= 1

                    # Arrow keys for panning
                    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                        target[0] -= 0.9
                    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                        target[0] += 0.9
                    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                        target[1] += 0.9
                    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                        target[1] -= 0.9

                    # Update camera
                    p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=yaw, 
                                                cameraPitch=pitch, cameraTargetPosition=target)

                    # Safe parameter reading with error handling
                    auto_mode = False
                    speed = 1
                    
                    try:
                        auto_mode = p.readUserDebugParameter(self.auto_slider) > 0.5
                    except:
                        pass  # Use default auto_mode = False
                    
                    try:
                        speed = int(p.readUserDebugParameter(self.speed_slider))
                    except:
                        speed = 1  # Use default speed
                    
                    # Update cable visuals
                    self.update_cables()
                    
                    if auto_mode:
                        self.handle_automatic_cycle(speed)
                        self.collect_data_point()
                    else:
                        self.phase = "WAITING"
                        if self.attached and self.container_constraint is not None:
                            try:
                                p.removeConstraint(self.container_constraint)
                            except:
                                pass  # Constraint might already be removed
                            self.attached = False
                            self.container_constraint = None
                        
                        # Stop trolley
                        p.resetBaseVelocity(self.trolleyId, linearVelocity=[0, 0, 0])
                    
                    # Update status display
                    self.update_status_display()

                    # Step simulation
                    p.stepSimulation()
                    time.sleep(1.0/240.0)
                    
                    # Reset error counter on successful iteration
                    consecutive_errors = 0
                    
                except Exception as e:
                    consecutive_errors += 1
                    print(f"Loop error #{consecutive_errors}: {e}")
                    
                    if consecutive_errors > max_errors:
                        print("Too many consecutive errors, stopping simulation")
                        break
                    
                    # Try to continue
                    time.sleep(0.01)
                    continue

        except KeyboardInterrupt:
            print("\nSimulation interrupted")
            self.save_dataset()
        except Exception as e:
            print(f"\nCritical error: {e}")
            self.save_dataset()
        finally:
            try:
                self.save_dataset()
            except:
                print("Could not save dataset on exit")
            p.disconnect()


def main():
    """Main function"""
    simulator = ContainerSimulator()
    simulator.run()


if __name__ == "__main__":
    main()