#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TIAGo Robot Controller - Pick and Place Operations
This controller implements a behavior tree architecture for a TIAGo robot
to perform pick and place operations in the Webots simulator.
"""

# ------------------------------------------------------------------------------
# IMPORTS AND CONSTANTS
# ------------------------------------------------------------------------------
import numpy as np
import py_trees
import math
import operator
from ikpy.chain import Chain
from controller import Supervisor
import urdf_parser_py.urdf as urdf_model

# Constants
MAX_MOTOR_SPEED = 6.0
MIN_MOTOR_SPEED = 0.5
DEFAULT_PRINT_INTERVAL = 50
ANGLE_TOLERANCE = 0.05  # About 3 degrees
WAYPOINT_DISTANCE_THRESHOLD = 0.5  # Meters

# ------------------------------------------------------------------------------
# ROBOT INITIALIZATION AND SETUP
# ------------------------------------------------------------------------------
# Create the Robot instance
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Save and parse URDF file
urdf_path = "Robot.urdf"
with open(urdf_path, "w") as file:
    file.write(robot.getUrdf())
urdf_root = urdf_model.URDF.from_xml_file(urdf_path)

# Parse joint limits
joint_limits = {
    joint.name: {
        "lower": joint.limit.lower,
        "upper": joint.limit.upper,
        "velocity": joint.limit.velocity
    }
    for joint in urdf_root.joint_map.values()
    if joint.limit is not None
}

# Create IK chain
base_elements = [
    "base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint",
    "torso_lift_link", "torso_lift_link_TIAGo front arm_joint", "TIAGo front arm_3",
    "arm_1_joint", "TIAGo front arm_3", "arm_2_joint", "arm_2_link",
    "arm_3_joint", "arm_3_link", "arm_4_joint", "arm_4_link",
    "arm_5_joint", "arm_5_link", "arm_6_joint", "arm_6_link",
    "arm_7_joint", "arm_7_link", "arm_7_link_wrist_ft_tool_link_joint",
    "wrist_ft_tool_link", "wrist_ft_tool_link_front_joint"
]


def create_ik_chain():
    """Create an improved IK chain with better configuration"""
    # First create the chain without an active links mask
    chain = Chain.from_urdf_file(
        urdf_path,
        base_elements=base_elements,
        last_link_vector=[0.016, 0, 0],
        name="tiago_arm"
    )

    # Now create an appropriate active links mask based on the actual chain
    active_links_mask = []
    for i, link in enumerate(chain.links):
        # First link (origin) is always inactive
        if i == 0:
            active_links_mask.append(False)
            continue

        # Explicitly mark fixed links as inactive to avoid warnings
        if hasattr(link, "joint_type") and link.joint_type == "fixed":
            active_links_mask.append(False)
        # Only mark revolute joints as active
        elif hasattr(link, "joint_type") and link.joint_type == "revolute":
            active_links_mask.append(True)
        else:
            active_links_mask.append(False)

    # Create a new chain with the proper active links mask
    print(f"Creating chain with {len(chain.links)} links and mask of length {len(active_links_mask)}")
    return Chain(
        links=chain.links,
        active_links_mask=active_links_mask,
        name="tiago_arm"
    )


# Create the IK chain
ik_chain = create_ik_chain()
# Initialize motors and sensors
part_names = [
    "head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
    "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
    "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint",
    "gripper_left_finger_joint", "gripper_right_finger_joint"
]

motors = {}
sensors = {}

# Define a mapping for special sensor names
special_sensor_names = {
    "gripper_left_finger_joint": "gripper_left_sensor_finger_joint",
    "gripper_right_finger_joint": "gripper_right_sensor_finger_joint"
}

# Initialize all devices
for part_name in part_names:
    try:
        motor = robot.getDevice(part_name)
        limit_info = joint_limits.get(part_name)
        motor.setVelocity(limit_info['velocity'] * 0.3 if limit_info else 1.0)

        # Use special sensor names for gripper fingers, otherwise use default naming
        sensor_name = special_sensor_names.get(part_name, f"{part_name}_sensor")
        sensor = robot.getDevice(sensor_name)

        if sensor:
            sensor.enable(timestep)
            sensors[part_name] = sensor
            print(f"Motor '{part_name}' initialized with sensor '{sensor_name}'")
        else:
            print(f"Warning: Sensor '{sensor_name}' not found for motor '{part_name}'")

        motors[part_name] = motor
    except Exception as e:
        print(f"Failed to initialize motor '{part_name}' or its sensor: {e}")

# Print initialization summary
print("Part names:", part_names)
print(f"Number of motors initialized: {len(motors)}")
print(f"Number of sensors initialized: {len(sensors)}")


def initialize_camera(camera, reduce_resolution=False):
    """Initialize camera with proper resource management"""
    if camera:
        # Reduce resolution if needed
        if reduce_resolution:
            width = camera.getWidth()
            height = camera.getHeight()
            camera.setWidth(max(80, width // 2))
            camera.setHeight(max(60, height // 2))

        # Set up camera correctly
        camera.enable(timestep)
        camera.recognitionEnable(timestep)

        # Use grayscale to reduce memory usage if needed
        if reduce_resolution:
            # Enable with lower refresh rate to reduce CPU usage
            camera.setFov(0.8)  # Reduce field of view

        print(f"Camera configured: {camera.getWidth()}x{camera.getHeight()}")


# Enable additional devices
camera = robot.getDevice('camera')
initialize_camera(camera, reduce_resolution=False)

display = robot.getDevice('display')
display.attachCamera(camera)

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

# Enable force feedback for grippers
motors['gripper_left_finger_joint'].enableForceFeedback(timestep)
motors['gripper_right_finger_joint'].enableForceFeedback(timestep)

# Initialize wheel motors
leftMotor = robot.getDevice('wheel_left_joint')
rightMotor = robot.getDevice('wheel_right_joint')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Define predefined positions
starting_position = {
    'torso_lift_joint': 0.3, 'arm_1_joint': 0.71, 'arm_2_joint': 1.02,
    'arm_3_joint': -2.815, 'arm_4_joint': 1.011, 'arm_5_joint': 0,
    'arm_6_joint': 0, 'arm_7_joint': 0, 'gripper_left_finger_joint': 0.045,
    'gripper_right_finger_joint': 0.045, 'head_1_joint': 0, 'head_2_joint': 0
}

# Simplified lift position - based on safe position but higher
lift_position = {
    "torso_lift_joint": 0.3,
    "arm_1_joint": 0.7,
    "arm_2_joint": 0.4,
    "arm_3_joint": -1.5,
    "arm_4_joint": 1.7,
    "arm_5_joint": -1.5,
    "arm_6_joint": 0.0,
    "arm_7_joint": 0.0
}

# Stable place position - controlled extension
place_position = {
    "torso_lift_joint": 0.3, "arm_1_joint": 0.7, "arm_2_joint": 0.0,
    "arm_3_joint": -1.2, "arm_4_joint": 1.2, "arm_5_joint": -1.5,
    "arm_6_joint": 0.0, "arm_7_joint": -1.5
}

table_waypoints = [
    (0.2, -0.7, 0.095)  # Then move to table position
]
home_waypoint = [(0.3, 0.0, 0.095)]  # Simple home position

# Set starting position
for joint, position in starting_position.items():
    motors[joint].setPosition(position)


# ------------------------------------------------------------------------------
# HELPER FUNCTIONS
# ------------------------------------------------------------------------------

def camera_to_world_coordinates(camera_position, camera_offset=0.0):
    """Convert camera-relative coordinates to global coordinates using exact URDF measurements and correction for
    torso lift"""
    # Get robot's current position and orientation
    robot_pos = gps.getValues()
    compass_val = compass.getValues()
    robot_angle = np.arctan2(compass_val[0], compass_val[1])

    # Calculate rotation matrix
    cos_theta = np.cos(robot_angle)
    sin_theta = np.sin(robot_angle)

    # Forward and right vectors
    forward_x = cos_theta
    forward_y = sin_theta
    right_x = -sin_theta
    right_y = cos_theta

    # Get torso lift height and head tilt
    torso_height = 0
    head_tilt = 0
    if 'torso_lift_joint' in sensors:
        torso_height = sensors['torso_lift_joint'].getValue()
    if 'head_2_joint' in sensors:
        head_tilt = sensors['head_2_joint'].getValue()

    # Calculate precise camera height from URDF measurements
    camera_height = robot_pos[2] + 0.891 + torso_height

    # Camera forward offset from robot center
    camera_forward_offset = 0.25  # Forward offset correction

    # Transform camera coordinates to world coordinates
    world_x = robot_pos[0] + forward_x * (camera_position[0] + camera_forward_offset) + right_x * camera_position[1]
    world_y = robot_pos[1] + forward_y * (camera_position[0] + camera_forward_offset) + right_y * camera_position[1]

    # Apply Z correction based on torso height
    # Reference torso height for normal operation
    reference_torso_height = 0.2

    # Z correction factor based on the observed error:
    # When torso_height = 0.35 (0.15 above reference), Z is overestimated by ~0.28m
    z_correction = 0
    if torso_height > reference_torso_height:
        height_diff = torso_height - reference_torso_height
        z_correction = -1.87 * height_diff  # Empirical correction factor

    # Apply the correction to the Z calculation
    world_z = camera_height + camera_position[2] + z_correction

    # Debug info
    print(f"Robot base height: {robot_pos[2]:.3f}m")
    print(f"Torso lift value: {torso_height:.3f}m")
    print(f"Head tilt: {head_tilt:.3f}rad")
    print(f"Total camera height: {camera_height:.3f}m")
    print(f"Camera-relative Z: {camera_position[2]:.3f}m")
    print(f"Z correction: {z_correction:.3f}m")
    print(f"World Z: {world_z:.3f}m")

    return [world_x, world_y, world_z]


def calculate_approach_offsets(robot_pos, target_pos):
    """Calculate approach vector based on robot-to-target relationship"""
    # Get robot orientation from compass
    compass_val = compass.getValues()
    robot_angle = np.arctan2(compass_val[0], compass_val[1])

    # Calculate vector from robot to target
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    distance = np.sqrt(dx ** 2 + dy ** 2)

    # Calculate approach angle (from robot to jar)
    approach_angle = np.arctan2(dy, dx)

    # Calculate approach vector (opposite direction)
    # This creates a vector pointing from the jar toward the robot
    approach_vector_angle = approach_angle + np.pi  # Reverse direction

    # Create approach offset (a small vector pointing from jar toward robot)
    offset_magnitude = 0.045  # 8cm offset, adjust as needed
    offset_x = offset_magnitude * np.cos(approach_vector_angle)
    offset_y = offset_magnitude * np.sin(approach_vector_angle)

    print(f"Robot at ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}), angle {np.degrees(robot_angle):.1f}°")
    print(f"Target at ({target_pos[0]:.3f}, {target_pos[1]:.3f}), distance: {distance:.3f}m")
    print(f"Approach vector: {np.degrees(approach_vector_angle):.1f}°")
    print(f"Calculated offsets: ({offset_x:.3f}, {offset_y:.3f})")

    return offset_x, offset_y


def calculate_inverse_kinematics(target_position, offset_x=0.0, offset_y=0.0):
    """Calculate inverse kinematics with proper bounds checking, no precision mode."""

    # Apply offsets directly to the target position
    final_target = [
        target_position[0] + offset_x,
        target_position[1] + offset_y,
        target_position[2]
    ]
    print(f"Attempting IK for final target: {final_target}")

    # Gather and clamp initial joint positions
    initial_position = [
        sensors[joint.name].getValue() if joint.name in sensors else 0.0
        for joint in ik_chain.links
    ]
    for i, link in enumerate(ik_chain.links):
        if link.name in joint_limits:
            lower = joint_limits[link.name]['lower']
            upper = joint_limits[link.name]['upper']
            initial_position[i] = np.clip(initial_position[i], lower, upper)
    print(f"Initial joint positions (clamped): {initial_position}")

    # Perform the IK calculation
    try:
        ik_results = ik_chain.inverse_kinematics(
            target_position=final_target,
            initial_position=initial_position,
            target_orientation=[0, 0, 1],
            orientation_mode="Y"
        )
        print("IK solution found successfully")
        return {
            link.name: ik_results[i]
            for i, link in enumerate(ik_chain.links)
            if link.name in part_names
        }
    except ValueError as e:
        print(f"IK solver error: {e}")
        return None


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return ((angle + math.pi) % (2 * math.pi)) - math.pi


def angle_difference(angle1, angle2):
    """Calculate the difference between two angles"""
    return normalize_angle(angle1 - angle2)


def get_target_position(behavior_name):
    """
    Get target position from the blackboard.

    Args:
        behavior_name: Name of the calling behavior (for logging)

    Returns:
        List[float]: Target position [x, y, z] or None if not available
    """
    try:
        blackboard = py_trees.blackboard.Blackboard()
        target_position = blackboard.get("target_position")
        if target_position is not None:
            print(f"{behavior_name}: Using target position from blackboard: {target_position}")
            return target_position
    except Exception as e:
        print(f"{behavior_name}: Error reading from blackboard: {e}")

    print(f"{behavior_name}: No target position available!")
    return None

# ------------------------------------------------------------------------------
# BEHAVIOR TREE NODE CLASSES
# ------------------------------------------------------------------------------

class EnhancedObjectRecognizer(py_trees.behaviour.Behaviour):
    def __init__(self, name, z_offset=0.0, samples=5, timeout=3.0):
        super(EnhancedObjectRecognizer, self).__init__(name)
        self.z_offset = z_offset
        self.samples = samples
        self.timeout = timeout
        self.target_position = None
        self.object_name = None
        self.start_time = None
        print(f"Created {name} with timeout {self.timeout}s")

    def initialise(self):
        self.target_position = None
        self.object_name = None
        self.start_time = robot.getTime()
        print(f"{self.name}: Initialized recognition with timeout {self.timeout}s")

    def update(self):
        # Safety check for start_time
        if self.start_time is None:
            self.start_time = robot.getTime()

        # Check for timeout
        if (robot.getTime() - self.start_time) > self.timeout:
            print(f"{self.name}: Object recognition timed out after {self.timeout}s")
            return py_trees.common.Status.FAILURE

        object_positions = []
        sample_count = 1 if "After Scan" in self.name else self.samples

        for _ in range(sample_count):
            objects = camera.getRecognitionObjects()
            if objects:
                print(f"{self.name}: Camera sees {len(objects)} objects")

            for obj in objects:
                try:
                    model_name = obj.getModel()
                    if model_name != "jam jar" and model_name != "honey jar":
                        continue

                    # Get position in camera coordinates
                    camera_position = list(obj.getPosition())

                    # Convert to absolute world coordinates using GPS and compass
                    world_position = camera_to_world_coordinates(camera_position, self.z_offset)

                    # Basic filter to eliminate impossible positions
                    if world_position[2] < 0 or world_position[2] > 2.0:
                        continue

                    object_positions.append((world_position, model_name))
                    print(f"{self.name}: Found object: {model_name} at {world_position}")
                except Exception as e:
                    print(f"{self.name}: Error processing recognition object: {e}")

        # If we found objects, choose the closest one
        if object_positions:
            # Get current robot position from GPS
            robot_pos = gps.getValues()

            # Calculate distances to robot and sort
            for i, (pos, _) in enumerate(object_positions):
                dist = sum((pos[j] - robot_pos[j]) ** 2 for j in range(3))
                object_positions[i] = (pos, object_positions[i][1], dist)

            # Sort by distance
            object_positions.sort(key=lambda x: x[2])
            closest_object = object_positions[0]

            self.target_position = closest_object[0]
            self.object_name = closest_object[1]

            # Store results on blackboard without any rotation angle
            try:
                blackboard = py_trees.blackboard.Blackboard()
                blackboard.set("target_position", self.target_position)
                blackboard.set("object_name", self.object_name)
                print(
                    f"{self.name}: Stored target position {self.target_position} and object {self.object_name} on blackboard")
            except Exception as e:
                print(f"{self.name}: Error storing on blackboard: {e}")

            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class ComprehensiveScanner(py_trees.behaviour.Behaviour):
    def __init__(self, name, total_angles=8, angle_increment=45, rotation_speed=1.0):
        super(ComprehensiveScanner, self).__init__(name)
        self.total_angles = total_angles
        self.angle_increment = angle_increment
        self.rotation_speed = rotation_speed
        self.current_angle_index = 0
        self.rotation_complete = False
        self.rotation_duration = abs(math.radians(angle_increment) / rotation_speed)

    def initialise(self):
        self.current_angle_index = 0
        self.start_time = robot.getTime()
        self.rotation_complete = False
        print(f"{self.name}: Starting comprehensive scan with {self.total_angles} angles")

        # Setup camera position
        motors['torso_lift_joint'].setPosition(0.35)
        motors['head_1_joint'].setPosition(0.0)
        motors['head_2_joint'].setPosition(-0.2)

    def update(self):
        current_time = robot.getTime()
        time_elapsed = current_time - self.start_time

        # Check if the current rotation is complete
        if self.rotation_complete:
            # Wait for stabilization
            if time_elapsed > self.rotation_duration + 0.3:
                # Check if we've completed all angles
                if self.current_angle_index >= self.total_angles - 1:
                    print(f"{self.name}: Completed full scan")
                    return py_trees.common.Status.SUCCESS

                # Move to next angle
                self.current_angle_index += 1
                self.start_time = current_time
                self.rotation_complete = False
                print(f"{self.name}: Moving to angle {self.current_angle_index + 1}/{self.total_angles}")
                return py_trees.common.Status.RUNNING
            else:
                # Still stabilizing
                leftMotor.setVelocity(0.0)
                rightMotor.setVelocity(0.0)
                return py_trees.common.Status.RUNNING

        # Check if current rotation is finished
        if time_elapsed >= self.rotation_duration:
            self.rotation_complete = True
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            print(f"{self.name}: Rotation {self.current_angle_index + 1}/{self.total_angles} complete")

            # After each rotation, try to recognize
            object_recognizer = EnhancedObjectRecognizer(
                f"Recognize at position {self.current_angle_index + 1}",
                timeout=2.0
            )
            object_recognizer.initialise()
            recognize_status = object_recognizer.update()

            if recognize_status == py_trees.common.Status.SUCCESS:
                print(f"{self.name}: Object recognized after rotation")
                # No need to store rotation angle anymore
                return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.RUNNING

        # Continue rotating
        leftMotor.setVelocity(self.rotation_speed)
        rightMotor.setVelocity(-self.rotation_speed)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)


class GraspController(py_trees.behaviour.Behaviour):
    def __init__(self, name, force_threshold=-12.0):
        super(GraspController, self).__init__(name)
        self.force_threshold = force_threshold
        self.state = "APPROACHING"
        self.grip_width = 0.045  # Fully open
        self.verification_time = 0.5
        self.verification_start_time = None
        print(f"{name}: Created simplified grasp controller with threshold={force_threshold}")

    def initialise(self):
        self.state = "APPROACHING"
        self.grip_width = 0.045
        self.verification_start_time = None
        print(f"{self.name}: Starting simplified grasp sequence")

        # Reset blackboard grasp_success
        try:
            blackboard = py_trees.blackboard.Blackboard()
            blackboard.set("grasp_success", False)
        except Exception as e:
            print(f"Error initializing blackboard: {e}")

    def update(self):
        # Get force feedback and position
        left_force = motors['gripper_left_finger_joint'].getForceFeedback()
        right_force = motors['gripper_right_finger_joint'].getForceFeedback()
        current_left = sensors['gripper_left_finger_joint'].getValue()
        current_right = sensors['gripper_right_finger_joint'].getValue()

        # Simple state machine for grasping
        if self.state == "APPROACHING":
            # Move fingers closer slowly
            self.grip_width = max(0.0, self.grip_width - 0.001)
            motors['gripper_left_finger_joint'].setPosition(self.grip_width)
            motors['gripper_right_finger_joint'].setPosition(self.grip_width)

            # Check if both forces indicate contact - SIMPLIFIED CONDITION!
            if abs(left_force) >= abs(self.force_threshold) and abs(right_force) >= abs(self.force_threshold):
                print(f"{self.name}: Contact detected: L={left_force:.2f}, R={right_force:.2f}")
                self.state = "VERIFYING"
                self.verification_start_time = robot.getTime()

        elif self.state == "VERIFYING":
            # Apply a bit more pressure
            target_width = max(0.0, self.grip_width - 0.001)
            motors['gripper_left_finger_joint'].setPosition(target_width)
            motors['gripper_right_finger_joint'].setPosition(target_width)

            current_time = robot.getTime()
            if current_time - self.verification_start_time >= self.verification_time:
                # SUPER SIMPLE VERIFICATION: If both grippers detect force, grasp is good!
                if abs(left_force) >= abs(self.force_threshold) and abs(right_force) >= abs(self.force_threshold):
                    print(f"{self.name}: Grasp successful! L={left_force:.2f}, R={right_force:.2f}")

                    # Set grasp success on blackboard
                    try:
                        blackboard = py_trees.blackboard.Blackboard()
                        blackboard.set("grasp_success", True)
                        print(f"{self.name}: Set grasp_success to True")
                    except Exception as e:
                        print(f"Error setting blackboard: {e}")

                    return py_trees.common.Status.SUCCESS
                else:
                    print(f"{self.name}: Grasp verification failed, retrying")
                    self.state = "APPROACHING"

        # Check for failure conditions - fingers fully closed with no force
        if current_left < 0.005 and current_right < 0.005 and abs(left_force) < abs(self.force_threshold):
            print(f"{self.name}: Grasp failed - fingers closed but no force detected")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Reset blackboard on termination
        try:
            blackboard = py_trees.blackboard.Blackboard()
            blackboard.set("grasp_success", new_status == py_trees.common.Status.SUCCESS)
            print(f"{self.name}: Set grasp_success to {new_status == py_trees.common.Status.SUCCESS}")
        except Exception as e:
            print(f"Error setting blackboard: {e}")

        # If failed, reopen gripper
        if new_status == py_trees.common.Status.FAILURE:
            print(f"{self.name}: Grasp failed - reopening gripper")
            motors['gripper_left_finger_joint'].setPosition(0.045)
            motors['gripper_right_finger_joint'].setPosition(0.045)


# --- Navigation Behaviors ---
class MoveToObject(py_trees.behaviour.Behaviour):
    """Navigate to a target using GPS and compass for direct navigation"""

    def __init__(self, name, recognize_object, gps, compass, move_arm_behavior, camera):
        super(MoveToObject, self).__init__(name)
        self.current_target = None
        self._recognize_object = recognize_object
        self.gps = gps
        self.compass = compass
        self.move_arm_behavior = move_arm_behavior
        self.camera = camera

        # Distance thresholds
        self.arm_adjustment_distance = 1.25
        self.very_close_distance = 1.0

        # Simplified state machine - removed rotation correction
        self.state = "INITIAL"
        self.start_time = None
        self.stabilization_start_time = None

        # Control parameters
        self.Kp_linear = 0.9
        self.Kp_angular = 1.0
        self.Kd_angular = 0.8
        self.max_speed = 3.0

        # Timing parameters
        self.stabilization_duration = 0.5

    def initialise(self):
        self.state = "INITIAL"
        self.start_time = robot.getTime()

        # Force wheels to stop and stabilize
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        robot.step(timestep * 5)

        # Get target position
        self.current_target = get_target_position(
            self.name,
            recognizer=self._recognize_object
        )

        if self.current_target is None:
            return py_trees.common.Status.FAILURE

        # Reset motion variables
        self.prev_left_speed = 0.0
        self.prev_right_speed = 0.0
        self.prev_alpha = 0.0
        self.last_time = robot.getTime()

        # Skip to orienting directly - no rotation correction needed with GPS
        self.state = "ORIENTING"

        # Log initial position and target
        robot_pos = self.gps.getValues()
        print(f"{self.name}: Starting from GPS position ({robot_pos[0]:.3f}, {robot_pos[1]:.3f})")
        print(f"{self.name}: Target at ({self.current_target[0]:.3f}, {self.current_target[1]:.3f})")

    def update(self):
        if self.current_target is None:
            return py_trees.common.Status.FAILURE

        # Get current position and orientation from GPS and compass
        current_pos = self.gps.getValues()
        compass_values = self.compass.getValues()
        theta = np.arctan2(compass_values[0], compass_values[1])

        # Calculate distance and angle to target using GPS coordinates
        target_x, target_y = self.current_target[:2]
        dx = target_x - current_pos[0]
        dy = target_y - current_pos[1]
        rho = np.sqrt(dx * dx + dy * dy)
        target_angle = np.arctan2(dy, dx)
        alpha = angle_difference(target_angle, theta)

        # Calculate derivatives for control
        current_time = robot.getTime()
        dt = max(current_time - self.last_time, timestep / 1000.0)
        alpha_rate = (alpha - self.prev_alpha) / dt
        self.prev_alpha = alpha
        self.last_time = current_time

        # Print status occasionally
        if current_time % 1.0 < 0.01:
            print(f"State: {self.state}, Distance: {rho:.4f}m, Angle: {np.degrees(alpha):.2f}°")
            print(f"Robot at ({current_pos[0]:.3f}, {current_pos[1]:.3f}), Target at ({target_x:.3f}, {target_y:.3f})")
            print(f"Robot heading: {np.degrees(theta):.2f}°, Target bearing: {np.degrees(target_angle):.2f}°")

        # STATE MACHINE - simplified without rotation correction
        if self.state == "ORIENTING":
            # Pure orientation phase
            if abs(alpha) < ANGLE_TOLERANCE:
                leftMotor.setVelocity(0.0)
                rightMotor.setVelocity(0.0)
                print(f"Orientation complete. Angle: {np.degrees(alpha):.2f}°")
                self.state = "APPROACHING"
                return py_trees.common.Status.RUNNING

            # Proportional-derivative control for turning
            turn_speed = 1.0 * alpha - 0.5 * alpha_rate
            turn_speed = np.clip(turn_speed, -1.8, 1.8)
            leftMotor.setVelocity(-turn_speed)
            rightMotor.setVelocity(turn_speed)
            return py_trees.common.Status.RUNNING

        elif self.state == "APPROACHING":
            # Check for arm adjustment
            if rho < self.arm_adjustment_distance:
                leftMotor.setVelocity(0.0)
                rightMotor.setVelocity(0.0)
                print(f"Stopping to adjust arm at {rho:.2f}m")
                self.stabilization_start_time = current_time
                self.state = "STABILIZING"
                return py_trees.common.Status.RUNNING

            # Keep approaching
            linear_speed = self.Kp_linear * rho
            angular_speed = self.Kp_angular * alpha - self.Kd_angular * alpha_rate

            # Slow down when close
            if rho < 1.5:
                linear_speed *= (0.5 + rho / 3.0)

            # Calculate wheel speeds
            leftSpeed = linear_speed - angular_speed
            rightSpeed = linear_speed + angular_speed

            # Apply limits and set motors
            leftMotor.setVelocity(np.clip(leftSpeed, -self.max_speed, self.max_speed))
            rightMotor.setVelocity(np.clip(rightSpeed, -self.max_speed, self.max_speed))
            return py_trees.common.Status.RUNNING

        elif self.state == "STABILIZING":
            # Wait for robot to fully stop
            if current_time - self.stabilization_start_time < self.stabilization_duration:
                leftMotor.setVelocity(0.0)
                rightMotor.setVelocity(0.0)
                return py_trees.common.Status.RUNNING

            print("Robot stabilized, starting arm adjustment")
            self.state = "ADJUSTING_ARM"
            if self.move_arm_behavior:
                self.move_arm_behavior.initialise()
            self.arm_adjustment_start_time = current_time
            return py_trees.common.Status.RUNNING

        elif self.state == "ADJUSTING_ARM":
            # Keep motors stopped during arm adjustment
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)

            # Update arm movement
            if self.move_arm_behavior:
                arm_status = self.move_arm_behavior.update()

                # Check if arm movement completed or timed out
                elapsed = current_time - self.arm_adjustment_start_time
                if arm_status == py_trees.common.Status.SUCCESS or elapsed > 2.5:
                    print(f"Arm adjustment complete, continuing approach")

                    # Check if we're already close enough
                    if rho < self.very_close_distance:
                        print(f"Arrived at target, distance: {rho:.2f}m")
                        return py_trees.common.Status.SUCCESS

                    # Otherwise continue approach
                    self.state = "FINAL_APPROACH"
                    return py_trees.common.Status.RUNNING

            return py_trees.common.Status.RUNNING

        elif self.state == "FINAL_APPROACH":
            # Final approach with more careful control
            if rho < self.very_close_distance:
                leftMotor.setVelocity(0.0)
                rightMotor.setVelocity(0.0)
                print(f"Arrived at target, distance: {rho:.2f}m")
                return py_trees.common.Status.SUCCESS

            # Similar to APPROACHING but more conservative
            linear_speed = 0.5 * self.Kp_linear * rho
            angular_speed = self.Kp_angular * alpha - self.Kd_angular * alpha_rate

            leftSpeed = linear_speed - angular_speed
            rightSpeed = linear_speed + angular_speed

            # Lower max speed for final approach
            max_final_speed = 1.5
            leftMotor.setVelocity(np.clip(leftSpeed, -max_final_speed, max_final_speed))
            rightMotor.setVelocity(np.clip(rightSpeed, -max_final_speed, max_final_speed))
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING


class MoveToWaypoint(py_trees.behaviour.Behaviour):

    def __init__(self, name, waypoints, timeout=30.0):
        super(MoveToWaypoint, self).__init__(name)
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.timeout = timeout

        # Motion parameters - ADJUSTED FOR TIGHTER TURNS
        self.angle_threshold = 0.1  # Reduced from 0.2 for more precise angle control
        self.turn_speed = 2.0  # Increased from 1.5 for faster turning
        self.max_forward_speed = 3.5  # Slightly reduced from 4.0 for better control
        self.distance_threshold = WAYPOINT_DISTANCE_THRESHOLD

        # Controller parameters - BALANCED FOR STABILITY AND RESPONSIVENESS
        self.prev_left_speed = 0.0
        self.prev_right_speed = 0.0
        self.smoothing_factor = 0.4  # Increased for more smoothing to reduce oscillations
        self.prev_alpha = 0.0
        self.kd_angular = 0.9  # Reduced to prevent over-correction
        self.angular_deadband = 0.05  # Deadband to prevent tiny corrections that cause wobbling

        # Timing variables
        self.start_time = None
        self.last_time = None

        # Add progress tracking to detect if robot is stuck
        self.last_position = None
        self.stuck_time = None
        self.progress_check_interval = 3.0  # Check progress every 3 seconds
        self.last_progress_check = 0
        self.min_progress_distance = 0.05  # Minimum progress expected in 3 seconds

    def initialise(self):
        self.current_waypoint_index = 0
        self.current_waypoint = self.waypoints[self.current_waypoint_index]

        # Reset control variables
        self.start_time = robot.getTime()
        self.last_time = self.start_time
        self.prev_left_speed = 0.0
        self.prev_right_speed = 0.0
        self.prev_alpha = 0.0

        # Initialize progress tracking
        self.last_position = gps.getValues()[:2]
        self.last_progress_check = self.start_time
        self.stuck_time = None

        # Log the starting GPS position and waypoint targets
        print(f"Starting navigation from GPS position: {self.last_position}")
        print(f"Target waypoint sequence: {self.waypoints}")

    def update(self):
        try:
            current_time = robot.getTime()

            # Check for timeout
            if current_time - self.start_time > self.timeout:
                print(f"{self.name} timed out after {self.timeout} seconds")
                leftMotor.setVelocity(0.0)
                rightMotor.setVelocity(0.0)
                return py_trees.common.Status.SUCCESS

            # Get current position and orientation from GPS and compass
            current_pos = gps.getValues()
            xw, yw = current_pos[:2]
            compass_values = compass.getValues()
            theta = np.arctan2(compass_values[0], compass_values[1])

            # Calculate distance and angle to current waypoint
            target_x, target_y, _ = self.current_waypoint
            rho = np.sqrt((xw - target_x) ** 2 + (yw - target_y) ** 2)
            target_angle = np.arctan2(target_y - yw, target_x - xw)
            alpha = angle_difference(target_angle, theta)

            # Check if we're making progress (every few seconds)
            if current_time - self.last_progress_check > self.progress_check_interval:
                dx = xw - self.last_position[0]
                dy = yw - self.last_position[1]
                distance_moved = np.sqrt(dx * dx + dy * dy)

                # Log detailed GPS position and progress
                print(f"GPS Position: ({xw:.3f}, {yw:.3f}), heading: {np.degrees(theta):.1f}°")
                print(
                    f"Distance to waypoint: {rho:.3f}m, moved {distance_moved:.3f}m in last {self.progress_check_interval:.1f}s")

                # Check if we're stuck
                if distance_moved < self.min_progress_distance:
                    if self.stuck_time is None:
                        self.stuck_time = current_time
                        print(f"Warning: Robot may be stuck, minimal progress detected ({distance_moved:.3f}m)")
                    elif current_time - self.stuck_time > 5.0:
                        # If stuck for more than 5 seconds, try to recover
                        print(f"Robot appears stuck at {current_pos}, attempting recovery")

                        # Recovery strategy: rotate slightly to change trajectory
                        leftMotor.setVelocity(-self.turn_speed * 1.5)  # More aggressive recovery turn
                        rightMotor.setVelocity(self.turn_speed * 1.5)
                        robot.step(timestep * 15)  # Rotate for longer time

                        # Reset stuck detection
                        self.stuck_time = None
                else:
                    self.stuck_time = None  # Reset stuck detection if we're moving

                # Update for next progress check
                self.last_position = [xw, yw]
                self.last_progress_check = current_time

            # Print status occasionally (using simplified condition)
            if current_time % 1.0 < timestep / 1000.0:
                print(
                    f"Moving to waypoint: {self.current_waypoint}, distance: {rho:.4f}m, angle: {np.degrees(alpha):.2f}°")

            # Calculate time derivatives for control
            dt = max(current_time - self.last_time, timestep / 1000.0)
            alpha_rate = (alpha - self.prev_alpha) / dt
            self.prev_alpha = alpha
            self.last_time = current_time

            # Check if we've reached the current waypoint
            if rho < self.distance_threshold:
                leftMotor.setVelocity(0.0)
                rightMotor.setVelocity(0.0)
                print(f"Reached waypoint: {self.current_waypoint} (GPS position: {current_pos[:2]})")

                # Move to next waypoint
                self.current_waypoint_index += 1

                # If we've reached the end of waypoints, we're done
                if self.current_waypoint_index >= len(self.waypoints):
                    print(f"Completed all waypoints! Final GPS position: {current_pos[:2]}")
                    return py_trees.common.Status.SUCCESS

                # Set up next waypoint and continue
                self.current_waypoint = self.waypoints[self.current_waypoint_index]
                return py_trees.common.Status.RUNNING

            # ANTI-WOBBLE STEERING CONTROL - Modified for stability and tighter turns
            # Apply deadband to prevent tiny corrections that cause wobbling
            if abs(alpha) < self.angular_deadband and rho > 0.3:
                # If angle is very small and we're not too close, just go straight
                alpha_for_control = 0
                alpha_rate_for_control = 0
            else:
                alpha_for_control = alpha
                alpha_rate_for_control = alpha_rate

            # If large angle difference, prioritize turning in place first
            if abs(alpha_for_control) > self.angle_threshold * 1.5:  # Less aggressive turning threshold
                # Turn in place with controlled intensity when angle is large
                turn_intensity = np.sign(alpha_for_control) * min(1.2, abs(alpha_for_control))  # Less aggressive
                leftSpeed = -self.turn_speed * turn_intensity
                rightSpeed = self.turn_speed * turn_intensity
            else:
                # More balanced approach for smaller angles
                speed_factor = min(1.0, max(0.4, rho / 0.5))  # Adjusted min speed factor

                # Adjust turn component based on angle with smoother transition
                forward_speed = self.max_forward_speed * speed_factor * (
                        1 - 0.7 * abs(alpha_for_control) / self.angle_threshold)

                # Softer turn components with damping to reduce oscillation
                turn_component = (self.turn_speed * alpha_for_control / np.pi) - (
                        self.kd_angular * alpha_rate_for_control)

                # Apply non-linear smoothing to turn component to reduce oscillations
                turn_component = np.sign(turn_component) * (1 - np.exp(-2 * abs(turn_component)))

                if self.name.startswith("Move to Table Waypoint"):
                    forward_speed *= 0.7  # Slower for table approach

                leftSpeed = forward_speed - turn_component * self.turn_speed
                rightSpeed = forward_speed + turn_component * self.turn_speed

            # Apply speed constraints
            leftSpeed = np.clip(leftSpeed, -self.max_forward_speed, self.max_forward_speed)
            rightSpeed = np.clip(rightSpeed, -self.max_forward_speed, self.max_forward_speed)

            # Enforce minimum speed for mobility
            if abs(leftSpeed) < MIN_MOTOR_SPEED and leftSpeed != 0:
                leftSpeed = np.sign(leftSpeed) * MIN_MOTOR_SPEED
            if abs(rightSpeed) < MIN_MOTOR_SPEED and rightSpeed != 0:
                rightSpeed = np.sign(rightSpeed) * MIN_MOTOR_SPEED

            # Apply smoothing to motor commands (reduced smoothing for responsiveness)
            self.prev_left_speed = self.smoothing_factor * self.prev_left_speed + (
                    1 - self.smoothing_factor) * leftSpeed
            self.prev_right_speed = self.smoothing_factor * self.prev_right_speed + (
                    1 - self.smoothing_factor) * rightSpeed

            # Set motor speeds
            leftMotor.setVelocity(self.prev_left_speed)
            rightMotor.setVelocity(self.prev_right_speed)

            return py_trees.common.Status.RUNNING

        except Exception as e:
            print(f"Error in MoveToWaypoint: {e}")
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            return py_trees.common.Status.FAILURE


# --- Manipulation Behaviors ---
class MoveArmIK(py_trees.behaviour.Behaviour):

    def __init__(self, name, offset_x=0.0, offset_y=0.0, tolerance=0.015, timeout=5.0):
        super(MoveArmIK, self).__init__(name)
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.tolerance = tolerance
        self.timeout = timeout

        self.movement_started = False
        self.movement_complete = False
        self.target_angles = None
        self.start_time = None

        # A simple pre-grasp posture (adjust as needed)
        self.pre_grasp_position = {
            "torso_lift_joint": 0.3,
            "arm_1_joint": 0.7,
            "arm_2_joint": 0.4,
            "arm_3_joint": -1.5,
            "arm_4_joint": 1.7,
            "arm_5_joint": -1.5,
            "arm_6_joint": 0.0,
            "arm_7_joint": 0.0
        }

    def initialise(self):
        """Reset internal flags and data each time this behavior starts."""
        self.movement_started = False
        self.movement_complete = False
        self.target_angles = None
        self.start_time = None

    def update(self):
        """Main behavior update cycle."""
        # If we're already done, report SUCCESS
        if self.movement_complete:
            return py_trees.common.Status.SUCCESS

        # 1) Start the IK movement if not yet started
        if not self.movement_started:
            self.start_time = robot.getTime()

            # (a) Get target position (from blackboard or a fallback)
            target_position = get_target_position(self.name)
            if not target_position:
                print(f"{self.name}: No valid target position available.")
                return py_trees.common.Status.FAILURE

            print(f"{self.name}: Target position: {target_position}")

            # (b) Move arm to a pre-grasp posture
            for joint, position in self.pre_grasp_position.items():
                if joint in motors:
                    motors[joint].setPosition(position)

            # Wait a short moment for the arm to settle
            robot.step(timestep * 5)

            # (c) Compute offsets based on approach
            current_robot_pos = gps.getValues()[:2]
            dx, dy = calculate_approach_offsets(current_robot_pos, target_position[:2])

            # (d) Solve IK
            self.target_angles = calculate_inverse_kinematics(
                target_position,
                offset_x=dx + self.offset_x,
                offset_y=dy + self.offset_y
            )
            if not self.target_angles:
                print(f"{self.name}: Failed to calculate IK solution.")
                return py_trees.common.Status.FAILURE

            # (e) Set the joint motors to the IK solution
            for joint, angle in self.target_angles.items():
                if joint in motors:
                    motors[joint].setPosition(angle)

            self.movement_started = True
            print(f"{self.name}: Started arm movement with angles {self.target_angles}")
            return py_trees.common.Status.RUNNING

        # 2) If movement is started, we check progress
        current_time = robot.getTime()
        if (current_time - self.start_time) > self.timeout:
            print(f"{self.name}: Movement timed out—considering it complete.")
            self.movement_complete = True
            return py_trees.common.Status.SUCCESS

        # 3) Check if all joints reached their targets within tolerance
        all_in_place = True
        for joint, target_angle in self.target_angles.items():
            if joint in sensors:
                current_angle = sensors[joint].getValue()
                if abs(target_angle - current_angle) > self.tolerance:
                    all_in_place = False
                    break

        if all_in_place:
            print(f"{self.name}: Arm reached target positions.")
            self.movement_complete = True
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class MoveToPosition(py_trees.behaviour.Behaviour):
    """Move joints to a predefined position"""

    def __init__(self, name, joint_targets, tolerance=0.02, timeout=10.0):
        super(MoveToPosition, self).__init__(name)
        self.joint_targets = joint_targets
        self.tolerance = tolerance
        self.movement_complete = False
        self.print_interval = 20
        self.timeout = timeout
        self.start_time = None
        self.progress_time = None
        self.progress_threshold = 3.0  # Consider success if no progress for 3 seconds

    def initialise(self):
        self.movement_complete = False
        self.update_count = 0
        self.start_time = robot.getTime()
        self.progress_time = self.start_time
        self.last_errors = {}

        # Set all joint positions at initialization
        for joint, target in self.joint_targets.items():
            if joint in motors:
                motors[joint].setPosition(target)
                if joint in sensors:
                    self.last_errors[joint] = abs(target - sensors[joint].getValue())
                print(f"Setting {joint} to position {target}")

    def update(self):
        if self.movement_complete:
            return py_trees.common.Status.SUCCESS

        current_time = robot.getTime()

        # Check for timeout
        if current_time - self.start_time > self.timeout:
            print(f"{self.name} timed out after {self.timeout} seconds, considering it complete")
            self.movement_complete = True
            return py_trees.common.Status.SUCCESS

        # Check if we're making progress
        making_progress = False
        all_joints_in_position = True

        for joint, target in self.joint_targets.items():
            if joint not in sensors:
                continue

            current_position = sensors[joint].getValue()
            error = abs(target - current_position)

            # Check if this joint is still moving
            if joint in self.last_errors:
                if abs(self.last_errors[joint] - error) > 0.005:
                    making_progress = True
                self.last_errors[joint] = error

            if error > self.tolerance:
                motors[joint].setPosition(target)
                all_joints_in_position = False

                # Print detailed information for joints that are far from target
                if error > 0.1 and self.update_count % self.print_interval == 0:
                    print(f"Joint {joint} at {current_position:.4f}, target: {target:.4f}, error: {error:.4f}")

        # If we're not making progress for a while, consider it done
        if making_progress:
            self.progress_time = current_time
        elif current_time - self.progress_time > self.progress_threshold:
            print(f"{self.name} stopped making progress, considering it complete")
            self.movement_complete = True
            return py_trees.common.Status.SUCCESS

        self.update_count += 1
        if self.update_count % self.print_interval == 0:
            print(f"{self.name} in progress... ({current_time - self.start_time:.1f}s elapsed)")

        if all_joints_in_position:
            print(f"{self.name} completed successfully.")

            # If this is the lift position after grasping, ensure the object is lifted
            if self.name == "Move to Lift Position":
                # Make sure the arm is raised high enough
                motors['torso_lift_joint'].setPosition(0.35)  # Raise torso
                motors['arm_4_joint'].setPosition(1.5)  # Keep wrist aligned

                # Wait a moment for stability
                robot.step(timestep * 5)

            self.movement_complete = True
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class OpenGripper(py_trees.behaviour.Behaviour):
    """Open the gripper to release an object with verification"""

    def __init__(self, name, open_position=0.045, timeout=2.0):
        super(OpenGripper, self).__init__(name)
        self.open_position = open_position
        self.timeout = timeout
        self.start_time = None
        self.gripper_opened = False

    def initialise(self):
        self.start_time = robot.getTime()
        self.gripper_opened = False
        print(f"{self.name}: Opening gripper to position {self.open_position}")

        # Set gripper to fully open position
        motors['gripper_left_finger_joint'].setPosition(self.open_position)
        motors['gripper_right_finger_joint'].setPosition(self.open_position)

    def update(self):
        current_time = robot.getTime()

        # Check if we've been running too long
        if current_time - self.start_time > self.timeout:
            print(f"{self.name}: Timeout reached, considering gripper opened")
            return py_trees.common.Status.SUCCESS

        # Check if gripper is fully open
        left_pos = sensors['gripper_left_finger_joint'].getValue()
        right_pos = sensors['gripper_right_finger_joint'].getValue()

        if abs(left_pos - self.open_position) < 0.005 and abs(right_pos - self.open_position) < 0.005:
            if not self.gripper_opened:
                print(f"{self.name}: Gripper fully opened")
                self.gripper_opened = True
                # Wait a bit longer after reaching position
                self.start_time = current_time - self.timeout + 0.5

        # Check if we've waited enough after opening
        if self.gripper_opened and current_time - self.start_time > 0.5:
            print(f"{self.name}: Gripper open sequence complete")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class LiftAndVerify(py_trees.behaviour.Behaviour):
    """Lift the grasped object and verify it's still held securely"""

    def __init__(self, name, lift_positions, timeout=2.0, force_threshold=-5.0):
        super(LiftAndVerify, self).__init__(name)
        self.lift_positions = lift_positions  # Dictionary of joint positions for lifting
        self.timeout = timeout
        self.force_threshold = force_threshold  # Threshold to determine if object is still grasped
        self.start_time = None
        self.movement_started = False

    def initialise(self):
        self.start_time = robot.getTime()
        self.movement_started = False
        print("Starting lift sequence")

        # Stop the robot's wheels
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)

    def update(self):
        current_time = robot.getTime()

        # First check if gripper still has the object
        left_force = motors['gripper_left_finger_joint'].getForceFeedback()
        right_force = motors['gripper_right_finger_joint'].getForceFeedback()

        # If force feedback indicates object was dropped
        if (abs(left_force) < abs(self.force_threshold) and
                abs(right_force) < abs(self.force_threshold)):
            print(f"Object may have been dropped! Forces: L={left_force:.2f}, R={right_force:.2f}")
            # Update blackboard to indicate grasp failure
            blackboard = py_trees.blackboard.Blackboard()
            blackboard.set("grasp_success", False)
            return py_trees.common.Status.FAILURE

        # Start the lift movement if not started yet
        if not self.movement_started:
            # Apply all joint positions from the lift_position dictionary
            for joint_name, position in self.lift_positions.items():
                if joint_name in motors:
                    motors[joint_name].setPosition(position)

            # Ensure gripper maintains its grasp
            # Don't change gripper position as it should maintain the grasp
            self.movement_started = True
            print("Arm moving to lift position")

        # Check for timeout - consider motion complete after timeout
        if current_time - self.start_time > self.timeout:
            print("Lift sequence completed")

            # Final force check to confirm object is still held
            if (abs(left_force) >= abs(self.force_threshold) or
                    abs(right_force) >= abs(self.force_threshold)):
                print(f"Object securely held with forces: L={left_force:.2f}, R={right_force:.2f}")
                return py_trees.common.Status.SUCCESS
            else:
                print("Object lost during final lift position")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


# Backup after grasp
class BackupAfterGrasp(py_trees.behaviour.Behaviour):
    def __init__(self, name, backup_distance=0.12, duration=3.0):
        super(BackupAfterGrasp, self).__init__(name)
        self.backup_distance = backup_distance
        self.duration = duration
        self.start_time = None
        self.start_position = None
        self.state = "INIT"

    def initialise(self):
        self.start_time = robot.getTime()
        self.start_position = gps.getValues()[:2]
        self.state = "INIT"
        print("Starting controlled backup after grasp")

    def update(self):
        current_time = robot.getTime()
        current_position = gps.getValues()[:2]
        dx = current_position[0] - self.start_position[0]
        dy = current_position[1] - self.start_position[1]
        distance_moved = np.sqrt(dx * dx + dy * dy)

        if self.state == "INIT":
            if current_time - self.start_time > 1.0:
                torso_height = sensors['torso_lift_joint'].getValue()
                if torso_height < 0.25:
                    motors['torso_lift_joint'].setPosition(0.30)
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    return py_trees.common.Status.RUNNING
                else:
                    self.state = "BACKUP"
                    print("Starting backup movement")
            else:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                return py_trees.common.Status.RUNNING

        elif self.state == "BACKUP":
            if distance_moved >= self.backup_distance or current_time - self.start_time > self.duration:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                print(f"Backup complete, moved {distance_moved:.3f}m")
                return py_trees.common.Status.SUCCESS
            leftMotor.setVelocity(-1.5)
            rightMotor.setVelocity(-1.5)

        return py_trees.common.Status.RUNNING


# --- Decorators and Composites ---
class CheckHardwareStatus(py_trees.behaviour.Behaviour):
    """Check if all hardware components are functioning properly"""

    def __init__(self, name):
        super(CheckHardwareStatus, self).__init__(name)
        self.required_devices = {
            'motors': ['torso_lift_joint', 'arm_1_joint', 'gripper_left_finger_joint'],
            'sensors': ['torso_lift_joint', 'arm_1_joint', 'gripper_left_finger_joint'],
            'other': ['camera', 'gps', 'compass']
        }

    def update(self):
        # Check motors
        for motor_name in self.required_devices['motors']:
            if motor_name not in motors:
                print(f"ERROR: Required motor '{motor_name}' not found")
                return py_trees.common.Status.FAILURE

        # Check sensors
        for sensor_name in self.required_devices['sensors']:
            if sensor_name not in sensors:
                print(f"ERROR: Required sensor '{sensor_name}' not found")
                return py_trees.common.Status.FAILURE

        # Check other devices
        if not camera:
            print("ERROR: Camera not found")
            return py_trees.common.Status.FAILURE

        if not gps:
            print("ERROR: GPS not found")
            return py_trees.common.Status.FAILURE

        if not compass:
            print("ERROR: Compass not found")
            return py_trees.common.Status.FAILURE

        print("All hardware components verified")
        return py_trees.common.Status.SUCCESS


# ------------------------------------------------------------------------------
# BEHAVIOR TREE CONSTRUCTION
# ------------------------------------------------------------------------------
def create_behavior_tree():
    # Initialize y_offsets with defaults if not provided
    root = py_trees.composites.Sequence(name="Root", memory=True)

    # Initialize and check hardware
    initialization = py_trees.composites.Sequence(name="Initialization", memory=True)
    check_hardware = CheckHardwareStatus("Check Hardware Status")
    move_to_safe_position = MoveToPosition("Move to Safe Position", starting_position)
    initialization.add_children([check_hardware, move_to_safe_position])

    # Default Y offsets for each jar
    y_offsets = [0.13, -0.85, -0.60]

    # Add initialization to root
    root.add_children([initialization])

    # Main task sequence with fallbacks
    for i in range(3):  # Handle 3 jars
        jar_sequence = py_trees.composites.Sequence(name=f"Handle Jar {i + 1}", memory=True)

        # Use same approach for all jars
        find_object = py_trees.composites.Selector(name="Find Object", memory=True)

        # First try direct recognition (exactly like jar 1)
        recognize = EnhancedObjectRecognizer(f"Recognize Object {i + 1}", timeout=3.0)

        # If direct recognition fails, try incremental scanning
        comprehensive_scanner = ComprehensiveScanner("Comprehensive Scanner", total_angles=8, angle_increment=45)

        # Add both direct recognition and scan-based recognition to the selector
        find_object.add_children([recognize, comprehensive_scanner])

        # Direct approach sequence without recovery wrapper
        approach_sequence = py_trees.composites.Sequence(name="Approach Sequence", memory=True)

        # Prepare arm for approach
        prepare_arm = MoveToPosition(f"Prepare Arm for Approach {i + 1}", lift_position)

        # Create arm movement behavior with Y offset only for each jar
        move_arm_behavior = MoveArmIK(
            f"Move Arm {i + 1}",
            offset_x=0.0,  # Use fixed zero X offset
            offset_y=y_offsets[i]  # Use custom Y offset for this jar
        )

        move_to_object = MoveToObject(
            f"Move to Object {i + 1}",
            None,  # Use blackboard for target position
            gps,
            compass,
            move_arm_behavior,
            camera
        )

        # Add behaviors to approach sequence
        approach_sequence.add_children([prepare_arm, move_to_object])

        # Simple direct grasp without retry mechanism
        grasp_behavior = GraspController(
            f"Grasp Object {i + 1}",
            force_threshold=-10.0
        )

        # Enhanced transport and place sequence
        transport_and_place = py_trees.composites.Sequence(name="Transport and Place", memory=True)

        lift_object = LiftAndVerify(f"Lift and Verify {i + 1}", lift_position)

        backup = BackupAfterGrasp(f"Backup After Grasp {i + 1}")

        move_to_table_bt = MoveToWaypoint(f"Move to Table Waypoint {i + 1}", table_waypoints)
        place_object_bt = MoveToPosition(f"Place Object {i + 1}", place_position, timeout=8.0)
        open_gripper_bt = OpenGripper(f"Open Gripper {i + 1}")
        reset_for_home = MoveToPosition(f"Reset Arm For Home {i + 1}", starting_position, timeout=4.0)
        move_to_home_bt = MoveToWaypoint(f"Move to Home Waypoint {i + 1}", home_waypoint)

        # Combine the transport
        transport_and_place.add_children([
            py_trees.behaviours.Success(name=f"StartTransport_{i + 1}"),
            lift_object,
            backup,
            move_to_table_bt,
            place_object_bt,
            open_gripper_bt,
            reset_for_home,
            move_to_home_bt
        ])

        # Add all tasks to jar_sequence
        jar_sequence.add_children([
            find_object,
            approach_sequence,
            grasp_behavior,
            transport_and_place
        ])

        # Add jar_sequence to root
        root.add_child(jar_sequence)

    # Create the behavior tree
    behavior_tree = py_trees.trees.BehaviourTree(root)
    behavior_tree.visitors.append(py_trees.visitors.DebugVisitor())
    return behavior_tree


class RuntimeMonitor:
    """Monitors and logs robot performance using GPS and sensors"""

    def __init__(self, log_interval=10.0):
        self.start_time = robot.getTime()
        self.last_log_time = self.start_time
        self.log_interval = log_interval
        self.joint_position_history = {}
        self.motor_velocity_history = {}

        # Add GPS position tracking
        self.position_history = []
        self.max_position_history = 100
        self.last_position = None
        if gps:
            self.last_position = gps.getValues()

    def update(self):
        current_time = robot.getTime()

        # Track joint positions
        for joint_name, sensor in sensors.items():
            if joint_name not in self.joint_position_history:
                self.joint_position_history[joint_name] = []
            self.joint_position_history[joint_name].append(sensor.getValue())
            # Limit history length
            if len(self.joint_position_history[joint_name]) > 100:
                self.joint_position_history[joint_name].pop(0)

        # Track GPS position
        if gps:
            current_pos = gps.getValues()
            self.position_history.append(current_pos)
            if len(self.position_history) > self.max_position_history:
                self.position_history.pop(0)

            # Calculate movement speed
            if self.last_position:
                dx = current_pos[0] - self.last_position[0]
                dy = current_pos[1] - self.last_position[1]
                distance = np.sqrt(dx * dx + dy * dy)
                dt = timestep / 1000.0
                speed = distance / dt if dt > 0 else 0
                self.last_position = current_pos

        # Log periodically
        if current_time - self.last_log_time >= self.log_interval:
            self.log_robot_status()
            self.last_log_time = current_time

    def log_robot_status(self):
        """Log important robot metrics with enhanced GPS and compass data"""
        # Robot position and orientation
        if gps and compass:
            pos = gps.getValues()
            compass_vals = compass.getValues()
            heading = np.degrees(np.arctan2(compass_vals[0], compass_vals[1]))

            print(f"GPS Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            print(f"Heading: {heading:.2f}° ({compass_vals[0]:.2f}, {compass_vals[1]:.2f})")

            # Calculate path stats if we have enough history
            if len(self.position_history) > 2:
                # Calculate total distance traveled
                total_distance = 0
                for i in range(1, len(self.position_history)):
                    p1 = self.position_history[i - 1]
                    p2 = self.position_history[i]
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    total_distance += np.sqrt(dx * dx + dy * dy)

                # Calculate displacement (straight-line distance from start)
                start = self.position_history[0]
                current = self.position_history[-1]
                dx = current[0] - start[0]
                dy = current[1] - start[1]
                displacement = np.sqrt(dx * dx + dy * dy)

                print(f"Total distance traveled: {total_distance:.2f}m")
                print(f"Displacement from start: {displacement:.2f}m")
                if total_distance > 0:
                    print(f"Path efficiency: {(displacement / total_distance) * 100:.1f}%")

        # Battery status (if available)
        battery_level = robot.getBatteryValue() if hasattr(robot, "getBatteryValue") else "N/A"
        print(f"Battery level: {battery_level}")

        # Check for any excessive joint velocities or oscillations
        for joint_name, history in self.joint_position_history.items():
            if len(history) > 20:
                # Check for oscillations by looking at position changes
                changes = [abs(history[i] - history[i - 1]) for i in range(1, len(history))]
                if sum(changes) > 0.5 and max(changes) < 0.1:
                    print(f"WARNING: Possible oscillation in joint {joint_name}")


# ------------------------------------------------------------------------------
# MAIN EXECUTION LOOP
# ------------------------------------------------------------------------------
def main():
    """Main execution function"""
    # Verify GPS and compass are working
    if gps and compass:
        pos = gps.getValues()
        compass_vals = compass.getValues()
        heading = np.degrees(np.arctan2(compass_vals[0], compass_vals[1]))
        print(f"Initial GPS Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        print(f"Initial Heading: {heading:.2f}°")
    else:
        print("WARNING: GPS or compass unavailable!")

    # Create runtime monitor
    monitor = RuntimeMonitor(log_interval=10.0)

    # Use the improved behavior tree with custom Y offsets
    behavior_tree = create_behavior_tree()
    behavior_tree.setup(timeout=15)

    # Initialize the tree with a first tick to get proper status
    behavior_tree.tick()

    last_print_time = robot.getTime()
    print_interval = 10.0  # Seconds between status prints
    tick_interval = 1  # Ticks per timestep

    while robot.step(timestep) != -1:
        if robot.getBasicTimeStep() % tick_interval == 0:
            # Update runtime monitor
            monitor.update()

            # Tick the behavior tree
            behavior_tree.tick()
            current_time = robot.getTime()

            # Get the status from the root node (not the tick() return value)
            status = behavior_tree.root.status

            if current_time - last_print_time >= print_interval:
                print("\nBehavior Tree Status:")
                print(py_trees.display.ascii_tree(behavior_tree.root))
                last_print_time = current_time

            if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                print(
                    f"Behavior tree {'completed successfully' if status == py_trees.common.Status.SUCCESS else 'failed'}")
                break

        robot.step(timestep)


# Run the main function
if __name__ == "__main__":
    main()
