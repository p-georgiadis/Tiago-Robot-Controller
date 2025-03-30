# TIAGo Robot Pick-and-Place Solution Report

## Solution Overview
This solution implements a behavior tree-controlled system for the TIAGo robot to autonomously locate, collect, and transport three jars from a kitchen counter to a designated table. The system combines computer vision, inverse kinematics, and reactive navigation to achieve reliable object manipulation.

## System Architecture

### Key Components
1. **Behavior Tree Framework**: Structured task execution with py_trees
2. **Computer Vision System**: Object recognition with position estimation
3. **Inverse Kinematics Engine**: Arm positioning with ikpy
4. **Waypoint Navigation**: GPS and compass-based movement
5. **Force-based Grasping**: Gripper control with force feedback

### Architectural Diagram
```plaintext
[Root Behavior Tree]
├── [Initialization]
│    ├── Check Hardware Status
│    └── Move to Safe Position
├── [Handle Jar 1]
│    ├── [Find Object]
│    │    ├── Recognize Object 1
│    │    └── Comprehensive Scanner
│    ├── [Approach Sequence]
│    │    ├── Prepare Arm for Approach
│    │    └── Move to Object 1
│    ├── Grasp Object 1
│    └── [Transport and Place]
│         ├── Lift and Verify
│         ├── Backup After Grasp
│         ├── Move to Table Waypoint
│         ├── Place Object
│         ├── Open Gripper
│         ├── Reset Arm For Home
│         └── Move to Home Waypoint
└── [Handle Jar 2/3]
     └── ... (repeats pattern)
```

## Key Implementation Details

### 1. Sensor Fusion System
- **Camera-to-World Coordinate Transformation**
  - Combines GPS, compass, and camera data for accurate object positioning
  - Applies height correction based on torso position
  - Handles perspective transformation
  
```python
def camera_to_world_coordinates(camera_position, camera_offset=0.0):
    """Converts camera-relative coordinates to global world coordinates"""
    # Get robot's current position and orientation
    robot_pos = gps.getValues()
    compass_val = compass.getValues()
    robot_angle = np.arctan2(compass_val[0], compass_val[1])
    
    # Calculate precise camera height with torso lift
    camera_height = robot_pos[2] + 0.891 + torso_height
    
    # Apply height correction based on empirical observations
    z_correction = 0
    if torso_height > reference_torso_height:
        height_diff = torso_height - reference_torso_height
        z_correction = -1.87 * height_diff
```

### 2. Navigation System
- **Waypoint-Based Navigation**
  - Sequential movement through predefined waypoints
  - Dynamic speed control based on target proximity
  - State-based approach (Orienting → Approaching → Final Approach)
  
```python
class MoveToObject(py_trees.behaviour.Behaviour):
    """Navigation behavior that moves robot to target object"""
    # Main states
    # ORIENTING: Rotational alignment
    # APPROACHING: Main movement phase
    # ADJUSTING_ARM: Coordinated arm movement
    # FINAL_APPROACH: Precision positioning
```

### 3. Object Manipulation
- **Multi-stage Grasping Pipeline**
  1. **Object Detection**: Camera recognition with position averaging
  2. **Pre-grasp Positioning**: Arm placement with approach vector calculation
  3. **Force-based Grasping**: Continual force monitoring during grasp
  4. **Transport Verification**: Ensuring object remains secure during movement

```python
class GraspController(py_trees.behaviour.Behaviour):
    """Robotic grasping controller with force feedback verification"""
    # States:
    # APPROACHING: Slowly close gripper
    # VERIFYING: Maintain grip with force validation
```

## Performance Analysis

### Recognition Capabilities
- Successfully detected multiple object types:
  - Jam jars (positions at [1.709, -0.299, 0.893] and [1.961, 0.516, 0.897])
  - Honey jar (position at [1.745, 0.734, 0.877])
- Used comprehensive scanning when direct recognition failed
- Handled different orientations and positions accurately

### Navigation Performance
- Effective waypoint sequencing through predefined paths
- Average path efficiency: ~85-90% (displacement/total distance)
- Successful navigation to multiple targets with approach angle calculation

### Grasping Performance
- Consistent grasp success rate across all objects
- Maintained grip forces between -11N and -16N
- Successfully verified object retention during transport
- Example force readings:
  ```
  Grasp Object 1: Grasp successful! L=-16.00, R=-16.00
  Object securely held with forces: L=-11.13, R=-16.00
  ```

## Implementation Highlights

### 1. Comprehensive Scanner
- 360° visual scanning for difficult-to-detect objects
- Camera orientation optimization with head tilt adjustment
- Multi-position object recognition with timeout protection
```python
class ComprehensiveScanner(py_trees.behaviour.Behaviour):
    """Systematic environment scanning for object discovery"""
    # 8 distinct angles, 45° between positions
    # Includes head positioning for optimal visibility
```

### 2. Inverse Kinematics System
- Custom 15-link kinematic chain with proper active link masking
- Joint limit enforcement with environment-aware approach vectors
- Position validation with real-time sensor feedback
```python
ik_chain = Chain(
    links=chain.links,
    active_links_mask=active_links_mask,
    name="tiago_arm"
)
```

### 3. Behavior Tree Architecture
- Memory-enabled sequences for task continuation
- Fallback mechanisms for recognition (direct → comprehensive)
- Modular components for reusable behaviors across multiple objects

## Observations and Analysis

### System Warnings
```
WARNING: Possible oscillation in joint arm_1_joint
WARNING: Possible oscillation in joint wheel_left_joint
```
These warnings indicate minor control instabilities in certain joints, likely due to rapid direction changes or obstacle proximity. The system was able to continue operation despite these oscillations.

### Navigation Metrics
- Total distances traveled: 0.60m-1.12m per task sequence
- Displacement efficiency: 77.4%-100% (higher is better)
- Average navigation completion time: ~15-20 seconds per waypoint sequence

## Conclusion

The implemented solution successfully demonstrates autonomous pick-and-place capabilities using the TIAGo robot platform. The behavior tree architecture provides a robust framework for complex task sequencing, while the inverse kinematics and force-feedback systems enable reliable object manipulation.

Key strengths of the implementation include:
1. Robust object recognition with fallback mechanisms
2. Precise arm positioning using inverse kinematics
3. Force-verified grasping with continuous monitoring
4. Efficient waypoint navigation with state-based control

The system successfully completed the full task of retrieving three different jars and placing them at the designated table location, demonstrating the effectiveness of the behavior tree approach to robotic task planning and execution.
