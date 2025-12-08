# Chapter 3 Outline: URDF for Humanoids

**File**: `book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx`
**Estimated Length**: ~20-25 pages
**Estimated Time**: 3-4 hours

## Learning Objectives

By the end of this chapter, students will be able to:

1. Explain the purpose and structure of URDF files
2. Define links with visual, collision, and inertial properties
3. Create joints connecting links with proper types and constraints
4. Build a complete simplified humanoid robot model
5. Visualize and validate URDF models using RViz2 and check_urdf

## Prerequisites

- Chapter 1 completed (nodes, topics, services)
- Chapter 2 completed (Python packages, joint commands)
- Basic understanding of 3D coordinate systems
- XML syntax familiarity

## Section Outline

### 1. Introduction to URDF (h2)
- What is URDF? (Unified Robot Description Format)
- Why URDF matters in ROS 2
- URDF vs SDF (when to use each)
- The robot as a kinematic tree

**Diagram**: Kinematic tree structure of a humanoid

### 2. Defining Links (h2)
- What is a link? (rigid body)
- The `<link>` element structure
- Link naming conventions
- The base_link convention

#### 2.1 Visual Elements (h3)
- Purpose: what the robot looks like
- Geometry options: box, cylinder, sphere, mesh
- Material and color specification
- Origin/offset from link frame

**Code Example**: Simple link with visual
```xml
<link name="torso">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
    <material name="blue">
      <color rgba="0.2 0.2 0.8 1.0"/>
    </material>
  </visual>
</link>
```

#### 2.2 Collision Elements (h3)
- Purpose: physics simulation boundaries
- Simplified geometry for performance
- When collision differs from visual
- Multiple collision geometries

**Code Example**: Link with collision
```xml
<link name="upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.25"/>
    </geometry>
  </collision>
</link>
```

#### 2.3 Inertial Properties (h3)
- Purpose: mass and inertia for dynamics
- The `<mass>` element
- The `<inertia>` matrix (Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
- Calculating inertia for primitive shapes
- Why accurate inertia matters for control

**Code Example**: Link with inertial
```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.008" ixy="0" ixz="0"
             iyy="0.008" iyz="0"
             izz="0.008"/>
  </inertial>
</link>
```

**Table**: Inertia formulas for primitive shapes

### 3. Defining Joints (h2)
- What is a joint? (connection between links)
- Parent-child relationship
- The kinematic chain concept
- Joint frame and axis

#### 3.1 Joint Types (h3)

| Type | DOF | Description | Use Case |
|------|-----|-------------|----------|
| `fixed` | 0 | No relative motion | Mounting sensors, rigid connections |
| `revolute` | 1 | Rotation around axis with limits | Arms, legs, fingers |
| `continuous` | 1 | Rotation without limits | Wheels |
| `prismatic` | 1 | Linear translation with limits | Telescoping arms |
| `floating` | 6 | Free motion (rarely used) | Mobile base |
| `planar` | 3 | Motion in a plane | Special mechanisms |

**Code Example**: Revolute joint
```xml
<joint name="right_shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="right_upper_arm"/>
  <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
</joint>
```

#### 3.2 Joint Limits and Dynamics (h3)
- Position limits (lower, upper)
- Velocity limits
- Effort (force/torque) limits
- Dynamics: damping and friction

**Code Example**: Joint with full dynamics
```xml
<joint name="right_elbow" type="revolute">
  <parent link="right_upper_arm"/>
  <child link="right_forearm"/>
  <origin xyz="0 0 -0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="30" velocity="1.5"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### 4. Building a Humanoid Model (h2)
- Design approach: start simple, add complexity
- Humanoid structure: torso → head, arms, (legs optional)
- Coordinate frame conventions
- Symmetry and naming

**Diagram**: Humanoid link/joint structure (tree view)

**Code Example**: Complete simplified humanoid URDF
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="neck" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <cylinder radius="0.03" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="30" velocity="1.5"/>
  </joint>

  <!-- Left Arm (mirror of right) -->
  <!-- ... similar structure with negative x offsets ... -->

</robot>
```

### 5. Visualizing in RViz2 (h2)
- The robot_state_publisher node
- The joint_state_publisher_gui node
- RViz2 configuration for robot visualization
- Adding the RobotModel display

**Code Example**: Launch file for visualization
```python
# launch/view_humanoid.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_basics'),
        'urdf',
        'humanoid_simple.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('humanoid_basics'),
                'rviz',
                'view_robot.rviz'
            )]
        ),
    ])
```

**Run Instructions**:
```bash
ros2 launch humanoid_basics view_humanoid.launch.py
```

### 6. Validating URDF Files (h2)
- The `check_urdf` command
- Common URDF errors and fixes
- Visualizing the kinematic tree with `urdf_to_graphviz`
- Validation checklist

**Code Example**: Validation commands
```bash
# Check URDF validity
check_urdf humanoid_simple.urdf

# Generate kinematic tree visualization
urdf_to_graphviz humanoid_simple.urdf
```

**Common Errors Table**:
| Error | Cause | Fix |
|-------|-------|-----|
| "No link elements found" | Missing `<link>` tags | Add at least one link |
| "parent link not found" | Typo in parent name | Check link names match |
| "child link not found" | Typo in child name | Check link names match |
| "Multiple parents" | Link has >1 parent joint | URDF is a tree, not a graph |

### 7. Summary (h2)
- URDF structure recap
- Best practices checklist
- Connection to Module 2 (simulation)

## Exercises

### Exercise 1: Create a Simple Link
**Objective**: Define a single link with all three properties
**Steps**:
1. Create URDF with one box link
2. Add visual, collision, and inertial
3. Validate with `check_urdf`
**Expected Outcome**: Valid URDF, no errors

### Exercise 2: Connect Links with Joints
**Objective**: Create a two-link arm
**Steps**:
1. Define base_link and upper_arm
2. Connect with revolute joint
3. Set appropriate limits
4. Validate and visualize
**Expected Outcome**: Arm visible in RViz2, joint moves

### Exercise 3: Build a Humanoid Torso and Head
**Objective**: Create the upper body structure
**Steps**:
1. Define torso (base_link)
2. Add head with fixed neck joint
3. Add visual materials
4. Visualize in RViz2
**Expected Outcome**: Torso and head visible

### Exercise 4: Add Arms with Shoulder and Elbow Joints
**Objective**: Complete the humanoid with articulated arms
**Steps**:
1. Add right arm (upper_arm, forearm)
2. Add shoulder and elbow joints
3. Mirror for left arm
4. Test joint limits in joint_state_publisher_gui
**Expected Outcome**: Full humanoid with moving arms

### Exercise 5: Visualize Your Humanoid in RViz2
**Objective**: Create a complete visualization setup
**Steps**:
1. Create launch file
2. Configure RViz2 display
3. Save RViz configuration
4. Test full pipeline
**Expected Outcome**: Robot visible and controllable

## Assessment Questions

1. What is the difference between visual and collision geometry?
2. Why is accurate inertia important for robot control?
3. What joint type would you use for a robot wheel?
4. How do you specify the axis of rotation for a revolute joint?
5. What must be true about the parent-child relationships in URDF?

## Citations

- [1] URDF Specification: https://wiki.ros.org/urdf/XML
- [2] robot_state_publisher: https://docs.ros.org/en/humble/p/robot_state_publisher/
- [3] RViz2 User Guide: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide.html
- [4] URDF Tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
