# Chapter 2 Outline: rclpy Control

**File**: `book/docs/module-01-ros2/chapter-02-rclpy-control.mdx`
**Estimated Length**: ~15-20 pages
**Estimated Time**: 2-3 hours

## Learning Objectives

By the end of this chapter, students will be able to:

1. Structure a ROS 2 Python package correctly with all required files
2. Implement a control node that publishes joint commands
3. Create an interface between AI agents and robot controllers
4. Handle node lifecycle, parameters, and clean shutdown

## Prerequisites

- Chapter 1 completed (nodes, topics, services)
- Understanding of Python classes and modules
- Familiarity with command-line package management

## Section Outline

### 1. ROS 2 Python Package Structure (h2)
- Why packages? (organization, reusability, dependency management)
- Creating packages with `ros2 pkg create`
- Package directory layout
- ament_python vs ament_cmake

**Diagram**: Package directory structure visualization

#### 1.1 The package.xml File (h3)
- Package metadata (name, version, description)
- Build type declaration
- Dependencies: `<depend>`, `<exec_depend>`, `<build_depend>`
- License and maintainer info

**Code Example**: Complete `package.xml`
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_basics</name>
  <version>0.1.0</version>
  <description>ROS 2 basics for humanoid robot control</description>
  <maintainer email="student@example.com">Student Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 1.2 The setup.py File (h3)
- Python package setup
- Entry points for executables
- Data files (resource marker, package.xml)

**Code Example**: Complete `setup.py`
```python
from setuptools import find_packages, setup

package_name = 'humanoid_basics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Name',
    maintainer_email='student@example.com',
    description='ROS 2 basics for humanoid robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_commander = humanoid_basics.joint_commander:main',
            'action_interface = humanoid_basics.action_interface:main',
        ],
    },
)
```

### 2. Building Control Nodes (h2)
- What is a control node?
- Control loop basics (sense → plan → act)
- Timer-based publishing
- Callback-driven architecture

**Diagram**: Control node architecture (inputs, processing, outputs)

### 3. Publishing Joint Commands (h2)
- Joint command message types
- sensor_msgs/JointState structure
- Publishing at fixed rates
- Mapping high-level commands to joint positions

**Code Example**: `joint_commander.py`
```python
#!/usr/bin/env python3
"""Joint command publisher for humanoid robot control."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')

        # Define joint names for a simple humanoid arm
        self.joint_names = [
            'right_shoulder_pitch',
            'right_shoulder_roll',
            'right_elbow',
        ]

        # Publisher for joint commands
        self.publisher_ = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )

        # Timer for publishing at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_command)
        self.time = 0.0

        self.get_logger().info('Joint Commander initialized')

    def publish_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Generate smooth sinusoidal motion
        msg.position = [
            math.sin(self.time) * 0.5,      # shoulder pitch
            math.sin(self.time * 0.5) * 0.3, # shoulder roll
            math.sin(self.time * 2) * 0.4,   # elbow
        ]

        self.publisher_.publish(msg)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Bridging AI Agents to ROS 2 (h2)
- The AI-robot interface problem
- Design patterns for agent integration
- High-level commands vs low-level control
- Action abstraction layer

**Diagram**: AI Agent → Action Interface → Joint Commands → Robot

**Code Example**: `action_interface.py`
```python
#!/usr/bin/env python3
"""High-level action interface for AI agent commands."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class ActionInterface(Node):
    def __init__(self):
        super().__init__('action_interface')

        # Subscribe to high-level commands from AI agent
        self.command_sub = self.create_subscription(
            String,
            'agent_commands',
            self.command_callback,
            10
        )

        # Publish joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )

        # Predefined actions (joint positions)
        self.actions = {
            'wave': [0.5, 0.3, -0.8],
            'point': [0.7, 0.0, 0.0],
            'rest': [0.0, 0.0, 0.0],
        }

        self.joint_names = [
            'right_shoulder_pitch',
            'right_shoulder_roll',
            'right_elbow',
        ]

        self.get_logger().info('Action Interface ready')

    def command_callback(self, msg):
        command = msg.data.lower()

        if command in self.actions:
            self.execute_action(command)
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def execute_action(self, action_name):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.actions[action_name]

        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f'Executing action: {action_name}')

def main(args=None):
    rclpy.init(args=args)
    node = ActionInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Node Lifecycle and Shutdown (h2)
- Node states (unconfigured, inactive, active, finalized)
- Clean shutdown with signal handling
- Resource cleanup in destructors
- Why proper shutdown matters

**Code Example**: Proper shutdown handling

### 6. Node Parameters (h2)
- Declaring parameters
- Reading parameters at runtime
- Dynamic parameter updates
- Parameter files (YAML)

### 7. Summary (h2)
- Key concepts recap
- Package structure checklist
- Preview of Chapter 3 (URDF)

## Exercises

### Exercise 1: Create a Humanoid Control Package
**Objective**: Set up a complete ROS 2 Python package
**Steps**:
1. Use `ros2 pkg create --build-type ament_python humanoid_basics`
2. Add dependencies to package.xml
3. Configure entry points in setup.py
4. Build with `colcon build`
**Expected Outcome**: Package builds and installs successfully

### Exercise 2: Implement Joint Command Publisher
**Objective**: Create a node that publishes joint positions
**Steps**:
1. Create `joint_commander.py`
2. Publish JointState messages at 10 Hz
3. Verify with `ros2 topic echo /joint_commands`
**Expected Outcome**: See joint positions updating in terminal

### Exercise 3: Add High-Level Action Interface
**Objective**: Create an action abstraction layer
**Steps**:
1. Create `action_interface.py`
2. Define 3 actions (wave, point, rest)
3. Test with `ros2 topic pub /agent_commands std_msgs/String "data: wave"`
**Expected Outcome**: Action translates to joint commands

### Exercise 4: Add Parameter Support
**Objective**: Make publishing rate configurable
**Steps**:
1. Declare `publish_rate` parameter (default: 10.0)
2. Use parameter in timer creation
3. Test with different rates via command line
**Expected Outcome**: Node respects parameter value

## Assessment Questions

1. What files are required in a ROS 2 Python package?
2. How do you add a new executable to a Python package?
3. Why use an action interface instead of publishing joint commands directly?
4. What happens if you don't call `rclpy.shutdown()`?

## Citations

- [1] ROS 2 Python Package Tutorial: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- [2] sensor_msgs/JointState: https://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/JointState.html
- [3] rclpy API Reference: https://docs.ros2.org/humble/api/rclpy/
