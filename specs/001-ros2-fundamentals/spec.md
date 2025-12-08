# Feature Specification: Module 1 — The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-ros2-fundamentals`
**Created**: 2025-12-08
**Status**: Draft
**Input**: User description: "Module 1 — The Robotic Nervous System (ROS 2)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Learn ROS 2 Communication Basics (Priority: P1)

A robotics student wants to understand how ROS 2 enables communication between different parts of a robot system. They need to learn about nodes (independent processes), topics (publish/subscribe channels), and services (request/response patterns) to build foundational knowledge for humanoid robot development.

**Why this priority**: This is the foundational knowledge required for all subsequent learning. Without understanding ROS 2 communication primitives, students cannot progress to control or robot description.

**Independent Test**: Student can create a simple ROS 2 workspace with two nodes that exchange messages via a topic, and can verify message flow using CLI tools (`ros2 topic list`, `ros2 topic echo`).

**Acceptance Scenarios**:

1. **Given** a student has ROS 2 Humble installed, **When** they follow Chapter 1 exercises, **Then** they can create and run a publisher node that sends string messages to a topic.
2. **Given** a running publisher node, **When** the student creates a subscriber node following the examples, **Then** they can receive and display messages from the publisher.
3. **Given** understanding of topics, **When** the student implements a service example, **Then** they can create a service server and client that exchange requests/responses.

---

### User Story 2 - Control Humanoid Actions with Python (Priority: P2)

A developer wants to use Python (rclpy) to create AI agents that send commands to humanoid robot controllers. They need to understand how to structure ROS 2 Python packages, create custom message types for humanoid actions, and implement control nodes that bridge AI decision-making to robot execution.

**Why this priority**: Building on communication basics, this enables students to write practical control code that can interface with AI systems—a key requirement for the textbook's focus on bridging AI agents to robot controllers.

**Independent Test**: Student can create a Python package with a control node that publishes joint commands to a humanoid robot topic, with the node structured to accept commands from an external AI agent interface.

**Acceptance Scenarios**:

1. **Given** a student understands ROS 2 basics, **When** they follow Chapter 2 exercises, **Then** they can create a properly structured ROS 2 Python package using `ros2 pkg create`.
2. **Given** a Python package, **When** the student implements a control node, **Then** they can publish joint position commands to a `/joint_commands` topic using rclpy.
3. **Given** a control node, **When** the student adds a simple command interface, **Then** they can send high-level commands (e.g., "wave", "bow") that translate to joint trajectories.

---

### User Story 3 - Model Humanoid Robot Structure with URDF (Priority: P3)

A robotics student wants to describe a humanoid robot's physical structure using URDF (Unified Robot Description Format). They need to understand links (rigid bodies), joints (connections between links), and how to create a complete robot model that can be visualized and used in simulation.

**Why this priority**: URDF is essential for any robot work in ROS 2, and humanoid robots require complex multi-joint structures. This chapter completes the foundational module by enabling students to define their own robot models.

**Independent Test**: Student can create a valid URDF file for a simplified humanoid (torso, head, two arms with shoulder/elbow joints) and visualize it using standard ROS 2 tools.

**Acceptance Scenarios**:

1. **Given** a student understands ROS 2 communication, **When** they follow Chapter 3 exercises, **Then** they can create a URDF file with multiple links and joints that passes validation (`check_urdf`).
2. **Given** a basic URDF, **When** the student adds visual and collision elements, **Then** they can visualize the robot model in RViz2 using the robot_state_publisher.
3. **Given** a humanoid URDF, **When** the student defines joint limits and types, **Then** the model accurately represents a simplified humanoid with proper joint constraints.

---

### Edge Cases

- What happens when a student's ROS 2 installation has missing dependencies? Provide troubleshooting section with common installation issues.
- How does the system handle invalid URDF syntax? Include validation commands and common error explanations.
- What if joint commands exceed defined limits? Demonstrate joint limit enforcement in URDF and control code.
- How should students debug silent node failures? Include logging best practices and diagnostic CLI commands.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Module MUST explain ROS 2 nodes as independent computational processes with clear lifecycle concepts.
- **FR-002**: Module MUST demonstrate publish/subscribe communication using topics with working Python examples.
- **FR-003**: Module MUST explain request/response patterns using services with client/server examples.
- **FR-004**: Module MUST provide rclpy code examples that compile and run on ROS 2 Humble or later.
- **FR-005**: Module MUST teach Python package structure including `setup.py`, `package.xml`, and entry points.
- **FR-006**: Module MUST explain URDF elements: `<robot>`, `<link>`, `<joint>`, `<visual>`, `<collision>`, `<inertial>`.
- **FR-007**: Module MUST provide a complete, valid URDF for a simplified humanoid robot (minimum: torso, head, 2 arms with 2 joints each).
- **FR-008**: Module MUST include commands to validate URDF files (`check_urdf`) and visualize in RViz2.
- **FR-009**: All code examples MUST include complete imports, not rely on implicit state, and run without modification.
- **FR-010**: Module MUST align with official ROS 2 documentation and use no fictional or deprecated APIs.

### Key Entities

- **Node**: An independent ROS 2 process that performs computation. Key attributes: name, namespace, parameters, callbacks.
- **Topic**: A named bus for publish/subscribe communication. Key attributes: name, message type, QoS settings.
- **Service**: A synchronous request/response communication channel. Key attributes: name, service type, server callback.
- **Message**: A structured data type for communication. Key attributes: fields, types, nested structures.
- **Link**: A rigid body in URDF. Key attributes: name, visual geometry, collision geometry, inertial properties.
- **Joint**: A connection between two links. Key attributes: name, type (revolute, prismatic, fixed), parent, child, axis, limits.

### Assumptions

- Students have ROS 2 Humble (or later LTS) installed on Ubuntu 22.04 or compatible environment.
- Students have basic Python programming knowledge (variables, functions, classes, imports).
- Students have access to a terminal and can run command-line tools.
- Visual examples use RViz2 for 3D visualization (comes with standard ROS 2 desktop install).
- Code examples target Python 3.10+ as shipped with ROS 2 Humble.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can create a ROS 2 workspace with a working publisher/subscriber pair within 30 minutes of starting Chapter 1.
- **SC-002**: 100% of Python code examples in the module execute without errors on ROS 2 Humble.
- **SC-003**: Students can create a custom ROS 2 Python package from scratch following Chapter 2 instructions.
- **SC-004**: The provided humanoid URDF passes `check_urdf` validation with zero errors.
- **SC-005**: Students can visualize the humanoid URDF in RViz2 and see all links/joints correctly rendered.
- **SC-006**: Students completing all three chapters can demonstrate a minimal humanoid control pipeline: Python node sending joint commands for a robot described in URDF.
- **SC-007**: All technical content matches current official ROS 2 documentation (no deprecated or fictional APIs).
- **SC-008**: Each chapter includes at least 3 hands-on exercises with clear expected outcomes.

## Scope Boundaries

### In Scope

- ROS 2 communication fundamentals (nodes, topics, services)
- Python-based node development with rclpy
- URDF file creation and validation for humanoid robots
- Basic RViz2 visualization of robot models
- CLI tools for debugging (`ros2 topic`, `ros2 service`, `ros2 node`)

### Out of Scope

- Simulation environments (covered in Module 2: Gazebo/Unity)
- Hardware deployment and real robot integration
- Advanced ROS 2 tools: Nav2, TF2, Lifecycle nodes, Actions
- C++ node development (Python only for this module)
- Custom message generation beyond basic examples
- ROS 2 networking and multi-machine setups

## Dependencies

- **ROS 2 Humble**: Required runtime environment (must be installed before starting)
- **Python 3.10+**: Required for rclpy examples
- **Module 2**: This module is a prerequisite for Simulation Foundations (Gazebo/Unity integration)
