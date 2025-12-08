# Data Model: Module 1 — ROS 2 Fundamentals

**Date**: 2025-12-08
**Branch**: `001-ros2-fundamentals`
**Purpose**: Define content entities and their relationships for the textbook module

## Content Entities

### Module

The top-level organizational unit containing related chapters.

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Unique identifier | `module-01-ros2` |
| number | integer | Sequential module number | `1` |
| title | string | Display title | `ROS 2 Fundamentals` |
| subtitle | string | Descriptive subtitle | `The Robotic Nervous System` |
| position | integer | Sidebar order | `1` |
| prerequisites | string[] | Required prior knowledge | `["Python basics"]` |
| chapters | Chapter[] | Child chapters | See below |

### Chapter

An individual teaching unit within a module.

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Unique identifier | `chapter-01-basics` |
| number | integer | Chapter number within module | `1` |
| title | string | Display title | `ROS 2 Basics: Nodes, Topics, Services` |
| slug | string | URL-friendly name | `basics` |
| learning_objectives | string[] | What student will learn | See below |
| prerequisites | string[] | Required prior chapters | `[]` |
| estimated_time | string | Time to complete | `"2-3 hours"` |
| sections | Section[] | Content sections | See below |
| exercises | Exercise[] | Hands-on activities | See below |
| assessment | Assessment | End-of-chapter quiz | See below |

### Section

A content block within a chapter.

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Unique identifier | `what-is-a-node` |
| heading | string | Section title | `What is a Node?` |
| depth | integer | Heading level (2 or 3) | `2` |
| content | string | Markdown prose | `"A node is..."` |
| code_examples | CodeExample[] | Associated code | See below |
| diagrams | Diagram[] | Visual aids | See below |
| citations | Citation[] | References | See below |

### CodeExample

A runnable code snippet with context.

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Unique identifier | `publisher-node-example` |
| title | string | Display title | `Simple Publisher Node` |
| language | string | Programming language | `python` |
| filename | string | Source file path | `publisher_node.py` |
| code | string | Complete source code | See examples |
| run_instructions | string | How to execute | `ros2 run ...` |
| expected_output | string | What student should see | `"Publishing: wave"` |
| dependencies | string[] | Required packages | `["rclpy", "std_msgs"]` |

### Diagram

A visual representation of concepts.

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Unique identifier | `ros2-node-communication` |
| title | string | Alt text / caption | `ROS 2 Node Communication` |
| type | enum | `mermaid`, `svg`, `png` | `mermaid` |
| source | string | Diagram content or path | See examples |
| width | string | Display width | `"100%"` |

### Exercise

A hands-on activity for students.

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Unique identifier | `exercise-1-create-workspace` |
| number | integer | Exercise number | `1` |
| title | string | Exercise title | `Create a ROS 2 Workspace` |
| objective | string | What student will accomplish | `"Set up a colcon workspace"` |
| steps | string[] | Numbered instructions | See examples |
| expected_outcome | string | Success criteria | `"Workspace builds without errors"` |
| hints | string[] | Optional help | `["Check your sourcing"]` |
| solution | string | Reference solution (hidden) | See examples |

### Citation

A reference to external documentation.

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | string | Reference number | `1` |
| type | enum | `docs`, `paper`, `book`, `web` | `docs` |
| title | string | Source title | `ROS 2 Documentation` |
| url | string | Direct link | `https://docs.ros.org/en/humble/` |
| accessed | string | Access date | `2025-12-08` |
| authors | string[] | Author names | `["Open Robotics"]` |

## Content Relationships

```text
Module
  └── has many → Chapter
                   ├── has many → Section
                   │                ├── has many → CodeExample
                   │                ├── has many → Diagram
                   │                └── has many → Citation
                   ├── has many → Exercise
                   └── has one → Assessment
```

## Chapter Content Models

### Chapter 1: ROS 2 Basics

```yaml
chapter:
  id: chapter-01-basics
  number: 1
  title: "ROS 2 Basics: Nodes, Topics, Services"
  learning_objectives:
    - "Explain what a ROS 2 node is and why nodes are used"
    - "Create publisher and subscriber nodes using rclpy"
    - "Implement a service server and client for request/response communication"
    - "Use ROS 2 CLI tools to inspect running nodes and topics"
  prerequisites: []
  estimated_time: "2-3 hours"
  sections:
    - id: introduction
      heading: "Introduction to ROS 2"
      depth: 2
    - id: what-is-node
      heading: "What is a Node?"
      depth: 2
    - id: topics-pubsub
      heading: "Topics: Publish/Subscribe Communication"
      depth: 2
    - id: creating-publisher
      heading: "Creating a Publisher Node"
      depth: 3
    - id: creating-subscriber
      heading: "Creating a Subscriber Node"
      depth: 3
    - id: services
      heading: "Services: Request/Response Communication"
      depth: 2
    - id: cli-tools
      heading: "ROS 2 Command-Line Tools"
      depth: 2
  exercises:
    - title: "Create a ROS 2 Workspace"
    - title: "Build Your First Publisher"
    - title: "Connect Publisher and Subscriber"
    - title: "Implement an Add Two Ints Service"
  code_examples:
    - publisher_node.py
    - subscriber_node.py
    - service_server.py
    - service_client.py
```

### Chapter 2: rclpy Control

```yaml
chapter:
  id: chapter-02-rclpy-control
  number: 2
  title: "rclpy Control: Python Agents → Humanoid Actions"
  learning_objectives:
    - "Structure a ROS 2 Python package correctly"
    - "Implement a control node that publishes joint commands"
    - "Create an interface between AI agents and robot controllers"
    - "Handle node lifecycle and clean shutdown"
  prerequisites:
    - chapter-01-basics
  estimated_time: "2-3 hours"
  sections:
    - id: python-packages
      heading: "ROS 2 Python Package Structure"
      depth: 2
    - id: package-xml
      heading: "The package.xml File"
      depth: 3
    - id: setup-py
      heading: "The setup.py File"
      depth: 3
    - id: control-nodes
      heading: "Building Control Nodes"
      depth: 2
    - id: joint-commands
      heading: "Publishing Joint Commands"
      depth: 2
    - id: ai-agent-interface
      heading: "Bridging AI Agents to ROS 2"
      depth: 2
    - id: node-lifecycle
      heading: "Node Lifecycle and Shutdown"
      depth: 2
  exercises:
    - title: "Create a Humanoid Control Package"
    - title: "Implement Joint Command Publisher"
    - title: "Add High-Level Action Interface"
    - title: "Clean Node Shutdown"
  code_examples:
    - joint_commander.py
    - action_interface.py
    - package.xml
    - setup.py
```

### Chapter 3: URDF for Humanoids

```yaml
chapter:
  id: chapter-03-urdf-humanoids
  number: 3
  title: "URDF for Humanoids: Links, Joints, Robot Structure"
  learning_objectives:
    - "Explain the purpose and structure of URDF files"
    - "Define links with visual, collision, and inertial properties"
    - "Create joints connecting links with proper constraints"
    - "Build a complete humanoid robot model"
    - "Visualize URDF models in RViz2"
  prerequisites:
    - chapter-01-basics
    - chapter-02-rclpy-control
  estimated_time: "3-4 hours"
  sections:
    - id: urdf-intro
      heading: "Introduction to URDF"
      depth: 2
    - id: links
      heading: "Defining Links"
      depth: 2
    - id: visual-elements
      heading: "Visual Elements"
      depth: 3
    - id: collision-elements
      heading: "Collision Elements"
      depth: 3
    - id: inertial-elements
      heading: "Inertial Properties"
      depth: 3
    - id: joints
      heading: "Defining Joints"
      depth: 2
    - id: joint-types
      heading: "Joint Types"
      depth: 3
    - id: joint-limits
      heading: "Joint Limits and Dynamics"
      depth: 3
    - id: humanoid-model
      heading: "Building a Humanoid Model"
      depth: 2
    - id: visualization
      heading: "Visualizing in RViz2"
      depth: 2
    - id: validation
      heading: "Validating URDF Files"
      depth: 2
  exercises:
    - title: "Create a Simple Link"
    - title: "Connect Links with Joints"
    - title: "Build a Humanoid Torso and Head"
    - title: "Add Arms with Shoulder and Elbow Joints"
    - title: "Visualize Your Humanoid in RViz2"
  code_examples:
    - simple_link.urdf
    - two_links.urdf
    - humanoid_simple.urdf
    - launch_rviz.py
```

## RAG Extraction Model

### Chunk

The unit of content for vector embedding.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique chunk identifier |
| module_id | string | Parent module |
| chapter_id | string | Parent chapter |
| section_id | string | Parent section (if prose) |
| heading | string | Context heading |
| type | enum | `prose`, `code`, `exercise` |
| content | string | Raw text content |
| language | string | Code language (if code) |
| token_count | integer | Approximate tokens |
| embedding | float[] | Vector embedding |

### Chunk Size Targets

| Type | Min Tokens | Max Tokens | Target |
|------|------------|------------|--------|
| Prose | 100 | 1000 | 500 |
| Code | 50 | 500 | 200 |
| Exercise | 100 | 500 | 300 |
