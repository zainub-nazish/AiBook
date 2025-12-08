# Data Model: Module 2 — The Digital Twin

**Feature**: 002-digital-twin-simulation
**Date**: 2025-12-08
**Purpose**: Define content entities, structures, and relationships for simulation chapters.

## Content Entities

### Module

```yaml
Module:
  id: "module-02-digital-twin"
  title: "The Digital Twin (Gazebo & Unity)"
  position: 2
  prerequisites:
    - "module-01-ros2"
  chapters: [Chapter]
  assets_path: "book/docs/module-02-digital-twin/assets/"
```

### Chapter

```yaml
Chapter:
  id: string                    # e.g., "chapter-01-gazebo-physics"
  title: string                 # e.g., "Physics Simulation in Gazebo"
  module_id: string             # Reference to parent module
  position: number              # Order within module
  learning_objectives: [string] # 3-5 objectives per chapter
  prerequisites: [string]       # Specific concepts required
  sections: [Section]
  exercises: [Exercise]
  assessment_questions: [Question]
  estimated_reading_time: string # e.g., "45 minutes"
```

### Section

```yaml
Section:
  id: string                    # e.g., "sdf-world-structure"
  title: string                 # e.g., "SDF World File Structure"
  heading_level: number         # 2 (h2) or 3 (h3)
  content_type: enum            # theory | tutorial | reference
  code_examples: [CodeExample]
  diagrams: [Diagram]
  admonitions: [Admonition]
  rag_chunk_id: string          # For RAG extraction tracking
```

### CodeExample

```yaml
CodeExample:
  id: string                    # e.g., "basic-world-sdf"
  language: enum                # xml | python | csharp | bash | yaml
  title: string                 # e.g., "Basic Gazebo World File"
  source_file: string           # Path to downloadable file
  inline_code: string           # Full code for embedding
  run_instructions: string      # How to execute
  expected_output: string       # What student should see
  platform: enum                # gazebo | unity | ros2 | any
```

### Diagram

```yaml
Diagram:
  id: string                    # e.g., "coordinate-systems"
  title: string                 # e.g., "ROS vs Unity Coordinate Systems"
  type: enum                    # mermaid | svg | png
  source: string                # Mermaid code or file path
  alt_text: string              # Accessibility description
  caption: string               # Figure caption
```

### Sensor

```yaml
Sensor:
  type: enum                    # lidar | depth_camera | imu
  platform: enum                # gazebo | unity
  config_parameters:
    - name: string
    - type: string
    - default: any
    - description: string
  output_format: string         # ROS message type
  noise_model:
    type: enum                  # gaussian | uniform | none
    parameters: object
```

## Chapter Content Models

### Chapter 1: Physics Simulation in Gazebo

```yaml
chapter_01:
  id: "chapter-01-gazebo-physics"
  title: "Physics Simulation in Gazebo"
  user_story: "US1"
  priority: "P1"

  learning_objectives:
    - "Understand Gazebo Harmonic architecture and components"
    - "Create SDF world files with ground planes and objects"
    - "Configure physics parameters (gravity, friction, collisions)"
    - "Load and position robot models in simulation"
    - "Verify physics behavior through visual inspection"

  sections:
    - id: "gazebo-overview"
      title: "Introduction to Gazebo Harmonic"
      content_type: theory
      topics:
        - "Gazebo vs Classic Gazebo naming"
        - "Architecture: gz-sim, gz-physics, gz-rendering"
        - "Integration with ROS 2 via ros_gz"

    - id: "sdf-world-structure"
      title: "SDF World File Structure"
      content_type: tutorial
      topics:
        - "SDF 1.10 specification overview"
        - "World element and children"
        - "Including models from Fuel"
        - "Pose and coordinate frames"
      code_examples:
        - "minimal-world.sdf"
        - "world-with-objects.sdf"

    - id: "physics-configuration"
      title: "Configuring Physics Properties"
      content_type: tutorial
      topics:
        - "Physics engines: ODE, DART, Bullet"
        - "Step size and real-time factor"
        - "Gravity vector configuration"
        - "Surface friction and contact parameters"
        - "Collision geometries"
      code_examples:
        - "physics-config.sdf"
        - "collision-demo.sdf"

    - id: "robot-in-world"
      title: "Loading Robot Models"
      content_type: tutorial
      topics:
        - "Including URDF via SDF"
        - "Model spawning with gz service"
        - "Initial pose configuration"
        - "Joint state visualization"
      code_examples:
        - "humanoid-world.sdf"
        - "spawn-robot.bash"

    - id: "verification"
      title: "Verifying Physics Behavior"
      content_type: tutorial
      topics:
        - "Using gz sim GUI"
        - "Applying forces interactively"
        - "Recording and playback"
        - "Common debugging techniques"

  exercises:
    - "Create a world with a table and objects that fall with gravity"
    - "Modify friction to make a robot slide vs grip"
    - "Configure a pendulum and verify period matches physics"

  assessment_questions:
    - "What is the difference between Gazebo Harmonic and Gazebo Classic?"
    - "How does step size affect simulation accuracy vs performance?"
    - "Why might objects pass through each other in simulation?"
```

### Chapter 2: Unity for Human-Robot Interaction

```yaml
chapter_02:
  id: "chapter-02-unity-rendering"
  title: "Unity for Human-Robot Interaction Environments"
  user_story: "US2"
  priority: "P2"

  learning_objectives:
    - "Set up Unity with Robotics Hub packages"
    - "Import URDF models with correct coordinate transforms"
    - "Configure lighting for realistic rendering"
    - "Apply materials and textures to environments"
    - "Connect Unity to ROS 2 for external control"

  sections:
    - id: "unity-setup"
      title: "Unity Robotics Environment Setup"
      content_type: tutorial
      topics:
        - "Installing Unity Hub and 2022 LTS"
        - "Adding Robotics Hub packages"
        - "Project settings for simulation"
        - "Understanding Unity's coordinate system"
      code_examples:
        - "manifest.json"

    - id: "urdf-import"
      title: "Importing Robot Models"
      content_type: tutorial
      topics:
        - "URDF Importer workflow"
        - "Coordinate frame transformations"
        - "Collision vs visual meshes"
        - "Joint configuration and limits"
      code_examples:
        - "ImportRobot.cs"

    - id: "scene-composition"
      title: "Building Indoor Environments"
      content_type: tutorial
      topics:
        - "Creating basic room geometry"
        - "ProBuilder for quick prototyping"
        - "Importing external assets"
        - "Scale and proportion considerations"

    - id: "lighting-materials"
      title: "Lighting and Materials"
      content_type: tutorial
      topics:
        - "Light types: directional, point, spot, area"
        - "Global illumination basics"
        - "PBR materials and textures"
        - "Reflection probes"
      code_examples:
        - "MaterialSetup.cs"

    - id: "ros2-integration"
      title: "ROS 2 Integration"
      content_type: tutorial
      topics:
        - "ROS-TCP-Connector setup"
        - "Publishing and subscribing in Unity"
        - "Joint state synchronization"
        - "Clock synchronization"
      code_examples:
        - "ROSConnection.cs"
        - "JointStatePublisher.cs"

  exercises:
    - "Import the humanoid URDF from Module 1 into Unity"
    - "Create a simple room with table, chair, and lighting"
    - "Publish joint states from Unity to ROS 2 and visualize in RViz"

  assessment_questions:
    - "How does Unity's coordinate system differ from ROS?"
    - "What is the purpose of reflection probes?"
    - "Why might a URDF import have incorrect joint orientations?"
```

### Chapter 3: Sensor Simulation

```yaml
chapter_03:
  id: "chapter-03-sensor-simulation"
  title: "Sensor Simulation: LiDAR, Depth Cameras, IMUs"
  user_story: "US3"
  priority: "P3"

  learning_objectives:
    - "Configure LiDAR sensors in Gazebo with appropriate parameters"
    - "Set up depth cameras in both Gazebo and Unity"
    - "Implement IMU simulation with noise models"
    - "Visualize sensor data in RViz and custom tools"
    - "Understand sensor data formats and message types"

  sections:
    - id: "sensor-overview"
      title: "Introduction to Robot Sensors"
      content_type: theory
      topics:
        - "Proprioceptive vs exteroceptive sensors"
        - "Sensor data in perception pipelines"
        - "Real vs simulated sensor characteristics"
        - "ROS 2 sensor message types"

    - id: "lidar-gazebo"
      title: "LiDAR Simulation in Gazebo"
      content_type: tutorial
      topics:
        - "GPU LiDAR sensor configuration"
        - "Range, resolution, field of view"
        - "Noise model parameters"
        - "Point cloud output format"
        - "ros_gz_bridge for ROS 2"
      code_examples:
        - "lidar-sensor.sdf"
        - "ros-gz-lidar-bridge.yaml"

    - id: "depth-camera"
      title: "Depth Camera Simulation"
      content_type: tutorial
      topics:
        - "Gazebo RGBD camera setup"
        - "Unity depth shader implementation"
        - "Intrinsic parameters (focal length, principal point)"
        - "Depth encoding formats"
        - "Visualizing depth images"
      code_examples:
        - "depth-camera.sdf"
        - "DepthCameraController.cs"

    - id: "imu-simulation"
      title: "IMU Simulation"
      content_type: tutorial
      topics:
        - "Gazebo IMU sensor plugin"
        - "Unity Rigidbody-based IMU"
        - "Acceleration and angular velocity"
        - "Orientation estimation"
        - "Noise and bias modeling"
      code_examples:
        - "imu-sensor.sdf"
        - "IMUSensor.cs"

    - id: "visualization"
      title: "Sensor Data Visualization"
      content_type: tutorial
      topics:
        - "RViz2 point cloud display"
        - "Image view for depth/RGB"
        - "IMU orientation visualization"
        - "Custom visualization tools"
      code_examples:
        - "rviz-sensor-config.yaml"

    - id: "integration"
      title: "Building a Complete Sensor Suite"
      content_type: tutorial
      topics:
        - "Multi-sensor robot configuration"
        - "Sensor fusion concepts (preview)"
        - "Performance considerations"
        - "Debugging sensor issues"
      code_examples:
        - "multi-sensor-robot.sdf"

  exercises:
    - "Add a LiDAR to the humanoid robot and visualize the scan"
    - "Configure a depth camera and measure distance to an object"
    - "Implement an IMU and verify readings match robot motion"
    - "Create a sensor suite with all three sensors publishing to ROS 2"

  assessment_questions:
    - "What parameters affect LiDAR scan density?"
    - "How does Gaussian noise affect depth camera accuracy?"
    - "Why is IMU bias important to model in simulation?"
```

## Sensor Configuration Reference

### LiDAR Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `horizontal_samples` | int | 640 | Points per horizontal scan |
| `vertical_samples` | int | 16 | Vertical scan lines |
| `horizontal_fov` | float | 6.28 (360°) | Horizontal field of view (rad) |
| `vertical_fov` | float | 0.52 (30°) | Vertical field of view (rad) |
| `min_range` | float | 0.1 | Minimum detection range (m) |
| `max_range` | float | 100.0 | Maximum detection range (m) |
| `noise_stddev` | float | 0.01 | Gaussian noise standard deviation (m) |
| `update_rate` | float | 10.0 | Scan frequency (Hz) |

### Depth Camera Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `width` | int | 640 | Image width (pixels) |
| `height` | int | 480 | Image height (pixels) |
| `horizontal_fov` | float | 1.047 (60°) | Horizontal field of view (rad) |
| `clip_near` | float | 0.1 | Near clipping plane (m) |
| `clip_far` | float | 10.0 | Far clipping plane (m) |
| `noise_stddev` | float | 0.005 | Depth noise (m) |
| `update_rate` | float | 30.0 | Frame rate (Hz) |

### IMU Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_rate` | float | 100.0 | Measurement frequency (Hz) |
| `accel_noise` | float | 0.01 | Accelerometer noise (m/s²) |
| `gyro_noise` | float | 0.001 | Gyroscope noise (rad/s) |
| `accel_bias` | float | 0.0 | Accelerometer bias (m/s²) |
| `gyro_bias` | float | 0.0 | Gyroscope bias (rad/s) |

## RAG Chunk Strategy

Following Module 1 patterns:
- **Chunk by h2/h3 headings**: Each section becomes a retrievable unit
- **Tag code blocks**: Separate code chunks with language and platform metadata
- **Preserve diagrams**: Include alt_text in chunk for searchability
- **Cross-reference**: Link to related chunks (e.g., Gazebo LiDAR → Unity LiDAR)

Expected chunks per chapter:
- Chapter 1: ~15-20 chunks
- Chapter 2: ~15-20 chunks
- Chapter 3: ~20-25 chunks (more subsections for each sensor type)
