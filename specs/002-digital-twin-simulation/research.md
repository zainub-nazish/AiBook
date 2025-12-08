# Phase 0 Research: Module 2 — The Digital Twin

**Feature**: 002-digital-twin-simulation
**Date**: 2025-12-08
**Purpose**: Resolve technical unknowns and document best practices for Gazebo/Unity simulation content.

## Research Tasks

### RT-001: Gazebo Version and SDF Format

**Question**: Which Gazebo version should be targeted, and what is the current SDF specification?

**Findings**:
- **Gazebo Harmonic** (gz-harmonic) is the current LTS release, paired with ROS 2 Humble/Iron
- Gazebo renamed from "Gazebo Classic" to "Gazebo" (formerly Ignition Gazebo)
- SDF 1.10 is the current specification for world/model files
- Key differences from Classic: new namespace (`gz::` instead of `gazebo::`), improved plugin architecture

**Decision**: Target **Gazebo Harmonic** with SDF 1.10 format.

**Rationale**: Aligns with ROS 2 Humble (Module 1 prerequisite), active development, best documentation.

**Alternatives Considered**:
- Gazebo Classic (gazebo11): Deprecated, no new features, confusing naming
- Gazebo Garden: Older release, less documentation than Harmonic

**Official Sources**:
- https://gazebosim.org/docs/harmonic
- http://sdformat.org/spec

---

### RT-002: Unity Robotics Integration

**Question**: What Unity packages are required for robotics simulation and ROS 2 integration?

**Findings**:
- **Unity Robotics Hub** provides official packages:
  - `com.unity.robotics.urdf-importer`: Import URDF models
  - `com.unity.robotics.ros-tcp-connector`: ROS 2 communication
  - `com.unity.perception`: Synthetic data generation (optional)
- Unity 2022 LTS is recommended for stability
- ROS-TCP-Endpoint package required on ROS 2 side
- Coordinate system: Unity uses left-handed Y-up; ROS uses right-handed Z-up

**Decision**: Use **Unity 2022 LTS** with Robotics Hub packages.

**Rationale**: Official Unity support, active maintenance, comprehensive documentation.

**Alternatives Considered**:
- Unity 2023: Newer but less stable, fewer tutorials
- ROS 2 for Unity (community): Less maintained than official packages

**Official Sources**:
- https://github.com/Unity-Technologies/Unity-Robotics-Hub
- https://github.com/Unity-Technologies/ROS-TCP-Connector

---

### RT-003: Sensor Plugin Architecture (Gazebo)

**Question**: How are sensors configured in Gazebo Harmonic?

**Findings**:
- Sensors defined in SDF within `<model>` or `<link>` elements
- Key sensor types for this module:
  - `<sensor type="gpu_lidar">`: LiDAR with GPU acceleration
  - `<sensor type="depth_camera">`: RGB-D cameras
  - `<sensor type="imu">`: Inertial measurement unit
- Sensor data published to gz-transport topics
- ROS 2 bridge (`ros_gz_bridge`) forwards to ROS topics
- Noise models configurable per sensor

**Decision**: Use native Gazebo sensors with `ros_gz_bridge` for ROS 2 integration.

**Rationale**: Official integration path, minimal custom code, noise models built-in.

**Official Sources**:
- https://gazebosim.org/api/sensors/8/index.html
- https://gazebosim.org/docs/harmonic/sensors

---

### RT-004: Unity Sensor Simulation

**Question**: How to simulate LiDAR, depth cameras, and IMU in Unity?

**Findings**:
- **LiDAR**: Unity's `Raycast` API or specialized assets (e.g., ROS-Industrial LiDAR)
- **Depth Camera**: Camera with depth shader, output to render texture
- **IMU**: Unity's `Rigidbody` provides acceleration/angular velocity
- `com.unity.perception` package provides ground truth and randomization
- Custom scripts required for ROS message formatting

**Decision**: Use **Perception package** for cameras, **custom scripts** for LiDAR/IMU with ROS-TCP-Connector.

**Rationale**: Perception package is official and well-documented; custom scripts give control over noise models.

**Alternatives Considered**:
- Third-party LiDAR assets: Variable quality, licensing concerns
- Full custom implementation: More work than necessary

**Official Sources**:
- https://docs.unity3d.com/Packages/com.unity.perception@latest
- https://github.com/Unity-Technologies/ROS-TCP-Connector

---

### RT-005: Coordinate System Conventions

**Question**: How to handle coordinate frame differences between Gazebo, Unity, and ROS?

**Findings**:
- **ROS/Gazebo**: Right-handed, Z-up (X forward, Y left, Z up)
- **Unity**: Left-handed, Y-up (Z forward, X right, Y up)
- URDF Importer handles transforms automatically when importing
- Manual transformations needed for custom scripts
- Transformation matrix: swap Y and Z, negate X

**Decision**: Document transformation clearly, rely on official importers where possible.

**Rationale**: Students must understand the difference; official tools handle most cases.

**Official Sources**:
- https://www.ros.org/reps/rep-0103.html (ROS coordinate conventions)
- Unity URDF Importer documentation

---

### RT-006: Docusaurus Structure for Simulation Content

**Question**: How should simulation-heavy content be structured in Docusaurus?

**Findings**:
- Code tabs (Tabs component) work well for SDF/Unity C# examples
- Admonitions (:::tip, :::warning) useful for coordinate system gotchas
- MDX allows embedding diagrams inline
- Large config files should be downloadable, not just inline
- Video embeds possible for simulation demos (optional enhancement)

**Decision**: Follow Module 1 patterns: Tabs for code, admonitions for warnings, downloadable complete files.

**Rationale**: Consistency across modules, proven patterns from Module 1.

---

### RT-007: Chapter Dependencies and Prerequisites

**Question**: What is the optimal chapter order for digital twin content?

**Findings**:
- Physics simulation (Gazebo) is foundational—must come first
- Unity rendering requires understanding coordinate systems from Gazebo
- Sensor simulation requires either Gazebo or Unity environment first
- Sensor chapter can demonstrate both Gazebo and Unity approaches

**Decision**: Chapter order: (1) Gazebo Physics → (2) Unity Rendering → (3) Sensors in Both.

**Rationale**: Progressive complexity, each chapter builds on previous.

---

## Best Practices Identified

### Gazebo World File Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="digital_twin_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Robot model -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Unity Project Structure for Robotics

```text
Assets/
├── Scripts/
│   ├── ROS/
│   │   ├── ROSConnection.cs
│   │   ├── LaserScanPublisher.cs
│   │   └── IMUPublisher.cs
│   └── Sensors/
│       ├── LiDARSimulator.cs
│       ├── DepthCameraController.cs
│       └── IMUSensor.cs
├── Prefabs/
│   └── Robots/
│       └── HumanoidRobot.prefab
├── Scenes/
│   └── DigitalTwinLab.unity
└── URDF/
    └── humanoid_robot/
```

### Sensor Noise Modeling

Both platforms should demonstrate:
1. Gaussian noise on sensor readings
2. Range limits (min/max distance)
3. Update rate configuration
4. Dropout/occlusion handling

---

## Technical Context Resolution

All NEEDS CLARIFICATION items from planning phase resolved:

| Item | Resolution |
|------|------------|
| Gazebo version | Gazebo Harmonic (LTS) |
| SDF specification | SDF 1.10 |
| Unity version | Unity 2022 LTS |
| Unity packages | Robotics Hub (URDF Importer, ROS-TCP-Connector) |
| Coordinate handling | Document explicitly, use official importers |
| Chapter order | Physics → Rendering → Sensors |

---

## Next Steps

1. Create `data-model.md` with chapter/section entities
2. Generate chapter outlines in `contracts/`
3. Create `quickstart.md` for development environment setup
4. Write `plan.md` consolidating all research findings
