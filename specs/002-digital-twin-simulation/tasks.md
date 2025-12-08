# Implementation Tasks: Module 2 — The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-simulation
**Branch**: `002-digital-twin-simulation`
**Date**: 2025-12-08
**Total Tasks**: 78
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## User Story Mapping

| Phase | User Story | Priority | Tasks | Independent Test |
|-------|------------|----------|-------|------------------|
| 3 | US1: Physics Simulation in Gazebo | P1 | 20 | Launch Gazebo world with humanoid robot, verify physics |
| 4 | US2: Unity Human-Robot Interaction | P2 | 18 | Import robot in Unity, connect to ROS 2, verify joint states |
| 5 | US3: Sensor Simulation | P3 | 24 | Configure sensors, verify data in RViz2 |

## Task Summary

- **Phase 1 (Setup)**: 8 tasks
- **Phase 2 (Foundational)**: 8 tasks
- **Phase 3 (US1/Chapter 1)**: 20 tasks
- **Phase 4 (US2/Chapter 2)**: 18 tasks
- **Phase 5 (US3/Chapter 3)**: 24 tasks
- **Phase 6 (Polish)**: 8 tasks (cross-cutting)

---

## Phase 1: Setup (Project Initialization)

**Goal**: Initialize Module 2 directory structure and example project scaffolds.

- [x] T001 Create module directory structure at book/docs/module-02-digital-twin/
- [x] T002 Create _category_.json for Module 2 at book/docs/module-02-digital-twin/_category_.json
- [x] T003 [P] Create module overview page at book/docs/module-02-digital-twin/index.md
- [x] T004 [P] Create assets directories at book/docs/module-02-digital-twin/assets/diagrams/ and assets/code/
- [x] T005 [P] Create Gazebo examples directory structure at examples/gazebo/worlds/, examples/gazebo/models/, examples/gazebo/config/, examples/gazebo/launch/
- [x] T006 [P] Create Unity examples directory structure at examples/unity/Assets/Scripts/ROS/, examples/unity/Assets/Scripts/Sensors/, examples/unity/Packages/
- [x] T007 [P] Create ROS 2 package directory at examples/ros2_ws/src/digital_twin_examples/
- [x] T008 Update sidebars.js to include Module 2 in navigation at book/sidebars.js

**Checkpoint**: Directory structure exists, `npm run build` succeeds with placeholder content.

---

## Phase 2: Foundational (Prerequisites for All Chapters)

**Goal**: Create shared assets and configurations used across all three chapters.

- [x] T009 Create ROS 2 package.xml for digital_twin_examples at examples/ros2_ws/src/digital_twin_examples/package.xml
- [x] T010 [P] Create ROS 2 setup.py for digital_twin_examples at examples/ros2_ws/src/digital_twin_examples/setup.py
- [x] T011 [P] Create __init__.py for Python package at examples/ros2_ws/src/digital_twin_examples/digital_twin_examples/__init__.py
- [x] T012 [P] Create Unity manifest.json with Robotics Hub dependencies at examples/unity/Packages/manifest.json
- [x] T013 [P] Create coordinate system comparison diagram (Mermaid/SVG) at book/docs/module-02-digital-twin/assets/diagrams/coordinate-systems.svg
- [x] T014 [P] Create base ros_gz_bridge.yaml configuration at examples/gazebo/config/ros_gz_bridge.yaml
- [x] T015 [P] Create digital_twin.launch.py base launch file at examples/gazebo/launch/digital_twin.launch.py
- [x] T016 Create shared humanoid model reference (copy or link from Module 1) at examples/gazebo/models/humanoid_robot/

**Checkpoint**: ROS 2 package builds with `colcon build`, Unity manifest valid.

---

## Phase 3: US1 — Physics Simulation in Gazebo (Chapter 1)

**User Story**: As a robotics student, I want to learn how to create physics-based simulation worlds in Gazebo so that I can test robot behaviors in realistic virtual environments.

**Independent Test**: Launch Gazebo world with humanoid robot, apply forces, observe physically accurate collision and gravity responses.

### Code Examples

- [x] T017 [P] [US1] Create minimal-world.sdf with ground plane and sun at examples/gazebo/worlds/minimal-world.sdf
- [x] T018 [P] [US1] Create world-with-objects.sdf with static objects at examples/gazebo/worlds/world-with-objects.sdf
- [x] T019 [P] [US1] Create physics-config.sdf demonstrating physics parameters at examples/gazebo/worlds/physics-config.sdf
- [x] T020 [P] [US1] Create collision-demo.sdf showing friction differences at examples/gazebo/worlds/collision-demo.sdf
- [x] T021 [US1] Create humanoid-world.sdf loading robot model at examples/gazebo/worlds/humanoid-world.sdf
- [x] T022 [US1] Create spawn-robot.bash script for runtime spawning at examples/gazebo/scripts/spawn-robot.bash

### Diagrams

- [x] T023 [P] [US1] Create Gazebo Architecture diagram at book/docs/module-02-digital-twin/assets/diagrams/gazebo-architecture.svg
- [x] T024 [P] [US1] Create Physics Timestep Trade-off diagram at book/docs/module-02-digital-twin/assets/diagrams/physics-timestep-tradeoff.svg
- [x] T025 [P] [US1] Create URDF to SDF Workflow diagram at book/docs/module-02-digital-twin/assets/diagrams/urdf-sdf-workflow.svg

### Chapter Content

- [x] T026 [US1] Write Section 1: Introduction to Gazebo Harmonic (theory) at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx
- [x] T027 [US1] Write Section 2: SDF World File Structure (tutorial) at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx
- [x] T028 [US1] Write Section 3: Configuring Physics Properties (tutorial) at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx
- [x] T029 [US1] Write Section 4: Loading Robot Models (tutorial) at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx
- [x] T030 [US1] Write Section 5: Verifying Physics Behavior (tutorial) at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx
- [x] T031 [US1] Add learning objectives and prerequisites to Chapter 1 at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx
- [x] T032 [US1] Write exercises section for Chapter 1 at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx
- [x] T033 [US1] Write assessment questions for Chapter 1 at book/docs/module-02-digital-twin/chapter-01-gazebo-physics.mdx

### Validation

- [x] T034 [US1] Validate all SDF files with `gz sdf -p` command
- [x] T035 [US1] Test humanoid-world.sdf launches with `gz sim` command
- [x] T036 [US1] Verify Chapter 1 content builds with Docusaurus

**Checkpoint**: Chapter 1 complete, all SDF files valid, Gazebo world launches with robot.

---

## Phase 4: US2 — Unity for Human-Robot Interaction (Chapter 2)

**User Story**: As a robotics student, I want to learn Unity for creating photorealistic environments so that I can generate synthetic training data and test human-robot interaction scenarios.

**Independent Test**: Import robot model in Unity, configure lighting, connect to ROS 2, verify joint states publish correctly.

### Code Examples

- [ ] T037 [P] [US2] Create ImportRobot.cs helper script at examples/unity/Assets/Scripts/ImportRobot.cs
- [ ] T038 [P] [US2] Create MaterialSetup.cs for programmatic materials at examples/unity/Assets/Scripts/MaterialSetup.cs
- [ ] T039 [P] [US2] Create ROSConnection.cs setup script at examples/unity/Assets/Scripts/ROS/ROSConnection.cs
- [ ] T040 [US2] Create JointStatePublisher.cs for joint state publishing at examples/unity/Assets/Scripts/ROS/JointStatePublisher.cs
- [ ] T041 [US2] Create ros_tcp_endpoint.launch.py for ROS 2 side at examples/ros2_ws/src/digital_twin_examples/launch/ros_tcp_endpoint.launch.py

### Diagrams

- [ ] T042 [P] [US2] Create URDF Import Workflow diagram at book/docs/module-02-digital-twin/assets/diagrams/urdf-import-workflow.svg
- [ ] T043 [P] [US2] Create PBR Material Properties diagram at book/docs/module-02-digital-twin/assets/diagrams/pbr-material-properties.svg
- [ ] T044 [P] [US2] Create Unity-ROS Architecture diagram at book/docs/module-02-digital-twin/assets/diagrams/unity-ros-architecture.svg
- [ ] T045 [P] [US2] Create Scene Hierarchy Best Practices diagram at book/docs/module-02-digital-twin/assets/diagrams/scene-hierarchy.svg

### Chapter Content

- [ ] T046 [US2] Write Section 1: Unity Robotics Environment Setup at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T047 [US2] Write Section 2: Importing Robot Models at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T048 [US2] Write Section 3: Building Indoor Environments at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T049 [US2] Write Section 4: Lighting and Materials at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T050 [US2] Write Section 5: ROS 2 Integration at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T051 [US2] Add learning objectives and prerequisites to Chapter 2 at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T052 [US2] Write exercises section for Chapter 2 at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T053 [US2] Write assessment questions for Chapter 2 at book/docs/module-02-digital-twin/chapter-02-unity-rendering.mdx
- [ ] T054 [US2] Verify Chapter 2 content builds with Docusaurus

**Checkpoint**: Chapter 2 complete, C# scripts compile, ROS 2 integration documented.

---

## Phase 5: US3 — Sensor Simulation (Chapter 3)

**User Story**: As a robotics student, I want to learn how to simulate LiDAR, depth cameras, and IMUs so that I can develop and test perception algorithms without physical sensors.

**Independent Test**: Configure all three sensor types, verify data publishes to ROS 2, visualize in RViz2.

### Gazebo Sensor Configs

- [ ] T055 [P] [US3] Create lidar-sensor.sdf configuration snippet at examples/gazebo/sensors/lidar-sensor.sdf
- [ ] T056 [P] [US3] Create depth-camera.sdf configuration at examples/gazebo/sensors/depth-camera.sdf
- [ ] T057 [P] [US3] Create imu-sensor.sdf configuration at examples/gazebo/sensors/imu-sensor.sdf
- [ ] T058 [US3] Create multi-sensor-robot.sdf with all sensors at examples/gazebo/models/humanoid_with_sensors/model.sdf
- [ ] T059 [US3] Create model.config for sensor-equipped robot at examples/gazebo/models/humanoid_with_sensors/model.config
- [ ] T060 [US3] Create ros-gz-sensor-bridge.yaml for all sensors at examples/gazebo/config/ros-gz-sensor-bridge.yaml

### Unity Sensor Scripts

- [ ] T061 [P] [US3] Create LiDARSimulator.cs raycast-based LiDAR at examples/unity/Assets/Scripts/Sensors/LiDARSimulator.cs
- [ ] T062 [P] [US3] Create DepthCameraController.cs depth rendering at examples/unity/Assets/Scripts/Sensors/DepthCameraController.cs
- [ ] T063 [P] [US3] Create IMUSensor.cs Rigidbody-based IMU at examples/unity/Assets/Scripts/Sensors/IMUSensor.cs

### ROS 2 Integration

- [ ] T064 [US3] Create sensor_launch.py for all sensors and bridges at examples/ros2_ws/src/digital_twin_examples/launch/sensor_launch.py
- [ ] T065 [US3] Create rviz-sensor-config.yaml visualization config at examples/ros2_ws/src/digital_twin_examples/config/rviz-sensor-config.yaml

### Diagrams

- [ ] T066 [P] [US3] Create Sensor Types on Humanoid diagram at book/docs/module-02-digital-twin/assets/diagrams/sensor-types-humanoid.svg
- [ ] T067 [P] [US3] Create LiDAR Scan Pattern diagram at book/docs/module-02-digital-twin/assets/diagrams/lidar-scan-pattern.svg
- [ ] T068 [P] [US3] Create Pinhole Camera Model diagram at book/docs/module-02-digital-twin/assets/diagrams/pinhole-camera-model.svg
- [ ] T069 [P] [US3] Create IMU Axes diagram at book/docs/module-02-digital-twin/assets/diagrams/imu-axes.svg
- [ ] T070 [P] [US3] Create TF Tree diagram for multi-sensor robot at book/docs/module-02-digital-twin/assets/diagrams/tf-tree-sensors.svg

### Chapter Content

- [ ] T071 [US3] Write Section 1: Introduction to Robot Sensors (theory) at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T072 [US3] Write Section 2: LiDAR Simulation in Gazebo at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T073 [US3] Write Section 3: Depth Camera Simulation at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T074 [US3] Write Section 4: IMU Simulation at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T075 [US3] Write Section 5: Sensor Data Visualization at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T076 [US3] Write Section 6: Building a Complete Sensor Suite at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T077 [US3] Add learning objectives and prerequisites to Chapter 3 at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T078 [US3] Write exercises section for Chapter 3 at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T079 [US3] Write assessment questions for Chapter 3 at book/docs/module-02-digital-twin/chapter-03-sensor-simulation.mdx
- [ ] T080 [US3] Validate all sensor SDF files with `gz sdf -p`
- [ ] T081 [US3] Test sensor launch file starts all bridges
- [ ] T082 [US3] Verify Chapter 3 content builds with Docusaurus

**Checkpoint**: Chapter 3 complete, all sensors produce data, RViz2 visualization works.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Final quality validation, cross-references, and build verification.

- [ ] T083 Add cross-references between chapters (coordinate systems, sensor prereqs)
- [ ] T084 Verify all internal links resolve correctly
- [ ] T085 Run Docusaurus build validation for entire Module 2
- [ ] T086 Validate all code examples have "How to Run" sections
- [ ] T087 Verify IEEE citations for all external references
- [ ] T088 Create downloadable ZIP of all example files at book/static/examples/module-02/
- [ ] T089 Final proofread of all chapter content for clarity
- [ ] T090 Update book introduction (intro.md) to reference Module 2

**Final Checkpoint**: `npm run build` succeeds, all validation passes, module ready for review.

---

## Dependencies & Execution Order

```text
Phase 1 (Setup) ──────────────────────────────────────────┐
    │                                                      │
    ▼                                                      │
Phase 2 (Foundational) ───────────────────────────────────┤
    │                                                      │
    ├─────────────┬─────────────┬─────────────────────────┘
    │             │             │
    ▼             ▼             ▼
Phase 3       Phase 4       Phase 5
(US1/Ch1)     (US2/Ch2)     (US3/Ch3)
Gazebo        Unity         Sensors
Physics       Rendering     (Both)
    │             │             │
    └─────────────┴─────────────┘
                  │
                  ▼
           Phase 6 (Polish)
```

**Parallel Opportunities**:
- Phase 1: T003-T007 can run in parallel
- Phase 2: T010-T015 can run in parallel
- Phase 3-5: Chapters can be developed in parallel after Phase 2
- Within each chapter: Code examples and diagrams can be created in parallel

---

## Implementation Strategy

### MVP Scope (Minimal Viable Content)

Complete Phase 1 + Phase 2 + Phase 3 (US1/Chapter 1) for minimal viable module.

**MVP Deliverables**:
- Module directory structure
- Chapter 1: Physics Simulation in Gazebo
- All Gazebo SDF examples
- Gazebo architecture diagrams

**MVP Test**: Student can create and launch a Gazebo world with physics-enabled robot.

### Incremental Delivery

1. **Increment 1** (MVP): Chapters 1 (Gazebo Physics)
2. **Increment 2**: Chapter 2 (Unity Rendering)
3. **Increment 3**: Chapter 3 (Sensors)
4. **Increment 4**: Polish and cross-references

Each increment is independently deployable and testable.

---

## Format Validation

All 90 tasks follow checklist format:
- ✅ Checkbox prefix (`- [ ]`)
- ✅ Task ID (T001-T090)
- ✅ [P] marker for parallelizable tasks
- ✅ [US#] story labels for chapter tasks
- ✅ File paths in descriptions
