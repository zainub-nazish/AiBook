# Implementation Plan: Module 2 — The Digital Twin (Gazebo & Unity)

**Branch**: `002-digital-twin-simulation` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-digital-twin-simulation/spec.md`

## Summary

Create Module 2 (3 chapters) for the Physical AI & Humanoid Robotics textbook covering digital twin development. This module teaches physics simulation in Gazebo Harmonic, photorealistic rendering in Unity with ROS 2 integration, and sensor simulation (LiDAR, depth cameras, IMUs). All content will be Docusaurus-compatible Markdown with runnable simulation configurations validated against current tool versions.

## Technical Context

**Language/Version**: Markdown (MDX), Python 3.10+ (ROS 2 scripts), C# (Unity scripts), XML (SDF/URDF)
**Primary Dependencies**: Docusaurus 3.x, Gazebo Harmonic, Unity 2022 LTS, ROS 2 Humble, ros_gz_bridge, Unity Robotics Hub
**Storage**: Git (content), GitHub Pages (deployment)
**Testing**: Docusaurus build validation, `gz sdf -p` for SDF validation, Unity project compilation, ROS 2 launch tests
**Target Platform**: Web (Docusaurus static site), Gazebo/Unity on Ubuntu 22.04 (simulation examples)
**Project Type**: Documentation/Textbook (Docusaurus static site)
**Performance Goals**: Book builds in <2 minutes, all pages load in <3 seconds
**Constraints**: All configs must run on stated tool versions, no fictional APIs, IEEE citation format, APA style per Constitution
**Scale/Scope**: 3 chapters, ~60-80 pages total, 20+ code examples, 15+ diagrams

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| I. Technical Accuracy | Verify against official Gazebo/Unity docs | ✅ PASS | Research.md cites gazebosim.org, Unity docs |
| II. Educational Clarity | Learning objectives, prerequisites, diagrams | ✅ PASS | Each chapter has 5 objectives, 15+ diagrams planned |
| III. Reproducible Code | Complete, runnable SDF/Unity configs | ✅ PASS | Full world files, Unity scripts, validated |
| IV. AI-Native Workflow | SDD workflow with PHRs | ✅ PASS | Following /sp.plan → /sp.tasks flow |
| V. Citation Integrity | IEEE format, no fabricated refs | ✅ PASS | Links to official docs, REPs cited |
| VI. Modular Curriculum | Part of 12+ chapter structure | ✅ PASS | Module 2 of 12, follows ROS 2 Fundamentals |
| VII. RAG Chatbot Integrity | Content extractable for RAG | ✅ PASS | Clean Markdown, semantic headings, code tagged |
| VIII. Build & Deploy Reliability | Docusaurus builds without errors | ✅ PASS | CI will validate on every commit |

**Gate Status**: ALL PASS — Proceed to Phase 0 (completed) and Phase 1 (completed)

## Project Structure

### Documentation (this feature)

```text
specs/002-digital-twin-simulation/
├── plan.md              # This file
├── research.md          # Phase 0: Gazebo/Unity best practices, version decisions
├── data-model.md        # Phase 1: Chapter/content model, sensor parameters
├── quickstart.md        # Phase 1: Development environment setup
├── contracts/           # Phase 1: Chapter outlines
│   ├── chapter-01-outline.md  # Gazebo Physics
│   ├── chapter-02-outline.md  # Unity Rendering
│   └── chapter-03-outline.md  # Sensor Simulation
├── checklists/
│   └── requirements.md  # Spec quality validation
└── tasks.md             # Phase 2: Implementation tasks (/sp.tasks)
```

### Source Code (Book Content)

```text
book/
├── docusaurus.config.js     # Site configuration (already configured)
├── sidebars.js              # Navigation structure (update for Module 2)
├── docs/
│   ├── intro.md             # Book introduction
│   ├── module-01-ros2/      # Module 1 (existing)
│   └── module-02-digital-twin/
│       ├── _category_.json  # Module metadata
│       ├── index.md         # Module overview
│       ├── chapter-01-gazebo-physics.mdx
│       ├── chapter-02-unity-rendering.mdx
│       ├── chapter-03-sensor-simulation.mdx
│       └── assets/
│           ├── diagrams/    # Architecture diagrams, sensor illustrations
│           └── code/        # Downloadable config files
└── static/
    └── examples/
        └── module-02/       # Downloadable complete examples
```

### Simulation Examples

```text
examples/
├── gazebo/
│   ├── worlds/
│   │   ├── minimal_world.sdf
│   │   ├── physics_demo.sdf
│   │   ├── humanoid_world.sdf
│   │   └── multi_sensor_world.sdf
│   ├── models/
│   │   └── humanoid_with_sensors/
│   │       ├── model.config
│   │       └── model.sdf
│   ├── config/
│   │   └── ros_gz_bridge.yaml
│   └── launch/
│       └── digital_twin.launch.py
├── unity/
│   ├── Assets/
│   │   ├── Scripts/
│   │   │   ├── ROS/
│   │   │   │   ├── ROSConnection.cs
│   │   │   │   └── JointStatePublisher.cs
│   │   │   └── Sensors/
│   │   │       ├── LiDARSimulator.cs
│   │   │       ├── DepthCameraController.cs
│   │   │       └── IMUSensor.cs
│   │   ├── Prefabs/
│   │   └── Scenes/
│   └── Packages/
│       └── manifest.json
└── ros2_ws/
    └── src/
        └── digital_twin_examples/
            ├── package.xml
            ├── setup.py
            ├── config/
            ├── launch/
            └── digital_twin_examples/
```

**Structure Decision**: Follows Module 1 patterns with separate Gazebo and Unity example directories. Unity project is self-contained for easy import. ROS 2 workspace extends Module 1 workspace.

## Chapter Organization

| Chapter | Title | User Story | Pages | Code Examples |
|---------|-------|------------|-------|---------------|
| 1 | Physics Simulation in Gazebo | US1 (P1) | 20-25 | 6 SDF files, 2 scripts |
| 2 | Unity for Human-Robot Interaction | US2 (P2) | 20-25 | 5 C# scripts, 2 configs |
| 3 | Sensor Simulation | US3 (P3) | 25-30 | 8 configs (both platforms) |

## Writing Workflow

Following Module 1 established patterns:

### Phase Flow: Research → Foundation → Analysis → Synthesis

```text
┌──────────┐    ┌────────────┐    ┌──────────┐    ┌───────────┐
│ RESEARCH │ → │ FOUNDATION │ → │ ANALYSIS │ → │ SYNTHESIS │
└──────────┘    └────────────┘    └──────────┘    └───────────┘
     │               │                 │               │
     ▼               ▼                 ▼               ▼
 Verify APIs     Write core       Add diagrams    Polish prose
 Check versions  tutorials        Deep examples   Assessment Qs
 Document refs   Basic code       Edge cases      Final review
```

### Per-Chapter Workflow

**1. RESEARCH** (Before writing)
- Verify API documentation current
- Test example configurations work
- Document version-specific behavior
- Collect reference URLs

**2. FOUNDATION** (Initial draft)
- Write learning objectives
- Create section structure
- Write core explanations
- Add basic code examples with "How to Run"

**3. ANALYSIS** (Depth pass)
- Add detailed diagrams
- Expand edge case coverage
- Include admonitions (tips, warnings)
- Cross-reference related content

**4. SYNTHESIS** (Polish pass)
- Add exercises with clear outcomes
- Write assessment questions
- Verify all citations
- Final prose polish

## Quality Validation Plan

### Automated Checks (CI/CD)

| Check | Tool | Trigger | Failure Action |
|-------|------|---------|----------------|
| Docusaurus Build | `npm run build` | Every commit | Block merge |
| Markdown Lint | markdownlint-cli | Every commit | Warning |
| Link Validation | linkinator | PR | Block merge |
| SDF Validation | `gz sdf -p` | PR | Block merge |
| Python Syntax | Python AST | PR | Block merge |
| C# Compilation | Unity batch build | PR (optional) | Warning |

### Manual Review Checklist

- [ ] Learning objectives match chapter content
- [ ] All code examples have "How to Run" section
- [ ] Technical claims cite official documentation
- [ ] Diagrams are clear and correctly labeled
- [ ] Coordinate system differences clearly explained
- [ ] Exercises have clear expected outcomes
- [ ] Prerequisite knowledge accurately stated
- [ ] Sensor parameters documented with units

### Acceptance Criteria Validation

From spec success criteria:

| Criterion | Validation Method |
|-----------|------------------|
| SC-001: Gazebo world in <30 min | Time test by reviewer |
| SC-002: Unity robot import in <45 min | Time test by reviewer |
| SC-003: All sensors produce valid data | Automated test with expected output |
| SC-004: 100% configs execute | CI runs all configs |
| SC-005: Zero doc contradictions | Manual review vs official docs |
| SC-006: Minimal digital twin works | End-to-end test |
| SC-007: Clear diagrams | Review checklist item |
| SC-008: 80%+ comprehension | Assessment question analysis |

## Architectural Decisions

### ADR-001: Gazebo Harmonic over Classic

**Decision**: Use Gazebo Harmonic (formerly Ignition Gazebo) for all physics simulation content.

**Rationale**:
- Active development and community support
- Better ROS 2 integration via ros_gz packages
- Modern architecture with better performance
- Consistent with ROS 2 Humble ecosystem

**Alternatives Considered**:
- Gazebo Classic (gazebo11): Deprecated, confusing naming, no new features
- Gazebo Garden: Older release, less documentation

### ADR-002: Unity 2022 LTS with Robotics Hub

**Decision**: Use Unity 2022 LTS with official Robotics Hub packages.

**Rationale**:
- LTS provides stability for educational content
- Official packages ensure maintenance and documentation
- URDF Importer handles coordinate transforms automatically
- ROS-TCP-Connector is actively maintained

**Alternatives Considered**:
- Unity 2023: Newer but less stable
- Community ROS packages: Less maintained, inconsistent quality

### ADR-003: Dual-Platform Sensor Coverage

**Decision**: Cover sensor simulation in both Gazebo and Unity for Chapter 3.

**Rationale**:
- Students may prefer one platform
- Different platforms have different strengths (Gazebo: physics accuracy; Unity: visual quality)
- Real-world projects may use either
- Demonstrates transferable concepts

**Trade-offs**:
- More content to maintain
- Students may be overwhelmed (mitigated by clear sections)

### ADR-004: Coordinate System Documentation Strategy

**Decision**: Dedicate explicit sections to coordinate system differences with visual diagrams and transformation examples.

**Rationale**:
- #1 source of errors for students
- Cannot be glossed over
- Visual diagrams essential for understanding
- Explicit transformation code prevents confusion

**Implementation**:
- Chapter 1: Introduce ROS/Gazebo convention (Z-up)
- Chapter 2: Compare to Unity convention (Y-up)
- Both chapters: Include transformation diagrams

## Key Decisions Summary

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Gazebo Version | Harmonic (LTS) | ROS 2 Humble alignment, active dev |
| SDF Version | 1.10 | Current spec, full feature support |
| Unity Version | 2022 LTS | Stability, Robotics Hub compatibility |
| Sensor Chapter Scope | Both platforms | Comprehensive coverage, transferable skills |
| Coordinate Documentation | Explicit + diagrams | Critical for student success |
| Code Format | Tabs (Full + Run) | Consistency with Module 1 |

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| API changes in Gazebo/Unity | Medium | High | Pin versions, add version notes |
| Coordinate confusion | High | Medium | Extensive documentation, diagrams |
| Unity package instability | Low | Medium | Test before each release |
| Long chapter load times | Low | Low | Optimize images, lazy-load code |

## Complexity Tracking

> No constitution violations requiring justification.

| Item | Complexity Level | Justification |
|------|-----------------|---------------|
| Docusaurus content | Low | Established patterns from Module 1 |
| Gazebo SDF configs | Medium | SDF syntax requires precision |
| Unity C# scripts | Medium | Requires Unity + ROS knowledge |
| Cross-platform sensors | Medium-High | Dual implementation, coordination |

## Next Steps

1. ✅ **Phase 0 Complete**: research.md with version decisions, API verification
2. ✅ **Phase 1 Complete**: data-model.md, chapter outlines, quickstart.md
3. **Run /sp.tasks**: Generate implementation tasks for chapters
4. **Begin Implementation**: Follow Research → Foundation → Analysis → Synthesis workflow

## Dependencies on Other Modules

- **Module 1 (ROS 2 Fundamentals)**: Students need nodes, topics, launch file knowledge
- **Module 3 (URDF)**: Can run in parallel, but URDF from Module 1 used here
- **Module 4 (Perception)**: Builds on sensor simulation from this module

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [SDF Specification 1.10](http://sdformat.org/spec)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [ros_gz Integration](https://github.com/gazebosim/ros_gz)
- [REP-103: Standard Units](https://www.ros.org/reps/rep-0103.html)
- [REP-145: IMU Conventions](https://www.ros.org/reps/rep-0145.html)
