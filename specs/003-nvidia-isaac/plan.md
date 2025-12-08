# Implementation Plan: Module 3 — The AI-Robot Brain (NVIDIA Isaac)

**Branch**: `003-nvidia-isaac` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/003-nvidia-isaac/spec.md`

## Summary

This module teaches robotics students how humanoid robots perceive and navigate their environments using the NVIDIA Isaac ecosystem. The implementation creates 5 chapters covering perception pipelines (VSLAM, depth processing, mapping, localization), Isaac Sim synthetic data generation, Isaac ROS GPU-accelerated perception nodes, Nav2 humanoid navigation, and end-to-end integration. Content follows Docusaurus MDX format with embedded diagrams, code snippets, and hands-on exercises.

## Technical Context

**Language/Version**: Python 3.10+ (ROS 2 Humble), MDX (Docusaurus 3.x)
**Primary Dependencies**: NVIDIA Isaac Sim 2023.1.0+, Isaac ROS, Nav2, ROS 2 Humble
**Storage**: N/A (static educational content)
**Testing**: `npm run build` validation, code example verification
**Target Platform**: Ubuntu 22.04 with NVIDIA GPU (CUDA 11.8+, driver 525+)
**Project Type**: Educational module (Docusaurus static site)
**Performance Goals**: All code examples run successfully on target platform
**Constraints**: Beginner-friendly explanations, APA/IEEE citations, <30 min setup per chapter
**Scale/Scope**: 5 chapters, ~15 diagrams, ~20 code examples, ~25 exercises

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Technical Accuracy | PASS | Content verified against NVIDIA Isaac docs, ROS 2 docs |
| II. Educational Clarity | PASS | Learning objectives, prerequisites, progressive complexity planned |
| III. Reproducible Code | PASS | All code includes setup instructions, version pinning |
| IV. AI-Native Workflow | PASS | Following SDD: specify → plan → tasks → implement |
| V. Citation Integrity | PASS | IEEE format citations planned, official doc links |
| VI. Modular Curriculum | PASS | Module 3 = "NVIDIA Isaac" in curriculum progression |
| VII. RAG Chatbot Integrity | N/A | Not applicable to module content |
| VIII. Build & Deploy Reliability | PASS | Docusaurus build validation planned |

**Gate Status**: ✅ PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/003-nvidia-isaac/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output (content entities)
├── quickstart.md        # Phase 1 output (dev quickstart)
├── contracts/           # Phase 1 output (chapter outlines)
│   ├── chapter-01-outline.md
│   ├── chapter-02-outline.md
│   ├── chapter-03-outline.md
│   ├── chapter-04-outline.md
│   └── chapter-05-outline.md
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
book/docs/module-03-ai-robot-brain/
├── _category_.json           # Docusaurus sidebar config
├── index.md                  # Module overview
├── chapter-01-perception.mdx # Perception Pipelines
├── chapter-02-isaac-sim.mdx  # Isaac Sim Synthetic Data
├── chapter-03-isaac-ros.mdx  # Isaac ROS Perception
├── chapter-04-nav2.mdx       # Nav2 Humanoid Navigation
├── chapter-05-integration.mdx # End-to-End Integration
└── assets/
    ├── diagrams/             # SVG/PNG diagrams
    │   ├── perception-pipeline.svg
    │   ├── isaac-ecosystem.svg
    │   ├── vslam-architecture.svg
    │   ├── isaac-sim-workflow.svg
    │   ├── domain-randomization.svg
    │   ├── isaac-ros-graph.svg
    │   ├── depth-processing.svg
    │   ├── nav2-architecture.svg
    │   ├── costmap-layers.svg
    │   ├── behavior-tree.svg
    │   └── integration-flow.svg
    └── code/                 # Referenced code snippets
        └── (inline in MDX)

examples/isaac/
├── synthetic_data/
│   ├── generate_rgb_depth.py
│   ├── domain_randomizer.py
│   └── export_dataset.py
├── isaac_ros/
│   ├── launch/
│   │   ├── vslam_demo.launch.py
│   │   └── perception_pipeline.launch.py
│   └── config/
│       ├── vslam_params.yaml
│       └── depth_params.yaml
└── nav2_humanoid/
    ├── launch/
    │   └── humanoid_nav.launch.py
    └── config/
        ├── nav2_params.yaml
        └── humanoid_footprint.yaml

examples/ros2_ws/src/
└── isaac_perception/
    ├── package.xml
    ├── setup.py
    └── isaac_perception/
        ├── __init__.py
        └── (nodes as needed)
```

**Structure Decision**: Educational module structure with Docusaurus chapters, supporting code examples in `examples/isaac/`, and ROS 2 package scaffolding for Isaac ROS integration examples.

## Complexity Tracking

> No constitution violations requiring justification.

## Architecture Decisions

### ADR-001: Isaac Sim Version Selection

**Decision**: Target Isaac Sim 2023.1.0 or later (Omniverse-based)
**Rationale**: 2023.x series provides stable Python API, better ROS 2 Humble integration, and improved documentation. Older versions have breaking API changes.
**Alternatives Considered**:
- Isaac Gym (rejected: deprecated in favor of Isaac Sim)
- Isaac SDK (rejected: superseded by Isaac ROS)

### ADR-002: Isaac ROS Package Selection

**Decision**: Focus on isaac_ros_visual_slam, isaac_ros_depth_image_proc, isaac_ros_nvblox
**Rationale**: These packages provide the core perception functionality (VSLAM, depth, mapping) that map directly to spec requirements. They're the most mature and well-documented Isaac ROS packages.
**Alternatives Considered**:
- Full Isaac ROS suite (rejected: too complex for beginner module)
- CPU-only alternatives (rejected: defeats purpose of GPU acceleration topic)

### ADR-003: Nav2 Integration Approach

**Decision**: Use Nav2 with custom humanoid footprint and behavior trees
**Rationale**: Nav2 is the standard ROS 2 navigation stack. Teaching custom footprint configuration covers humanoid-specific needs without requiring custom planners.
**Alternatives Considered**:
- Custom humanoid navigation stack (rejected: out of scope, too complex)
- MoveIt 2 for navigation (rejected: MoveIt is for manipulation, Nav2 for navigation)

### ADR-004: Synthetic Data Format

**Decision**: Export synthetic data in COCO format with depth as EXR
**Rationale**: COCO format is widely supported by ML frameworks. EXR preserves floating-point depth precision required for robotics applications.
**Alternatives Considered**:
- KITTI format (rejected: more complex, less universal)
- Custom format (rejected: reduces transferability of skills)

## Validation Strategy

### Per-Chapter Validation

| Chapter | Validation Method | Success Criteria |
|---------|------------------|------------------|
| Ch 1 | Concept quiz | Student identifies 4+ perception stages |
| Ch 2 | Isaac Sim lab | Student generates synthetic dataset |
| Ch 3 | Isaac ROS demo | VSLAM shows pose tracking in RViz2 |
| Ch 4 | Nav2 lab | Humanoid navigates to goal in simulation |
| Ch 5 | Integration test | Full pipeline runs sensors → navigation |

### Build Validation

```bash
# Validate Docusaurus build
cd book && npm run build

# Validate ROS 2 package
cd examples/ros2_ws && colcon build --packages-select isaac_perception

# Validate Python examples (syntax only)
python3 -m py_compile examples/isaac/synthetic_data/*.py
```

### Content Validation Checklist

- [ ] All code examples tested on Ubuntu 22.04 + NVIDIA GPU
- [ ] All diagrams render correctly in Docusaurus
- [ ] All citations link to valid URLs
- [ ] All learning objectives map to exercises
- [ ] All prerequisites reference prior modules
