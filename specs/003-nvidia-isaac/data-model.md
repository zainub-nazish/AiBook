# Data Model: Module 3 — The AI-Robot Brain (NVIDIA Isaac)

**Feature**: 003-nvidia-isaac
**Date**: 2025-12-08

## Content Entities

### Entity: Chapter

Represents a single chapter in Module 3.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique chapter identifier (e.g., "chapter-01") |
| title | string | Chapter title |
| slug | string | URL slug for Docusaurus |
| position | number | Order in sidebar (1-5) |
| userStory | reference | Maps to spec user story |
| prerequisites | string[] | Required prior knowledge |
| learningObjectives | string[] | What student will learn |
| estimatedTime | string | Expected completion time |
| sections | Section[] | Chapter sections |
| exercises | Exercise[] | Hands-on activities |
| assessments | Assessment[] | Verification questions |

### Entity: Section

Represents a section within a chapter.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Section identifier |
| title | string | Section heading |
| type | enum | theory, tutorial, reference, exercise |
| content | markdown | MDX content |
| diagrams | Diagram[] | Associated diagrams |
| codeExamples | CodeExample[] | Code snippets |
| tips | string[] | Pro tips/warnings |

### Entity: Diagram

Represents a visual diagram embedded in content.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Diagram identifier |
| filename | string | SVG filename |
| alt | string | Accessibility description |
| caption | string | Figure caption |
| width | string | Display width |

### Entity: CodeExample

Represents a runnable code snippet.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Example identifier |
| language | enum | python, yaml, bash, cpp |
| title | string | Code block title |
| source | string | Code content |
| runnable | boolean | Can student execute directly |
| prerequisites | string[] | Required setup |
| expectedOutput | string | What student should see |

### Entity: Exercise

Represents a hands-on exercise.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Exercise identifier |
| title | string | Exercise name |
| difficulty | enum | beginner, intermediate, advanced |
| objective | string | What student will accomplish |
| steps | string[] | Step-by-step instructions |
| successCriteria | string | How to verify completion |
| hints | string[] | Optional hints |
| solution | string | Reference solution (hidden) |

### Entity: Assessment

Represents a quiz/assessment question.

| Field | Type | Description |
|-------|------|-------------|
| id | string | Question identifier |
| type | enum | multiple_choice, short_answer, diagram |
| question | string | Question text |
| options | string[] | For multiple choice |
| correctAnswer | string | Expected answer |
| explanation | string | Why this is correct |
| relatedSection | reference | Source section |

## Chapter Mapping

| Chapter | User Story | Key Entities |
|---------|------------|--------------|
| Ch 1: Perception | US1 (P1) | perception pipeline, VSLAM, depth, mapping, localization |
| Ch 2: Isaac Sim | US2 (P2) | Isaac Sim scene, synthetic dataset, domain randomization |
| Ch 3: Isaac ROS | US3 (P3) | Isaac ROS graph, GPU nodes, point clouds |
| Ch 4: Nav2 | US4 (P4) | Nav2 stack, costmaps, humanoid footprint |
| Ch 5: Integration | All | End-to-end pipeline |

## Technical Entities

### Entity: PerceptionPipeline

The complete data flow from sensors to navigation.

| Stage | Input | Output | Isaac ROS Package |
|-------|-------|--------|-------------------|
| Sensor Input | Camera/LiDAR | Raw images/scans | ros_gz_bridge |
| Feature Extraction | Images | Features | isaac_ros_visual_slam |
| Depth Processing | Stereo/RGB-D | Point clouds | isaac_ros_depth_image_proc |
| Mapping | Point clouds | Occupancy grid | isaac_ros_nvblox |
| Localization | Map + sensors | Robot pose | isaac_ros_visual_slam |
| Navigation | Map + pose + goal | Velocity commands | nav2_bringup |

### Entity: SyntheticDataset

Generated training data from Isaac Sim.

| Component | Format | Description |
|-----------|--------|-------------|
| RGB Images | PNG | Color camera output |
| Depth Images | EXR | Floating-point depth |
| Segmentation | PNG | Semantic class masks |
| Annotations | JSON (COCO) | Bounding boxes, labels |
| Camera Info | JSON | Intrinsics, extrinsics |
| Ground Truth | JSON | Object poses, states |

### Entity: HumanoidFootprint

Navigation footprint for bipedal robot.

| Parameter | Type | Description |
|-----------|------|-------------|
| shape | polygon | Footprint vertices |
| padding | float | Safety margin (m) |
| inflation_radius | float | Costmap inflation (m) |
| turning_radius | float | Min turning radius (m) |
| max_velocity | float | Max linear velocity (m/s) |
| max_angular | float | Max angular velocity (rad/s) |

## State Transitions

### Isaac Sim Workflow States

```text
[Not Installed] → [Installed] → [Scene Created] → [Sensors Configured] → [Data Generating] → [Data Exported]
```

### VSLAM States

```text
[Initializing] → [Tracking] → [Lost] → [Relocalized] → [Tracking]
                     ↓
              [Loop Closed]
```

### Nav2 States

```text
[Idle] → [Goal Received] → [Planning] → [Executing] → [Goal Reached]
                               ↓              ↓
                          [No Path]     [Aborted]
```

## Validation Rules

### Content Validation

1. Every chapter MUST have at least 3 learning objectives
2. Every section MUST have at least 1 diagram OR 1 code example
3. Every code example MUST have runnable=true OR explicit prerequisites
4. Every exercise MUST have success criteria
5. Every assessment MUST have an explanation

### Technical Validation

1. All Python code MUST pass `python -m py_compile`
2. All YAML MUST be valid YAML 1.1
3. All launch files MUST have valid XML/Python syntax
4. All ROS topics MUST use standard message types
