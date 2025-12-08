# Tasks: Module 1 — The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-fundamentals/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

**Tests**: Tests are NOT explicitly requested for this documentation feature. Validation is via Docusaurus build and code example execution.

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3)
- Include exact file paths in descriptions

## Path Conventions

- **Book content**: `book/docs/module-01-ros2/`
- **Code examples**: `examples/ros2_ws/src/humanoid_basics/`
- **Static assets**: `book/static/`

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize Docusaurus project and book structure

- [ ] T001 Initialize Docusaurus project in book/ directory with `npx create-docusaurus@latest book classic`
- [ ] T002 Configure docusaurus.config.js with book title, tagline, and GitHub Pages settings in book/docusaurus.config.js
- [ ] T003 [P] Configure Prism languages (python, bash, xml, yaml) in book/docusaurus.config.js
- [ ] T004 [P] Create module directory structure at book/docs/module-01-ros2/
- [ ] T005 [P] Create _category_.json for Module 1 at book/docs/module-01-ros2/_category_.json
- [ ] T006 [P] Create module overview page at book/docs/module-01-ros2/index.md
- [ ] T007 [P] Create book introduction at book/docs/intro.md
- [ ] T008 Configure sidebars.js with Module 1 structure in book/sidebars.js

**Checkpoint**: `npm run build` succeeds with placeholder content

---

## Phase 2: Foundational (ROS 2 Package Setup)

**Purpose**: Create the ROS 2 package that holds all code examples (shared across all chapters)

**CRITICAL**: This phase MUST complete before chapter content can reference code examples

- [ ] T009 Create ROS 2 workspace structure at examples/ros2_ws/src/
- [ ] T010 Create humanoid_basics package with `ros2 pkg create` at examples/ros2_ws/src/humanoid_basics/
- [ ] T011 Configure package.xml with dependencies (rclpy, std_msgs, sensor_msgs) at examples/ros2_ws/src/humanoid_basics/package.xml
- [ ] T012 Configure setup.py with entry points at examples/ros2_ws/src/humanoid_basics/setup.py
- [ ] T013 [P] Create __init__.py at examples/ros2_ws/src/humanoid_basics/humanoid_basics/__init__.py
- [ ] T014 [P] Create resource marker file at examples/ros2_ws/src/humanoid_basics/resource/humanoid_basics
- [ ] T015 [P] Create assets directories at book/docs/module-01-ros2/assets/diagrams/ and book/docs/module-01-ros2/assets/code/

**Checkpoint**: Package builds with `colcon build` in ROS 2 environment

---

## Phase 3: User Story 1 - ROS 2 Communication Basics (Priority: P1) MVP

**Goal**: Students learn ROS 2 nodes, topics, and services through Chapter 1

**Independent Test**: Student can create publisher/subscriber pair and verify with CLI tools

### Code Examples for US1

- [ ] T016 [P] [US1] Create publisher_node.py at examples/ros2_ws/src/humanoid_basics/humanoid_basics/publisher_node.py
- [ ] T017 [P] [US1] Create subscriber_node.py at examples/ros2_ws/src/humanoid_basics/humanoid_basics/subscriber_node.py
- [ ] T018 [P] [US1] Create service_server.py at examples/ros2_ws/src/humanoid_basics/humanoid_basics/service_server.py
- [ ] T019 [P] [US1] Create service_client.py at examples/ros2_ws/src/humanoid_basics/humanoid_basics/service_client.py
- [ ] T020 [US1] Update setup.py entry points with chapter 1 nodes at examples/ros2_ws/src/humanoid_basics/setup.py

### Chapter Content for US1

- [ ] T021 [US1] Write Chapter 1 frontmatter and learning objectives at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T022 [US1] Write "Introduction to ROS 2" section (h2) at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T023 [US1] Write "What is a Node?" section with diagram at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T024 [US1] Write "Topics: Publish/Subscribe Communication" section at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T025 [US1] Write "Creating a Publisher Node" subsection with code example at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T026 [US1] Write "Creating a Subscriber Node" subsection with code example at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T027 [US1] Write "Services: Request/Response Communication" section at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T028 [US1] Write service server and client subsections with code examples at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T029 [US1] Write "ROS 2 Command-Line Tools" section at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T030 [US1] Write Chapter 1 exercises (4 exercises) at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T031 [US1] Write Chapter 1 assessment questions and summary at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T032 [US1] Add Chapter 1 citations in IEEE format at book/docs/module-01-ros2/chapter-01-basics.mdx

### Diagrams for US1

- [ ] T033 [P] [US1] Create ROS 2 architecture diagram (Mermaid) at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T034 [P] [US1] Create pub/sub flow diagram (Mermaid) at book/docs/module-01-ros2/chapter-01-basics.mdx
- [ ] T035 [P] [US1] Create service flow diagram (Mermaid) at book/docs/module-01-ros2/chapter-01-basics.mdx

**Checkpoint**: Chapter 1 builds in Docusaurus, all code examples run on ROS 2 Humble

---

## Phase 4: User Story 2 - Python Control with rclpy (Priority: P2)

**Goal**: Students learn ROS 2 Python packages and joint control through Chapter 2

**Independent Test**: Student can create a Python package with control node publishing JointState messages

### Code Examples for US2

- [ ] T036 [P] [US2] Create joint_commander.py at examples/ros2_ws/src/humanoid_basics/humanoid_basics/joint_commander.py
- [ ] T037 [P] [US2] Create action_interface.py at examples/ros2_ws/src/humanoid_basics/humanoid_basics/action_interface.py
- [ ] T038 [US2] Update setup.py entry points with chapter 2 nodes at examples/ros2_ws/src/humanoid_basics/setup.py

### Chapter Content for US2

- [ ] T039 [US2] Write Chapter 2 frontmatter and learning objectives at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T040 [US2] Write "ROS 2 Python Package Structure" section at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T041 [US2] Write "The package.xml File" subsection with complete example at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T042 [US2] Write "The setup.py File" subsection with complete example at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T043 [US2] Write "Building Control Nodes" section with diagram at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T044 [US2] Write "Publishing Joint Commands" section with joint_commander.py example at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T045 [US2] Write "Bridging AI Agents to ROS 2" section with action_interface.py example at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T046 [US2] Write "Node Lifecycle and Shutdown" section at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T047 [US2] Write "Node Parameters" section at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T048 [US2] Write Chapter 2 exercises (4 exercises) at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T049 [US2] Write Chapter 2 assessment questions and summary at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T050 [US2] Add Chapter 2 citations in IEEE format at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx

### Diagrams for US2

- [ ] T051 [P] [US2] Create package structure diagram at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T052 [P] [US2] Create control node architecture diagram at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx
- [ ] T053 [P] [US2] Create AI agent interface diagram at book/docs/module-01-ros2/chapter-02-rclpy-control.mdx

**Checkpoint**: Chapter 2 builds in Docusaurus, joint_commander and action_interface run on ROS 2 Humble

---

## Phase 5: User Story 3 - URDF for Humanoids (Priority: P3)

**Goal**: Students learn URDF structure and create a humanoid robot model through Chapter 3

**Independent Test**: Student can create valid URDF and visualize in RViz2

### Code Examples for US3

- [ ] T054 [P] [US3] Create simple_link.urdf at examples/ros2_ws/src/humanoid_basics/urdf/simple_link.urdf
- [ ] T055 [P] [US3] Create two_links.urdf at examples/ros2_ws/src/humanoid_basics/urdf/two_links.urdf
- [ ] T056 [P] [US3] Create humanoid_simple.urdf (complete humanoid) at examples/ros2_ws/src/humanoid_basics/urdf/humanoid_simple.urdf
- [ ] T057 [P] [US3] Copy humanoid_simple.urdf to book/static/urdf/humanoid_simple.urdf
- [ ] T058 [US3] Create view_humanoid.launch.py at examples/ros2_ws/src/humanoid_basics/launch/view_humanoid.launch.py
- [ ] T059 [US3] Create RViz2 config file at examples/ros2_ws/src/humanoid_basics/rviz/view_robot.rviz
- [ ] T060 [US3] Update setup.py to include urdf, launch, and rviz data files at examples/ros2_ws/src/humanoid_basics/setup.py

### Chapter Content for US3

- [ ] T061 [US3] Write Chapter 3 frontmatter and learning objectives at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T062 [US3] Write "Introduction to URDF" section at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T063 [US3] Write "Defining Links" section at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T064 [US3] Write "Visual Elements" subsection with code example at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T065 [US3] Write "Collision Elements" subsection with code example at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T066 [US3] Write "Inertial Properties" subsection with code example and inertia table at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T067 [US3] Write "Defining Joints" section at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T068 [US3] Write "Joint Types" subsection with joint types table at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T069 [US3] Write "Joint Limits and Dynamics" subsection with code example at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T070 [US3] Write "Building a Humanoid Model" section with complete URDF at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T071 [US3] Write "Visualizing in RViz2" section with launch file example at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T072 [US3] Write "Validating URDF Files" section with error table at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T073 [US3] Write Chapter 3 exercises (5 exercises) at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T074 [US3] Write Chapter 3 assessment questions and summary at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T075 [US3] Add Chapter 3 citations in IEEE format at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx

### Diagrams for US3

- [ ] T076 [P] [US3] Create kinematic tree diagram at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx
- [ ] T077 [P] [US3] Create humanoid structure diagram at book/docs/module-01-ros2/chapter-03-urdf-humanoids.mdx

**Checkpoint**: Chapter 3 builds in Docusaurus, humanoid_simple.urdf passes check_urdf, visualizes in RViz2

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, consistency, and deployment preparation

- [ ] T078 [P] Run `npm run build` and fix any Docusaurus errors in book/
- [ ] T079 [P] Run `colcon build` and fix any ROS 2 package errors in examples/ros2_ws/
- [ ] T080 [P] Validate all URDF files with `check_urdf` in examples/ros2_ws/src/humanoid_basics/urdf/
- [ ] T081 [P] Run markdownlint on all chapter files in book/docs/module-01-ros2/
- [ ] T082 Verify all internal links work in built site
- [ ] T083 Verify all code examples have "How to Run" tabs
- [ ] T084 Review learning objectives match chapter content
- [ ] T085 [P] Create GitHub Actions workflow for Docusaurus build at .github/workflows/build.yml
- [ ] T086 [P] Create GitHub Actions workflow for GitHub Pages deploy at .github/workflows/deploy.yml

**Checkpoint**: Module 1 complete, builds pass, ready for deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 completion — BLOCKS all user stories
- **Phase 3 (US1/Chapter 1)**: Depends on Phase 2 completion
- **Phase 4 (US2/Chapter 2)**: Depends on Phase 2 completion (can parallel with Phase 3)
- **Phase 5 (US3/Chapter 3)**: Depends on Phase 2 completion (can parallel with Phase 3 & 4)
- **Phase 6 (Polish)**: Depends on Phases 3, 4, 5 completion

### User Story Dependencies

- **US1 (Chapter 1)**: No dependencies on other chapters — can complete as MVP
- **US2 (Chapter 2)**: No blocking dependency, but content references Chapter 1 concepts
- **US3 (Chapter 3)**: No blocking dependency, but content references Chapters 1 & 2 concepts

### Within Each User Story

- Code examples FIRST, then chapter content that references them
- Core sections before exercises
- Exercises before assessment
- Citations added last

### Parallel Opportunities

**Phase 1** (all [P] tasks can run in parallel):
```
T003, T004, T005, T006, T007 — different files, no dependencies
```

**Phase 2** (after T012):
```
T013, T014, T015 — different files, no dependencies
```

**Phase 3** (code examples):
```
T016, T017, T018, T019 — different Python files, no dependencies
```

**Phase 3** (diagrams):
```
T033, T034, T035 — different diagram types, no dependencies
```

**Chapters can be written in parallel**:
```
Phase 3 (US1), Phase 4 (US2), Phase 5 (US3) — different chapter files
```

---

## Implementation Strategy

### MVP First (Chapter 1 Only)

1. Complete Phase 1: Setup (Docusaurus initialized)
2. Complete Phase 2: Foundational (ROS 2 package ready)
3. Complete Phase 3: User Story 1 (Chapter 1 complete)
4. **STOP and VALIDATE**: Build site, test code examples
5. Deploy/demo if ready — students can learn ROS 2 basics

### Incremental Delivery

1. Setup + Foundational → Project skeleton ready
2. Add Chapter 1 (US1) → Test independently → Deploy (MVP!)
3. Add Chapter 2 (US2) → Test independently → Deploy
4. Add Chapter 3 (US3) → Test independently → Deploy
5. Polish phase → Final validation → Production deploy

### Parallel Team Strategy

With multiple writers:
1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Writer A: Chapter 1 (US1) content + code
   - Writer B: Chapter 2 (US2) content + code
   - Writer C: Chapter 3 (US3) content + code
3. Each chapter completes independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific chapter for traceability
- Each chapter should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate chapter independently
- Code examples must be complete and runnable — no placeholder code
- All citations must link to official ROS 2 Humble documentation
