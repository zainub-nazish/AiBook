# Tasks: Module 4 — Vision-Language-Action (VLA)

**Input**: Design documents from `specs/004-vision-language-action/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Not explicitly requested - educational content validation via build and manual review

**Organization**: Tasks are grouped by user story (chapter) to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- **Book content**: `book/docs/module-04-vla/`
- **Code examples**: `examples/vla/`
- **ROS 2 package**: `examples/ros2_ws/src/vla_control/`
- **Diagrams**: `book/docs/module-04-vla/assets/diagrams/`
- **Audio samples**: `book/docs/module-04-vla/assets/audio/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Module directory structure, Docusaurus configuration, and shared scaffolding

- [x] T001 Create module directory structure at book/docs/module-04-vla/
- [x] T002 Create _category_.json for Docusaurus sidebar in book/docs/module-04-vla/_category_.json
- [x] T003 Create index.md with module overview, prerequisites, learning objectives in book/docs/module-04-vla/index.md
- [x] T004 [P] Create assets/diagrams/ directory at book/docs/module-04-vla/assets/diagrams/
- [x] T005 [P] Create assets/audio/ directory at book/docs/module-04-vla/assets/audio/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Shared code infrastructure, ROS 2 package scaffolding, and example directories that ALL user stories depend on

**⚠️ CRITICAL**: No chapter content can reference code examples until this phase is complete

- [x] T006 Create examples/vla/ directory structure per plan.md
- [x] T007 [P] Create examples/vla/whisper/requirements.txt with Whisper dependencies
- [x] T008 [P] Create examples/vla/llm_planning/requirements.txt with LLM dependencies
- [x] T009 [P] Create examples/vla/capstone/requirements.txt with capstone dependencies
- [x] T010 Create ROS 2 package scaffolding in examples/ros2_ws/src/vla_control/package.xml
- [x] T011 Create setup.py for vla_control package in examples/ros2_ws/src/vla_control/setup.py
- [x] T012 Create __init__.py in examples/ros2_ws/src/vla_control/vla_control/__init__.py
- [x] T013 [P] Create Intent.msg custom message in examples/ros2_ws/src/vla_control/msg/Intent.msg
- [x] T014 [P] Create vla-architecture.svg shared diagram in book/docs/module-04-vla/assets/diagrams/vla-architecture.svg

**Checkpoint**: Foundation ready - chapter content can now reference code and diagrams

---

## Phase 3: User Story 1 - Understanding VLA Architecture (Priority: P1) 🎯 MVP

**Goal**: Student understands VLA paradigm, can explain multimodal convergence, and identifies real-world examples (RT-2, PaLM-E)

**Independent Test**: Student can draw and explain a VLA architecture diagram showing vision, language, and action modalities

**Success Criteria**: SC-001 - Student explains VLA paradigm and names 2 examples in under 3 minutes

### Diagrams for Chapter 1

- [x] T015 [P] [US1] Create vla-vs-traditional.svg comparing VLA to traditional pipelines in book/docs/module-04-vla/assets/diagrams/vla-vs-traditional.svg
- [x] T016 [P] [US1] Create rt2-architecture.svg showing RT-2 model structure in book/docs/module-04-vla/assets/diagrams/rt2-architecture.svg
- [x] T017 [P] [US1] Create vla-research-landscape.svg overview in book/docs/module-04-vla/assets/diagrams/vla-research-landscape.svg

### Chapter 1 Content

- [x] T018 [US1] Write Chapter 1 introduction: What are VLA models in book/docs/module-04-vla/chapter-01-vla-revolution.mdx
- [x] T019 [US1] Write section: The convergence of LLMs and robotics (FR-001)
- [x] T020 [US1] Write section: Real-world VLA systems (RT-2, PaLM-E, OpenVLA) with diagrams (FR-001)
- [x] T021 [US1] Write section: Why VLA changes robot programming
- [x] T022 [US1] Write section: From hand-coded to learned behaviors
- [x] T023 [US1] Add Chapter 1 exercises with clear success criteria (FR-012)
- [x] T024 [US1] Add Chapter 1 assessment questions (FR-013)
- [x] T025 [US1] Add Chapter 1 references with IEEE citations

**Checkpoint**: Chapter 1 complete - student can explain VLA paradigm and identify examples

---

## Phase 4: User Story 2 - Whisper Voice-to-Action Pipeline (Priority: P2)

**Goal**: Student builds voice-to-action pipeline using Whisper, converts voice commands to structured intents

**Independent Test**: Student runs Python script transcribing "Go to the kitchen" and extracts goal_location: "kitchen"

**Success Criteria**: SC-002 - Transcribe commands within 15 min; SC-003 - Extract intents within 20 min

### Diagrams for Chapter 2

- [ ] T026 [P] [US2] Create whisper-pipeline.svg showing audio→text→intent flow in book/docs/module-04-vla/assets/diagrams/whisper-pipeline.svg
- [ ] T027 [P] [US2] Create intent-parsing-flow.svg comparing regex vs LLM parsing in book/docs/module-04-vla/assets/diagrams/intent-parsing-flow.svg
- [ ] T028 [P] [US2] Create voice-ros2-integration.svg showing ROS 2 message flow in book/docs/module-04-vla/assets/diagrams/voice-ros2-integration.svg

### Code Examples for Chapter 2

- [ ] T029 [P] [US2] Create verify_install.py to validate Whisper setup in examples/vla/whisper/verify_install.py
- [ ] T030 [P] [US2] Create transcribe_audio.py for basic transcription in examples/vla/whisper/transcribe_audio.py
- [ ] T031 [P] [US2] Create intent_parser_regex.py for rule-based parsing (FR-004) in examples/vla/whisper/intent_parser_regex.py
- [ ] T032 [P] [US2] Create intent_parser_llm.py for LLM-based parsing (FR-004) in examples/vla/whisper/intent_parser_llm.py
- [ ] T033 [US2] Create voice_command_node.py ROS 2 node (FR-005) in examples/ros2_ws/src/vla_control/vla_control/voice_command_node.py

### Audio Samples for Chapter 2

- [ ] T034 [P] [US2] Create sample audio file go_to_kitchen.wav in examples/vla/whisper/sample_audio/go_to_kitchen.wav
- [ ] T035 [P] [US2] Create sample audio file bring_cup.wav in examples/vla/whisper/sample_audio/bring_cup.wav
- [ ] T036 [P] [US2] Create sample audio file clean_table.wav in examples/vla/whisper/sample_audio/clean_table.wav

### Chapter 2 Content

- [ ] T037 [US2] Write Chapter 2 introduction: OpenAI Whisper overview (FR-002) in book/docs/module-04-vla/chapter-02-whisper.mdx
- [ ] T038 [US2] Write section: Installation and setup with verify_install.py (FR-002)
- [ ] T039 [US2] Write section: Speech-to-text transcription with code examples (FR-003)
- [ ] T040 [US2] Write section: Intent parsing - rule-based approach (FR-004)
- [ ] T041 [US2] Write section: Intent parsing - LLM-based approach (FR-004)
- [ ] T042 [US2] Write section: ROS 2 integration for voice commands (FR-005)
- [ ] T043 [US2] Write section: Handling speech recognition errors (edge cases)
- [ ] T044 [US2] Add Chapter 2 exercises with clear success criteria (FR-012)
- [ ] T045 [US2] Add Chapter 2 assessment questions (FR-013)
- [ ] T046 [US2] Add Chapter 2 references with IEEE citations

**Checkpoint**: Chapter 2 complete - student can transcribe voice commands and extract structured intents

---

## Phase 5: User Story 3 - LLM Cognitive Planning for ROS 2 (Priority: P3)

**Goal**: Student prompts LLM to decompose tasks into ROS 2 action sequences, implements plan execution with failure recovery

**Independent Test**: Student prompts LLM with "Make me coffee", receives step-by-step plan, converts to ROS 2 actions

**Success Criteria**: SC-004 - Generate plans within 30 min; SC-005 - Convert to ROS 2 calls within 45 min

### Diagrams for Chapter 3

- [ ] T047 [P] [US3] Create llm-planning-flow.svg showing prompt→plan→actions in book/docs/module-04-vla/assets/diagrams/llm-planning-flow.svg
- [ ] T048 [P] [US3] Create world-state-schema.svg showing context representation in book/docs/module-04-vla/assets/diagrams/world-state-schema.svg
- [ ] T049 [P] [US3] Create plan-executor-flow.svg showing execution and recovery in book/docs/module-04-vla/assets/diagrams/plan-executor-flow.svg

### Code Examples for Chapter 3

- [ ] T050 [P] [US3] Create task_planner_ollama.py for local LLM planning (FR-014) in examples/vla/llm_planning/task_planner_ollama.py
- [ ] T051 [P] [US3] Create task_planner_openai.py for cloud API planning (FR-014) in examples/vla/llm_planning/task_planner_openai.py
- [ ] T052 [P] [US3] Create world_state.json example context in examples/vla/llm_planning/world_state.json
- [ ] T053 [P] [US3] Create robot_capabilities.json capability spec in examples/vla/llm_planning/robot_capabilities.json
- [ ] T054 [US3] Create plan_to_ros2.py converting plans to actions (FR-007) in examples/vla/llm_planning/plan_to_ros2.py
- [ ] T055 [US3] Create llm_planner_node.py ROS 2 node in examples/ros2_ws/src/vla_control/vla_control/llm_planner_node.py
- [ ] T056 [US3] Create plan_executor_node.py with failure recovery (FR-008) in examples/ros2_ws/src/vla_control/vla_control/plan_executor_node.py

### Chapter 3 Content

- [ ] T057 [US3] Write Chapter 3 introduction: LLMs as robot brains (FR-006) in book/docs/module-04-vla/chapter-03-llm-planning.mdx
- [ ] T058 [US3] Write section: Prompting strategies for task decomposition (FR-006)
- [ ] T059 [US3] Write section: World state representation for planning
- [ ] T060 [US3] Write section: Converting plans to ROS 2 actions (FR-007)
- [ ] T061 [US3] Write section: Plan monitoring and failure recovery (FR-008)
- [ ] T062 [US3] Write section: Safety constraints in LLM planning (FR-011)
- [ ] T063 [US3] Add Chapter 3 exercises with clear success criteria (FR-012)
- [ ] T064 [US3] Add Chapter 3 assessment questions (FR-013)
- [ ] T065 [US3] Add Chapter 3 references with IEEE citations

**Checkpoint**: Chapter 3 complete - student can generate LLM plans and execute via ROS 2

---

## Phase 6: User Story 4 - Capstone: Autonomous Voice-Controlled Humanoid (Priority: P4)

**Goal**: Student integrates voice→planning→navigation→detection→manipulation into complete autonomous system

**Independent Test**: Simulated humanoid receives "Bring me the red cup from the kitchen", completes full task autonomously

**Success Criteria**: SC-006 - Complete integration within 2 hours; SC-008 - Trace full pipeline

### Diagrams for Chapter 4

- [ ] T066 [P] [US4] Create capstone-architecture.svg system overview in book/docs/module-04-vla/assets/diagrams/capstone-architecture.svg
- [ ] T067 [P] [US4] Create capstone-sequence.svg showing message flow in book/docs/module-04-vla/assets/diagrams/capstone-sequence.svg

### Code Examples for Chapter 4

- [ ] T068 [P] [US4] Create voice_command_node.py capstone version in examples/vla/capstone/voice_command_node.py
- [ ] T069 [P] [US4] Create llm_planner_node.py capstone version in examples/vla/capstone/llm_planner_node.py
- [ ] T070 [P] [US4] Create plan_executor_node.py capstone version in examples/vla/capstone/plan_executor_node.py
- [ ] T071 [P] [US4] Create object_detector_node.py for object detection in examples/vla/capstone/object_detector_node.py
- [ ] T072 [P] [US4] Create manipulation_node.py for pick-and-place in examples/vla/capstone/manipulation_node.py

### Launch Files and Config for Chapter 4

- [ ] T073 [P] [US4] Create voice_pipeline.launch.py in examples/vla/ros2_integration/launch/voice_pipeline.launch.py
- [ ] T074 [P] [US4] Create llm_planner.launch.py in examples/vla/ros2_integration/launch/llm_planner.launch.py
- [ ] T075 [US4] Create capstone_demo.launch.py in examples/vla/ros2_integration/launch/capstone_demo.launch.py
- [ ] T076 [P] [US4] Create whisper_params.yaml in examples/vla/ros2_integration/config/whisper_params.yaml
- [ ] T077 [P] [US4] Create llm_params.yaml in examples/vla/ros2_integration/config/llm_params.yaml
- [ ] T078 [US4] Create capstone_params.yaml in examples/vla/ros2_integration/config/capstone_params.yaml

### Chapter 4 Content

- [ ] T079 [US4] Write Chapter 4 introduction: System architecture overview (FR-010) in book/docs/module-04-vla/chapter-04-capstone.mdx
- [ ] T080 [US4] Write section: Integrating voice, planning, and execution (FR-010)
- [ ] T081 [US4] Write section: Navigation integration with Nav2 (FR-010)
- [ ] T082 [US4] Write section: Object detection integration (FR-010)
- [ ] T083 [US4] Write section: Simple manipulation with pick-and-place (FR-010)
- [ ] T084 [US4] Write section: End-to-end demonstration walkthrough (FR-010)
- [ ] T085 [US4] Write section: Troubleshooting and common issues
- [ ] T086 [US4] Add Chapter 4 exercises with clear success criteria (FR-012)
- [ ] T087 [US4] Add Chapter 4 assessment questions (FR-013)
- [ ] T088 [US4] Add Chapter 4 references with IEEE citations

**Checkpoint**: Chapter 4 complete - student has working capstone demonstrating full VLA loop

---

## Phase 7: User Story 5 - Future Directions (Priority: P5)

**Goal**: Student understands VLA research landscape and resources for continued learning

**Independent Test**: Student can identify open research problems and next steps for deeper learning

**Success Criteria**: Foundational awareness of VLA frontier research

### Chapter 5 Content

- [ ] T089 [US5] Write Chapter 5 introduction: End-to-end VLA training overview in book/docs/module-04-vla/chapter-05-future.mdx
- [ ] T090 [US5] Write section: Multimodal foundation models for robotics
- [ ] T091 [US5] Write section: Sim-to-real transfer for VLA
- [ ] T092 [US5] Write section: Open research problems
- [ ] T093 [US5] Write section: Resources for continued learning
- [ ] T094 [US5] Add Chapter 5 exercises with clear success criteria (FR-012)
- [ ] T095 [US5] Add Chapter 5 assessment questions (FR-013)
- [ ] T096 [US5] Add Chapter 5 references with IEEE citations

**Checkpoint**: Chapter 5 complete - student has roadmap for continued VLA learning

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Quality validation, documentation completeness, and build verification

- [ ] T097 [P] Verify all diagrams render correctly in Docusaurus build
- [ ] T098 [P] Verify all code examples have syntax validation (py_compile)
- [ ] T099 [P] Verify ROS 2 package builds with colcon
- [ ] T100 Verify all IEEE citations link to valid URLs
- [ ] T101 Verify all prerequisites reference Modules 1, 2, 3
- [ ] T102 Verify CPU fallback paths documented for Whisper
- [ ] T103 Verify Ollama examples work for free LLM option (FR-014)
- [ ] T104 Run npm run build for Docusaurus validation
- [ ] T105 Update tasks.md to mark all tasks complete

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all chapter content
- **Phase 3-7 (User Stories)**: All depend on Phase 2 completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4 → P5)
- **Phase 8 (Polish)**: Depends on all user story phases being complete

### User Story Dependencies

- **US1 (Ch 1)**: Can start after Phase 2 - No dependencies on other stories
- **US2 (Ch 2)**: Can start after Phase 2 - May reference Ch 1 concepts but independently testable
- **US3 (Ch 3)**: Can start after Phase 2 - Builds on US2 concepts, independently testable
- **US4 (Ch 4)**: Soft dependency on US2, US3 code components - Integration story
- **US5 (Ch 5)**: Can start after Phase 2 - Minimal code dependencies

### Within Each User Story

1. Diagrams can be created in parallel [P]
2. Code examples can be created in parallel [P] where no dependencies exist
3. Chapter content depends on diagrams and code being available
4. Exercises and assessments come after main content
5. References come last

### Parallel Opportunities

**Phase 2 Parallel Tasks**:
- T007, T008, T009 (requirements.txt files)
- T013, T014 (Intent.msg, vla-architecture.svg)

**US1 (Ch 1) Parallel Tasks**:
- T015, T016, T017 (all diagrams)

**US2 (Ch 2) Parallel Tasks**:
- T026, T027, T028 (all diagrams)
- T029, T030, T031, T032 (code examples)
- T034, T035, T036 (audio samples)

**US3 (Ch 3) Parallel Tasks**:
- T047, T048, T049 (all diagrams)
- T050, T051, T052, T053 (code examples)

**US4 (Ch 4) Parallel Tasks**:
- T066, T067 (diagrams)
- T068, T069, T070, T071, T072 (node scripts)
- T073, T074, T076, T077 (launch files and configs)

---

## Parallel Example: User Story 2 (Chapter 2)

```bash
# Launch all diagrams for US2 together:
Task: T026 "Create whisper-pipeline.svg"
Task: T027 "Create intent-parsing-flow.svg"
Task: T028 "Create voice-ros2-integration.svg"

# Launch all code examples for US2 together:
Task: T029 "Create verify_install.py"
Task: T030 "Create transcribe_audio.py"
Task: T031 "Create intent_parser_regex.py"
Task: T032 "Create intent_parser_llm.py"

# Launch all audio samples together:
Task: T034 "Create go_to_kitchen.wav"
Task: T035 "Create bring_cup.wav"
Task: T036 "Create clean_table.wav"

# Then sequentially write chapter content (depends on above):
Task: T037-T046 (Chapter 2 sections)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Chapter 1: VLA Revolution)
4. **STOP and VALIDATE**: Test Chapter 1 renders in Docusaurus, diagrams display
5. Deploy/demo if ready - students can learn VLA concepts

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Ch 1) → Deploy (MVP - VLA concepts)
3. Add User Story 2 (Ch 2) → Deploy (Voice pipeline hands-on)
4. Add User Story 3 (Ch 3) → Deploy (LLM planning hands-on)
5. Add User Story 4 (Ch 4) → Deploy (Capstone project)
6. Add User Story 5 (Ch 5) → Deploy (Future directions)
7. Each chapter adds value without breaking previous chapters

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Chapter 1)
   - Developer B: User Story 2 (Chapter 2)
   - Developer C: User Story 3 (Chapter 3)
3. After US1-3 complete:
   - Developer A: User Story 4 (Capstone - integrates US2, US3)
   - Developer B: User Story 5 (Chapter 5)
4. All: Polish phase

---

## Task Summary

| Phase | Task Count | Parallel Opportunities |
|-------|------------|----------------------|
| Phase 1: Setup | 5 | 2 |
| Phase 2: Foundational | 9 | 5 |
| Phase 3: US1 (Ch 1) | 11 | 3 |
| Phase 4: US2 (Ch 2) | 21 | 11 |
| Phase 5: US3 (Ch 3) | 19 | 7 |
| Phase 6: US4 (Ch 4) | 23 | 12 |
| Phase 7: US5 (Ch 5) | 8 | 0 |
| Phase 8: Polish | 9 | 4 |
| **Total** | **105** | **44** |

### Tasks per User Story

- **US1 (Chapter 1)**: 11 tasks (T015-T025)
- **US2 (Chapter 2)**: 21 tasks (T026-T046)
- **US3 (Chapter 3)**: 19 tasks (T047-T065)
- **US4 (Chapter 4)**: 23 tasks (T066-T088)
- **US5 (Chapter 5)**: 8 tasks (T089-T096)

### MVP Scope

**Minimum Viable Module**: Phase 1 + Phase 2 + Phase 3 (US1)
- Total: 25 tasks
- Delivers: VLA concepts chapter with diagrams
- Student outcome: Can explain VLA paradigm (SC-001)

---

## Notes

- [P] tasks = different files, no dependencies
- [USn] label maps task to specific user story/chapter for traceability
- Each user story = one chapter that's independently completable
- Commit after each task or logical group
- Stop at any checkpoint to validate chapter independently
- Audio samples can be placeholder files (text describing what should be recorded)
- Avoid: vague tasks, same file conflicts, cross-chapter dependencies that break independence
