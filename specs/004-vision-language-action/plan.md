# Implementation Plan: Module 4 — Vision-Language-Action (VLA)

**Branch**: `004-vision-language-action` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/004-vision-language-action/spec.md`

## Summary

This module teaches robotics students how Vision-Language-Action (VLA) models represent the convergence of Large Language Models and robotics. The implementation creates 5 chapters covering VLA fundamentals, Whisper voice-to-action pipelines, LLM cognitive planning for ROS 2, a capstone autonomous voice-controlled humanoid project, and future directions. Content follows Docusaurus MDX format with embedded diagrams, code snippets, and hands-on exercises.

## Technical Context

**Language/Version**: Python 3.10+ (ROS 2 Humble), MDX (Docusaurus 3.x)
**Primary Dependencies**: OpenAI Whisper, LLM APIs (OpenAI/Anthropic/Ollama), ROS 2 Humble, Nav2, MoveIt 2
**Storage**: N/A (static educational content)
**Testing**: `npm run build` validation, code example verification
**Target Platform**: Ubuntu 22.04 with GPU (recommended for Whisper, optional for small models)
**Project Type**: Educational module (Docusaurus static site)
**Performance Goals**: All code examples run successfully on target platform
**Constraints**: Beginner-friendly explanations, IEEE citations, <30 min setup per chapter
**Scale/Scope**: 5 chapters, ~12 diagrams, ~15 code examples, ~20 exercises

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Technical Accuracy | PASS | Content verified against Whisper docs, ROS 2 docs, LLM provider APIs |
| II. Educational Clarity | PASS | Learning objectives, prerequisites, progressive complexity planned |
| III. Reproducible Code | PASS | All code includes setup instructions, version pinning |
| IV. AI-Native Workflow | PASS | Following SDD: specify → plan → tasks → implement |
| V. Citation Integrity | PASS | IEEE format citations planned, official doc links, paper DOIs |
| VI. Modular Curriculum | PASS | Module 4 = "Vision-Language-Action (VLA)" in curriculum progression |
| VII. RAG Chatbot Integrity | N/A | Not applicable to module content |
| VIII. Build & Deploy Reliability | PASS | Docusaurus build validation planned |

**Gate Status**: ✅ PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/004-vision-language-action/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 output (VLA landscape research)
├── data-model.md        # Phase 1 output (content entities)
├── quickstart.md        # Phase 1 output (dev quickstart)
├── contracts/           # Phase 1 output (chapter outlines)
│   ├── chapter-01-outline.md
│   ├── chapter-02-outline.md
│   ├── chapter-03-outline.md
│   ├── chapter-04-outline.md
│   └── chapter-05-outline.md
├── checklists/
│   └── requirements.md  # Quality checklist
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
book/docs/module-04-vla/
├── _category_.json              # Docusaurus sidebar config
├── index.md                     # Module overview
├── chapter-01-vla-revolution.mdx # VLA Fundamentals
├── chapter-02-whisper.mdx       # Voice-to-Action with Whisper
├── chapter-03-llm-planning.mdx  # LLM Cognitive Planning
├── chapter-04-capstone.mdx      # Autonomous Voice-Controlled Humanoid
├── chapter-05-future.mdx        # Future Directions
└── assets/
    ├── diagrams/                # SVG/PNG diagrams
    │   ├── vla-architecture.svg
    │   ├── vla-vs-traditional.svg
    │   ├── rt2-architecture.svg
    │   ├── whisper-pipeline.svg
    │   ├── intent-parsing-flow.svg
    │   ├── voice-ros2-integration.svg
    │   ├── llm-planning-flow.svg
    │   ├── world-state-schema.svg
    │   ├── plan-executor-flow.svg
    │   ├── capstone-architecture.svg
    │   ├── capstone-sequence.svg
    │   └── vla-research-landscape.svg
    └── audio/                   # Sample audio files
        ├── sample_command_01.wav
        ├── sample_command_02.wav
        └── sample_command_03.wav

examples/vla/
├── whisper/
│   ├── requirements.txt
│   ├── verify_install.py
│   ├── transcribe_audio.py
│   ├── intent_parser_regex.py
│   ├── intent_parser_llm.py
│   └── sample_audio/
│       ├── go_to_kitchen.wav
│       ├── bring_cup.wav
│       └── clean_table.wav
├── llm_planning/
│   ├── requirements.txt
│   ├── task_planner_openai.py
│   ├── task_planner_ollama.py
│   ├── world_state.json
│   ├── robot_capabilities.json
│   └── plan_to_ros2.py
├── ros2_integration/
│   ├── launch/
│   │   ├── voice_pipeline.launch.py
│   │   ├── llm_planner.launch.py
│   │   └── capstone_demo.launch.py
│   └── config/
│       ├── whisper_params.yaml
│       ├── llm_params.yaml
│       └── capstone_params.yaml
└── capstone/
    ├── requirements.txt
    ├── voice_command_node.py
    ├── llm_planner_node.py
    ├── plan_executor_node.py
    ├── object_detector_node.py
    └── manipulation_node.py

examples/ros2_ws/src/
└── vla_control/
    ├── package.xml
    ├── setup.py
    ├── vla_control/
    │   ├── __init__.py
    │   ├── voice_command_node.py
    │   ├── llm_planner_node.py
    │   ├── plan_executor_node.py
    │   └── intent_msg.py
    └── msg/
        └── Intent.msg
```

**Structure Decision**: Educational module structure with Docusaurus chapters, supporting code examples in `examples/vla/`, ROS 2 package scaffolding for voice and planning integration, and sample audio files for students without microphones.

## Phases & Research-Concurrent Approach

### Phase 0: Research (Concurrent with Phase 1)

**Objective**: Gather authoritative information on VLA models, Whisper, and LLM-robotics integration.

| Research Task | Source | Output |
|--------------|--------|--------|
| VLA model landscape | Google DeepMind (RT-2), Google (PaLM-E), Berkeley (OpenVLA) papers | research.md §1 |
| Whisper architecture & usage | OpenAI Whisper GitHub, papers | research.md §2 |
| LLM prompting for robotics | SayCan, Code as Policies papers | research.md §3 |
| ROS 2 action patterns | ROS 2 documentation | research.md §4 |
| Safety considerations | Robotics safety literature | research.md §5 |

**Research Deliverable**: `specs/004-vision-language-action/research.md`

### Phase 1: Foundation (Architecture & Design)

**Objective**: Define content structure, chapter outlines, and code architecture.

| Task | Deliverable |
|------|-------------|
| Define content entities | data-model.md |
| Developer quickstart guide | quickstart.md |
| Chapter 1 outline (VLA fundamentals) | contracts/chapter-01-outline.md |
| Chapter 2 outline (Whisper) | contracts/chapter-02-outline.md |
| Chapter 3 outline (LLM Planning) | contracts/chapter-03-outline.md |
| Chapter 4 outline (Capstone) | contracts/chapter-04-outline.md |
| Chapter 5 outline (Future) | contracts/chapter-05-outline.md |

### Phase 2: Analysis (Task Generation)

**Objective**: Generate actionable tasks from plan.

| Task | Command |
|------|---------|
| Generate tasks.md | `/sp.tasks` |
| Review task dependencies | Manual review |
| Validate task coverage | Cross-reference spec requirements |

### Phase 3: Synthesis (Implementation)

**Objective**: Execute tasks to create deliverables.

| Phase | Focus | Deliverables |
|-------|-------|--------------|
| 3.1 Setup | Directory structure, scaffolding | _category_.json, index.md, package.xml |
| 3.2 Chapter 1 | VLA Revolution | chapter-01-vla-revolution.mdx, diagrams |
| 3.3 Chapter 2 | Whisper Pipeline | chapter-02-whisper.mdx, code examples |
| 3.4 Chapter 3 | LLM Planning | chapter-03-llm-planning.mdx, code examples |
| 3.5 Chapter 4 | Capstone | chapter-04-capstone.mdx, integration code |
| 3.6 Chapter 5 | Future | chapter-05-future.mdx |
| 3.7 Polish | Review, citations, validation | Final build verification |

## Architecture Decisions

### ADR-001: Whisper Model Size Selection

**Decision**: Default to Whisper "small" model with instructions for "base" (lower resources) and "medium" (higher accuracy)
**Rationale**: "small" provides good accuracy/performance balance. Students without GPUs can use "base" successfully. Clear documentation prevents frustration.
**Options Considered**:
| Option | Pros | Cons |
|--------|------|------|
| tiny | Fast, minimal resources | Poor accuracy for robotics commands |
| base | Good speed, low resources | Moderate accuracy |
| **small** (chosen) | Good accuracy, reasonable resources | Needs ~2GB VRAM |
| medium | High accuracy | Needs ~5GB VRAM |
| large | Best accuracy | Needs ~10GB VRAM, slow |

### ADR-002: LLM Provider Strategy

**Decision**: Support both cloud APIs (OpenAI, Anthropic) and local (Ollama) with Ollama as primary example
**Rationale**: Local Ollama ensures students without API keys can complete exercises. Cloud APIs shown for production context. Cost-free learning path is critical for education.
**Options Considered**:
| Option | Pros | Cons |
|--------|------|------|
| OpenAI only | Best quality, well-documented | Requires API key, costs money |
| Ollama only | Free, local, no API key | Lower quality for complex planning |
| **Both** (chosen) | Flexible, free local option | More code to maintain |
| Anthropic only | High quality | Requires API key, costs money |

### ADR-003: Intent Parsing Approach

**Decision**: Teach both rule-based (regex) and LLM-based intent parsing
**Rationale**: Rule-based demonstrates concepts simply; LLM-based shows production approach. Students understand tradeoffs between approaches.
**Options Considered**:
| Option | Pros | Cons |
|--------|------|------|
| Rule-based only | Simple, deterministic | Brittle, limited vocabulary |
| LLM-based only | Flexible, natural | Requires LLM call, latency |
| **Both** (chosen) | Educational, shows evolution | More content to write |

### ADR-004: Capstone Simulation Environment

**Decision**: Use Gazebo with Nav2 (from Module 3), simple pick-and-place manipulation
**Rationale**: Builds on Module 3 knowledge; avoids new simulation learning curve. MoveIt 2 used for basic manipulation only.
**Options Considered**:
| Option | Pros | Cons |
|--------|------|------|
| Isaac Sim | Advanced features, GPU-accelerated | Additional setup, heavy resources |
| **Gazebo + MoveIt 2** (chosen) | Familiar from Module 3, lighter | Less photorealistic |
| Unity | Cross-platform | New learning curve |

### ADR-005: ROS 2 Message Design for Intents

**Decision**: Create custom `Intent.msg` with action_type, target_object, target_location, constraints fields
**Rationale**: Clean separation between voice parsing and plan execution. Standardized message enables component reuse.
**Options Considered**:
| Option | Pros | Cons |
|--------|------|------|
| std_msgs/String | Simple | Requires parsing at receiver |
| JSON in String | Flexible | Still requires parsing |
| **Custom Intent.msg** (chosen) | Type-safe, documented | Requires message compilation |

## Validation Strategy

### Per-Chapter Validation

| Chapter | Validation Method | Success Criteria (from Spec) |
|---------|------------------|------------------------------|
| Ch 1 | Concept quiz | Student explains VLA paradigm, names 2 examples (SC-001) |
| Ch 2 | Whisper lab | Student transcribes commands within 15 min (SC-002) |
| Ch 3 | LLM planning lab | Student generates valid plans within 30 min (SC-004) |
| Ch 4 | Capstone demo | Student completes integration within 2 hours (SC-006) |
| Ch 5 | Reflection | Student traces full pipeline (SC-008) |

### Build Validation

```bash
# Validate Docusaurus build
cd book && npm run build

# Validate ROS 2 package
cd examples/ros2_ws && colcon build --packages-select vla_control

# Validate Python examples (syntax only)
python3 -m py_compile examples/vla/whisper/*.py
python3 -m py_compile examples/vla/llm_planning/*.py
python3 -m py_compile examples/vla/capstone/*.py

# Validate Whisper installation (runtime)
cd examples/vla/whisper && python3 verify_install.py
```

### Content Validation Checklist

- [ ] All code examples tested on Ubuntu 22.04
- [ ] Whisper examples work with CPU fallback
- [ ] LLM examples work with Ollama (free option)
- [ ] All diagrams render correctly in Docusaurus
- [ ] All citations link to valid URLs (IEEE format)
- [ ] All learning objectives map to exercises
- [ ] All prerequisites reference prior modules (1, 2, 3)
- [ ] Sample audio files included for students without microphones
- [ ] Capstone integrates with Module 3 Nav2 configuration

### Acceptance Criteria Mapping

| Spec Requirement | Plan Coverage |
|-----------------|---------------|
| FR-001: VLA paradigm + 2 examples | Chapter 1, diagrams |
| FR-002: Whisper installation tutorial | Chapter 2, verify_install.py |
| FR-003: Speech-to-text code examples | Chapter 2, transcribe_audio.py |
| FR-004: Intent parsing approaches | Chapter 2, intent_parser_*.py |
| FR-005: ROS 2 voice command nodes | Chapter 2, voice_command_node.py |
| FR-006: LLM prompting strategies | Chapter 3 |
| FR-007: Plan to ROS 2 conversion | Chapter 3, plan_to_ros2.py |
| FR-008: Failure recovery | Chapter 3, plan_executor_node.py |
| FR-009: Diagrams for each concept | 12 diagrams planned |
| FR-010: Capstone integration guide | Chapter 4 |
| FR-011: Safety considerations | Chapter 3 §6 |
| FR-012: Exercises per chapter | Each chapter section |
| FR-013: Assessment questions | End of each chapter |
| FR-014: Local + cloud LLM support | ADR-002, dual examples |
| FR-015: Beginner accessibility | Prerequisites, progressive complexity |

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Whisper API changes | Low | Medium | Pin to specific version, document alternatives |
| LLM output variability | Medium | Low | Provide expected output examples, handle gracefully |
| Complex capstone integration | Medium | High | Incremental integration, checkpoint testing |
| Resource constraints (GPU) | Medium | Medium | CPU fallback paths documented |
| Module 3 dependency issues | Low | High | Verify Module 3 examples work first |

## Complexity Tracking

> No constitution violations requiring justification.

## Next Steps

1. Run `/sp.tasks` to generate detailed task list
2. Execute Phase 0 research concurrently with Phase 1 foundation work
3. Begin implementation with Phase 3.1 Setup tasks
