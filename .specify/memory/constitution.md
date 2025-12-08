<!--
SYNC IMPACT REPORT
==================
Version change: 0.0.0 → 1.0.0
Bump rationale: MAJOR - Initial constitution creation

Added Principles:
- I. Technical Accuracy
- II. Educational Clarity
- III. Reproducible Code
- IV. AI-Native Workflow
- V. Citation Integrity
- VI. Modular Curriculum
- VII. RAG Chatbot Integrity
- VIII. Build & Deploy Reliability

Added Sections:
- Technology Stack (Section 2)
- Development Workflow (Section 3)
- Governance

Removed Sections: None (initial creation)

Templates Requiring Updates:
- ✅ plan-template.md - Compatible (Constitution Check section present)
- ✅ spec-template.md - Compatible (Requirements/Success Criteria aligned)
- ✅ tasks-template.md - Compatible (Phase structure supports curriculum modules)

Deferred Items: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Technical Accuracy

All technical content MUST be verified against official documentation from ROS 2, Gazebo, Unity, NVIDIA Isaac, and other referenced frameworks.

- Claims about APIs, configurations, and behaviors MUST cite official sources
- Code examples MUST be tested against current stable versions
- Deprecated features MUST be flagged with migration guidance
- Version-specific behaviors MUST explicitly state the applicable version

**Rationale**: Robotics students rely on textbook accuracy; incorrect information leads to debugging dead-ends and erodes trust in educational materials.

### II. Educational Clarity

Content MUST be written for robotics students with varying backgrounds, prioritizing comprehension over brevity.

- Complex concepts MUST include visual diagrams or illustrations
- New terminology MUST be defined on first use
- Prerequisites for each chapter MUST be explicitly stated
- Learning objectives MUST appear at the start of each chapter
- Progressive complexity: foundational concepts before advanced topics

**Rationale**: The goal is education, not documentation. Students need scaffolded learning paths that build understanding incrementally.

### III. Reproducible Code

All code examples MUST be complete, runnable, and reproducible.

- ROS 2 code MUST use Python and follow REP (ROS Enhancement Proposal) guidelines
- Gazebo/Unity simulation code MUST include complete scene/world files
- URDF/SDF files MUST be valid and complete
- NVIDIA Isaac examples MUST specify required hardware and software versions
- Environment setup instructions MUST be provided for all code
- Code MUST NOT rely on undocumented assumptions or hidden state

**Rationale**: Students learn by doing. Non-functional code examples destroy learning momentum and create frustration.

### IV. AI-Native Workflow

Development MUST leverage Claude Code and Spec-Kit Plus for consistent, traceable progress.

- All features MUST follow the SDD workflow: specify → plan → tasks → implement
- Prompt History Records (PHRs) MUST be created for all significant interactions
- Architectural decisions MUST be documented in ADRs when they meet significance criteria
- Changes MUST be small, testable, and reference code precisely

**Rationale**: AI-assisted development requires discipline; traceable workflows ensure reproducibility and enable learning from past decisions.

### V. Citation Integrity

All claims MUST be verifiable and properly cited using IEEE/ACM citation style.

- External sources MUST use IEEE numeric citation format [1], [2], etc.
- Official documentation MUST link directly to stable/versioned URLs
- Research papers MUST include DOI when available
- Claims without citations MUST be marked as needing verification
- No fabricated or hallucinated references permitted

**Rationale**: Academic integrity is non-negotiable. Students and instructors must be able to verify and extend the material.

### VI. Modular Curriculum

Content MUST be organized into a structured, progressive curriculum with minimum 12 chapters.

Required module progression:
1. **ROS 2 Fundamentals** - Nodes, topics, services, actions
2. **Simulation Foundations** - Gazebo and/or Unity integration
3. **Robot Description** - URDF/SDF modeling
4. **Perception** - Sensors, computer vision, point clouds
5. **Navigation** - SLAM, path planning, Nav2
6. **Manipulation** - MoveIt 2, motion planning
7. **NVIDIA Isaac** - Isaac Sim, Isaac ROS, hardware acceleration
8. **Vision-Language-Action (VLA)** - Foundation models for robotics
9. **Humanoid Robotics** - Bipedal locomotion, whole-body control
10. **System Integration** - Multi-robot systems, ROS 2 networking
11. **Deployment** - Real hardware, safety considerations
12. **Capstone Project** - Voice → plan → navigate → identify → manipulate

Each chapter MUST include:
- Learning objectives
- Prerequisites
- Theory with diagrams
- Hands-on exercises
- Code examples
- Assessment questions

**Rationale**: Structured progression ensures students build competencies systematically; the capstone validates integrated understanding.

### VII. RAG Chatbot Integrity

The integrated chatbot MUST answer exclusively from book content and user-selected text.

- Responses MUST be grounded in embedded book content
- Hallucinated or fabricated answers are PROHIBITED
- When book content is insufficient, chatbot MUST acknowledge limitations
- User-selected text context MUST be respected and incorporated
- Chat history MUST be persisted in Neon Postgres
- Vector embeddings MUST use Qdrant Cloud

Technical requirements:
- Embedding model for semantic search
- Qdrant vector database for content retrieval
- Neon Postgres for conversation history
- FastAPI backend for chatbot API
- Docusaurus UI integration for frontend

**Rationale**: Educational chatbots must be trustworthy; hallucinations undermine the learning process and spread misinformation.

### VIII. Build & Deploy Reliability

All deliverables MUST build and deploy without errors.

- Docusaurus book MUST build successfully with `npm run build`
- GitHub Pages deployment MUST complete without failures
- Chatbot backend MUST pass health checks on deployment
- All dependencies MUST be pinned to specific versions
- CI/CD pipelines MUST validate builds on every commit
- Broken builds MUST block deployment

**Rationale**: Deployment reliability ensures students and instructors can access materials; broken deployments waste everyone's time.

## Technology Stack

### Book Platform
- **Framework**: Docusaurus
- **Hosting**: GitHub Pages
- **Format**: MDX with React components for interactivity
- **Assets**: Diagrams, URDF visualizations, embedded simulations where feasible

### RAG Chatbot Stack
- **Agent Framework**: OpenAI Agents SDK
- **Backend**: FastAPI (Python)
- **Vector Database**: Qdrant Cloud
- **Relational Database**: Neon Postgres (chat history)
- **Embedding Model**: To be specified (e.g., OpenAI text-embedding-3-small)
- **UI**: Integrated into Docusaurus frontend

### Robotics Stack (Code Examples)
- **ROS Version**: ROS 2 (Humble or later)
- **Simulation**: Gazebo Fortress/Harmonic, Unity with ROS 2 bridge
- **NVIDIA**: Isaac Sim, Isaac ROS
- **Languages**: Python (primary), C++ (where required by ROS 2)

## Development Workflow

### Content Creation
1. **Draft** - Write chapter content with placeholders for diagrams/code
2. **Verify** - Validate all technical claims against official documentation
3. **Implement** - Create and test all code examples
4. **Review** - Technical review for accuracy and clarity
5. **Polish** - Add diagrams, refine prose, format citations

### Code Quality
- All code examples MUST be linted (ruff for Python)
- All code examples SHOULD have accompanying tests where feasible
- Code MUST follow framework-specific style guides (ROS 2 Python style)

### Chatbot Development
- Follow TDD for chatbot backend features
- Integration tests MUST verify RAG responses match expected content
- Load testing MUST validate concurrent user support

### Version Control
- Feature branches for all changes
- Pull requests require review before merge
- Main branch MUST always be deployable
- Semantic versioning for releases

## Governance

This constitution is the authoritative source for project standards. All contributors and AI assistants MUST adhere to these principles.

**Amendment Process**:
1. Propose changes via pull request to constitution.md
2. Document rationale and impact analysis
3. Review by project maintainers
4. Update dependent templates if principles change
5. Increment version according to semantic versioning

**Compliance**:
- All pull requests MUST be reviewed against constitution principles
- AI-generated content MUST cite this constitution in commit messages
- Violations MUST be corrected before merge

**Version Policy**:
- MAJOR: Backward-incompatible principle changes or removals
- MINOR: New principles or significant expansions
- PATCH: Clarifications, typos, non-semantic refinements

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08
