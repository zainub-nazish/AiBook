# Implementation Plan: Module 1 — The Robotic Nervous System (ROS 2)

**Branch**: `001-ros2-fundamentals` | **Date**: 2025-12-08 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ros2-fundamentals/spec.md`

## Summary

Create the foundational ROS 2 module (3 chapters) for the Physical AI & Humanoid Robotics textbook. This module teaches ROS 2 communication primitives (nodes, topics, services), Python-based robot control with rclpy, and humanoid robot description with URDF. All content will be Docusaurus-compatible Markdown with runnable code examples validated against ROS 2 Humble.

## Technical Context

**Language/Version**: Markdown (MDX), Python 3.10+ (code examples), XML (URDF)
**Primary Dependencies**: Docusaurus 3.x, ROS 2 Humble, rclpy, robot_state_publisher, RViz2
**Storage**: Git (content), GitHub Pages (deployment)
**Testing**: Docusaurus build validation, `check_urdf` for URDF, Python syntax check, RAG extraction tests
**Target Platform**: Web (Docusaurus static site), ROS 2 on Ubuntu 22.04 (code examples)
**Project Type**: Documentation/Textbook (Docusaurus static site)
**Performance Goals**: Book builds in <2 minutes, all pages load in <3 seconds
**Constraints**: All code must run on ROS 2 Humble+, no fictional APIs, IEEE citation format
**Scale/Scope**: 3 chapters, ~50-70 pages total, 15+ code examples, 1 complete URDF

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status | Notes |
|-----------|-------------|--------|-------|
| I. Technical Accuracy | Verify against official ROS 2 docs | ✅ PASS | All examples target Humble, will cite docs.ros.org |
| II. Educational Clarity | Learning objectives, prerequisites, diagrams | ✅ PASS | Each chapter has objectives, progressive complexity |
| III. Reproducible Code | Complete, runnable Python/URDF | ✅ PASS | Full imports, no hidden state, validated |
| IV. AI-Native Workflow | SDD workflow with PHRs | ✅ PASS | Following /sp.plan → /sp.tasks flow |
| V. Citation Integrity | IEEE format, no fabricated refs | ✅ PASS | Will link to docs.ros.org versioned URLs |
| VI. Modular Curriculum | Part of 12+ chapter structure | ✅ PASS | Module 1 of 12, covers ROS 2 Fundamentals |
| VII. RAG Chatbot Integrity | Content extractable for RAG | ✅ PASS | Clean Markdown, semantic headings |
| VIII. Build & Deploy Reliability | Docusaurus builds without errors | ✅ PASS | CI will validate on every commit |

**Gate Status**: ALL PASS — Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-fundamentals/
├── plan.md              # This file
├── research.md          # Phase 0: Docusaurus best practices, ROS 2 Humble APIs
├── data-model.md        # Phase 1: Chapter/content model
├── quickstart.md        # Phase 1: How to build and preview locally
├── contracts/           # Phase 1: Chapter outlines (content contracts)
│   ├── chapter-01-outline.md
│   ├── chapter-02-outline.md
│   └── chapter-03-outline.md
└── tasks.md             # Phase 2: Implementation tasks (/sp.tasks)
```

### Source Code (Book Content)

```text
book/
├── docusaurus.config.js     # Site configuration
├── sidebars.js              # Navigation structure
├── docs/
│   ├── intro.md             # Book introduction
│   └── module-01-ros2/
│       ├── _category_.json  # Module metadata
│       ├── index.md         # Module overview
│       ├── chapter-01-basics.mdx
│       ├── chapter-02-rclpy-control.mdx
│       ├── chapter-03-urdf-humanoids.mdx
│       └── assets/
│           ├── diagrams/    # Architecture diagrams
│           └── code/        # Downloadable code examples
├── static/
│   └── urdf/
│       └── humanoid_simple.urdf
└── src/
    └── components/
        └── CodeExample/     # Reusable code block component
```

### Code Examples (ROS 2 Workspace)

```text
examples/
└── ros2_ws/
    └── src/
        └── humanoid_basics/
            ├── package.xml
            ├── setup.py
            ├── humanoid_basics/
            │   ├── __init__.py
            │   ├── publisher_node.py
            │   ├── subscriber_node.py
            │   ├── service_server.py
            │   ├── service_client.py
            │   └── joint_commander.py
            └── urdf/
                └── humanoid_simple.urdf
```

**Structure Decision**: Docusaurus documentation site with separate ROS 2 workspace for runnable examples. Code is both embedded in chapters (for reading) and available as complete files (for execution).

## Book Architecture

### Docusaurus Structure

```text
book/
├── docusaurus.config.js    # Site config, theme, plugins
├── sidebars.js             # Auto-generated from folder structure
├── docs/                   # All chapter content
│   ├── intro.md            # Landing page
│   └── module-XX-name/     # Each module is a folder
│       ├── _category_.json # Module title, position, collapsible
│       ├── index.md        # Module overview
│       └── chapter-XX.mdx  # Individual chapters
├── blog/                   # (optional) Updates, errata
├── src/
│   ├── components/         # Custom React components
│   ├── css/                # Custom styles
│   └── pages/              # Custom pages (about, contact)
└── static/                 # Images, downloadable files, URDF
```

### Sidebar Configuration (sidebars.js)

```javascript
module.exports = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: { type: 'doc', id: 'module-01-ros2/index' },
      items: [
        'module-01-ros2/chapter-01-basics',
        'module-01-ros2/chapter-02-rclpy-control',
        'module-01-ros2/chapter-03-urdf-humanoids',
      ],
    },
    // Additional modules follow same pattern...
  ],
};
```

### Chapter Organization (All 12 Modules)

| Module | Title | Chapters | Prerequisites |
|--------|-------|----------|---------------|
| 1 | ROS 2 Fundamentals | 3 | Python basics |
| 2 | Simulation Foundations | 2-3 | Module 1 |
| 3 | Robot Description | 2 | Module 1 |
| 4 | Perception | 3 | Modules 1-2 |
| 5 | Navigation | 3 | Modules 1-4 |
| 6 | Manipulation | 3 | Modules 1-3 |
| 7 | NVIDIA Isaac | 3 | Modules 1-2 |
| 8 | Vision-Language-Action | 2-3 | Modules 1-7 |
| 9 | Humanoid Robotics | 3 | Modules 1-6 |
| 10 | System Integration | 2 | Modules 1-9 |
| 11 | Deployment | 2 | Modules 1-10 |
| 12 | Capstone Project | 1-2 | All modules |

### Code Example Format

All code examples follow this structure:

```mdx
## Creating a Publisher Node

<Tabs>
  <TabItem value="full" label="Full Code" default>

```python title="humanoid_basics/publisher_node.py"
#!/usr/bin/env python3
"""ROS 2 publisher node for humanoid joint commands."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(String, 'joint_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'wave'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

  </TabItem>
  <TabItem value="run" label="How to Run">

```bash
# Source your ROS 2 workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run the publisher
ros2 run humanoid_basics publisher_node
```

  </TabItem>
</Tabs>
```

### RAG Chatbot Integration Strategy

**Placement**: Chat widget in bottom-right corner of every page, expandable to full panel.

**Integration Points**:
1. **Content Extraction**: Build script extracts all `.md`/`.mdx` content
2. **Chunking**: Split by heading (h2/h3) for semantic chunks
3. **Embedding**: OpenAI text-embedding-3-small → Qdrant Cloud
4. **Selection Context**: User can highlight text → "Ask about this"
5. **Chat History**: Persisted per-session in Neon Postgres

**Architecture**:
```text
[Docusaurus Frontend] → [FastAPI Backend] → [OpenAI Agents SDK]
                                         ↓
                              [Qdrant Cloud] (vector search)
                              [Neon Postgres] (chat history)
```

## Writing Workflow

### Phase Flow: Draft → Build → Test → Refine → Publish

```text
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│  DRAFT  │ → │  BUILD  │ → │  TEST   │ → │ REFINE  │ → │ PUBLISH │
└─────────┘    └─────────┘    └─────────┘    └─────────┘    └─────────┘
     │              │              │              │              │
     ▼              ▼              ▼              ▼              ▼
 Write content   npm run      Validate       Address       Deploy to
 in Markdown     build        code/links     issues        GitHub Pages
```

### Detailed Steps

**1. DRAFT** (Content Creation)
- Write chapter content with learning objectives
- Include placeholder code blocks
- Add diagram descriptions (actual diagrams in Refine)
- Mark citations as `[REF: topic]` for later lookup

**2. BUILD** (Local Validation)
- Run `npm run build` to ensure Docusaurus compiles
- Check for MDX syntax errors
- Verify all imports resolve

**3. TEST** (Quality Validation)
- Run code examples in ROS 2 environment
- Validate URDF with `check_urdf`
- Check all internal links resolve
- Run RAG extraction test (text chunking)
- Verify images render correctly

**4. REFINE** (Polish)
- Replace citation placeholders with IEEE refs
- Add actual diagrams (Mermaid, SVG, or PNG)
- Improve prose clarity based on review
- Add exercises and assessment questions

**5. PUBLISH** (Deploy)
- Create PR to main branch
- CI runs all validation
- Merge triggers GitHub Pages deploy
- Verify live site renders correctly

## Quality Validation Steps

### Automated Checks (CI/CD)

| Check | Tool | Trigger | Failure Action |
|-------|------|---------|----------------|
| Docusaurus Build | `npm run build` | Every commit | Block merge |
| Markdown Lint | markdownlint-cli | Every commit | Warning |
| Link Validation | linkinator | PR | Block merge |
| Code Syntax | Python AST parse | PR | Block merge |
| URDF Validation | check_urdf | PR | Block merge |
| RAG Extraction | Custom script | PR | Warning |

### Manual Review Checklist

- [ ] Learning objectives match chapter content
- [ ] All code examples have "How to Run" section
- [ ] Technical claims cite official documentation
- [ ] Diagrams are clear and correctly labeled
- [ ] Exercises have clear expected outcomes
- [ ] Prerequisite knowledge is accurately stated

### RAG Extraction Tests

```python
# tests/test_rag_extraction.py
def test_chapter_chunks_have_context():
    """Verify each chunk has sufficient context for RAG."""
    chunks = extract_chapters('docs/module-01-ros2/')
    for chunk in chunks:
        assert len(chunk.text) >= 100, "Chunk too short for context"
        assert chunk.heading is not None, "Chunk missing heading"
        assert chunk.module is not None, "Chunk missing module context"

def test_code_blocks_extracted_separately():
    """Verify code blocks are tagged for special handling."""
    chunks = extract_chapters('docs/module-01-ros2/')
    code_chunks = [c for c in chunks if c.type == 'code']
    assert len(code_chunks) > 0, "No code chunks found"
    for chunk in code_chunks:
        assert chunk.language is not None, "Code chunk missing language"
```

## Architectural Decisions

### ADR-001: Docusaurus for Book Platform

**Decision**: Use Docusaurus 3.x for the textbook platform.

**Rationale**:
- Native MDX support for interactive components
- Built-in versioning for future editions
- Excellent code block support with syntax highlighting
- GitHub Pages deployment is trivial
- Active community and Meta backing

**Alternatives Considered**:
- MkDocs: Less interactive component support
- GitBook: Commercial, less customizable
- Custom Next.js: Too much overhead for content-focused site

### ADR-002: Code Examples as Both Embedded and Standalone

**Decision**: Code appears inline in chapters AND as downloadable complete packages.

**Rationale**:
- Inline code enables reading flow
- Complete packages ensure reproducibility
- Students can copy-paste or git clone

**Implementation**:
- `/examples/ros2_ws/` contains buildable ROS 2 workspace
- `/book/docs/*/assets/code/` mirrors for download links
- Chapter code blocks reference the source files

### ADR-003: Module-per-Folder Organization

**Decision**: Each module is a folder with `_category_.json` and chapter files.

**Rationale**:
- Matches Docusaurus sidebar generation
- Enables per-module navigation
- Supports module-level landing pages (index.md)
- Easy to reorder modules by changing `position` in category.json

### ADR-004: RAG Chunking by Semantic Headings

**Decision**: Chunk content by h2/h3 headings, preserve code blocks as separate tagged chunks.

**Rationale**:
- Heading-based chunks have natural topic boundaries
- Code blocks need special retrieval treatment
- Enables "ask about this code" functionality
- Average chunk size ~500-1000 tokens (optimal for embedding)

## Complexity Tracking

> No constitution violations requiring justification.

| Item | Complexity Level | Justification |
|------|-----------------|---------------|
| Docusaurus setup | Low | Standard tooling |
| Code example validation | Medium | Requires ROS 2 environment |
| RAG integration | Medium | Separate feature, minimal coupling |

## Next Steps

1. **Phase 0 Complete**: research.md with Docusaurus best practices, ROS 2 Humble API verification
2. **Phase 1 Complete**: data-model.md, chapter outlines in contracts/, quickstart.md
3. **Run /sp.tasks**: Generate implementation tasks for chapters
4. **Begin Implementation**: Follow Draft → Build → Test → Refine → Publish workflow
