# Developer Quickstart: Module 3 — The AI-Robot Brain (NVIDIA Isaac)

**Feature**: 003-nvidia-isaac
**Date**: 2025-12-08

## Prerequisites

Before working on Module 3 content, ensure you have:

### Required
- [ ] Git repository cloned
- [ ] Node.js 18+ (for Docusaurus)
- [ ] Python 3.10+ (for code examples)
- [ ] ROS 2 Humble installed
- [ ] Access to NVIDIA GPU system (for testing)

### Recommended
- [ ] Isaac Sim 2023.1.0+ installed (for Ch 2 verification)
- [ ] Isaac ROS packages installed (for Ch 3 verification)
- [ ] Nav2 installed (for Ch 4 verification)

## Quick Commands

### Setup Development Environment

```bash
# Clone repository (if not done)
cd ~/repos
git clone <repo-url> book1
cd book1

# Checkout feature branch
git checkout 003-nvidia-isaac

# Install Docusaurus dependencies
cd book
npm install

# Start development server
npm run start
```

### Run Development Server

```bash
cd book
npm run start
# Opens http://localhost:3000
```

### Build for Production

```bash
cd book
npm run build
# Output in book/build/
```

### Validate Content

```bash
# Check Docusaurus build
npm run build

# Lint Python examples
cd ../examples/isaac
python -m py_compile synthetic_data/*.py

# Validate ROS 2 package
cd ../ros2_ws
colcon build --packages-select isaac_perception
```

## Directory Map

```text
book1/
├── book/docs/module-03-ai-robot-brain/   # Chapter content (MDX files)
├── examples/isaac/                        # Isaac-specific examples
│   ├── synthetic_data/                    # Ch 2 code
│   ├── isaac_ros/                         # Ch 3 configs
│   └── nav2_humanoid/                     # Ch 4 configs
├── examples/ros2_ws/src/isaac_perception/ # ROS 2 package
└── specs/003-nvidia-isaac/                # Planning docs
    ├── spec.md                            # Feature specification
    ├── plan.md                            # Implementation plan
    ├── research.md                        # Research findings
    ├── data-model.md                      # Content entities
    ├── quickstart.md                      # This file
    └── contracts/                         # Chapter outlines
```

## Content Creation Workflow

### Creating a Chapter

1. **Start with outline**: Read `contracts/chapter-XX-outline.md`
2. **Create MDX file**: `book/docs/module-03-ai-robot-brain/chapter-XX-*.mdx`
3. **Add frontmatter**: id, title, sidebar_label, position
4. **Write sections**: Follow outline structure
5. **Add diagrams**: Create SVGs in `assets/diagrams/`
6. **Add code examples**: Inline or reference `examples/isaac/`
7. **Add exercises**: Include success criteria
8. **Add assessment**: Questions with explanations
9. **Test build**: `npm run build`

### Creating Diagrams

```bash
# Diagrams go in:
book/docs/module-03-ai-robot-brain/assets/diagrams/

# Naming convention:
[concept]-[type].svg
# Examples:
perception-pipeline.svg
isaac-ros-graph.svg
nav2-architecture.svg

# Reference in MDX:
import DiagramName from './assets/diagrams/diagram-name.svg';
<DiagramName />
```

### Creating Code Examples

```python
# Code examples in examples/isaac/

# Naming convention:
[action]_[subject].py
# Examples:
generate_rgb_depth.py
domain_randomizer.py

# Every code file needs:
# 1. Module docstring with description
# 2. Prerequisites comment
# 3. Example usage at bottom
```

## Testing Content

### Manual Testing Checklist

- [ ] Chapter builds without errors
- [ ] All images/diagrams render
- [ ] All code blocks have syntax highlighting
- [ ] All links work (internal and external)
- [ ] Learning objectives match content
- [ ] Exercises are completable

### Code Example Testing

```bash
# For Isaac Sim examples (requires Isaac Sim):
cd examples/isaac/synthetic_data
./isaacsim.sh python generate_rgb_depth.py

# For Isaac ROS examples (requires ROS 2 + Isaac ROS):
cd examples/isaac/isaac_ros
ros2 launch launch/vslam_demo.launch.py

# For Nav2 examples (requires Nav2):
cd examples/isaac/nav2_humanoid
ros2 launch launch/humanoid_nav.launch.py
```

## Common Issues

### Build Fails

```bash
# Clear cache and rebuild
cd book
rm -rf node_modules .docusaurus
npm install
npm run build
```

### Diagram Not Rendering

- Ensure SVG is valid XML
- Check import path in MDX
- Verify file exists in assets/diagrams/

### Code Highlighting Wrong

- Check language tag: ```python, ```yaml, ```bash
- Ensure no invisible characters

## Style Guide Quick Reference

### Markdown/MDX

- Use ATX headings (# H1, ## H2)
- Code blocks with language tags
- Tables for comparisons
- Admonitions for tips/warnings: :::tip, :::warning

### Citations

- IEEE numeric format: [1], [2]
- References section at chapter end
- Official docs get stable URLs

### Diagrams

- SVG format preferred
- Max width 800px
- Include alt text
- Caption as figure description

### Code

- Python follows PEP 8
- YAML uses 2-space indent
- Bash shows $ prompt
- Comments explain "why"

## Key Files to Edit

| Task | Files |
|------|-------|
| Chapter 1 content | `chapter-01-perception.mdx` |
| Chapter 2 content | `chapter-02-isaac-sim.mdx` |
| Chapter 3 content | `chapter-03-isaac-ros.mdx` |
| Chapter 4 content | `chapter-04-nav2.mdx` |
| Chapter 5 content | `chapter-05-integration.mdx` |
| Module overview | `index.md` |
| Sidebar config | `_category_.json` |
| Perception diagram | `assets/diagrams/perception-pipeline.svg` |
| Isaac ecosystem diagram | `assets/diagrams/isaac-ecosystem.svg` |

## Getting Help

- Check `specs/003-nvidia-isaac/` for planning docs
- Review prior modules for patterns
- Consult constitution for principles
- Reference official docs for technical accuracy
