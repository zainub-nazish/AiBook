# Research: Module 1 — ROS 2 Fundamentals

**Date**: 2025-12-08
**Branch**: `001-ros2-fundamentals`
**Purpose**: Resolve technical unknowns and establish best practices for implementation

## 1. Docusaurus Best Practices

### Decision: Docusaurus 3.x with MDX

**Rationale**:
- Docusaurus 3.x is the current stable version (released 2023)
- Native MDX support enables React components in Markdown
- Built-in features: versioning, i18n, search, dark mode
- Excellent code block support with Prism syntax highlighting

**Alternatives Considered**:
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| Docusaurus 3.x | MDX, versioning, active | React dependency | ✅ Selected |
| MkDocs Material | Python-native, clean | Less interactive | Rejected |
| GitBook | Hosted, collaborative | Commercial, less control | Rejected |
| Sphinx | RST, Python docs standard | Less modern, harder MDX | Rejected |

**Configuration Decisions**:
- Use `@docusaurus/preset-classic` for standard features
- Enable `prism-react-renderer` for code syntax highlighting
- Configure sidebar auto-generation from folder structure
- Use MDX for chapters requiring interactive components (Tabs, Admonitions)

### Code Block Configuration

```javascript
// docusaurus.config.js - Prism languages
prism: {
  theme: lightCodeTheme,
  darkTheme: darkCodeTheme,
  additionalLanguages: ['python', 'bash', 'xml', 'yaml', 'json'],
},
```

## 2. ROS 2 Humble API Verification

### Decision: Target ROS 2 Humble (LTS until May 2027)

**Rationale**:
- Humble is current LTS release (Ubuntu 22.04)
- Stable APIs unlikely to change during textbook lifecycle
- Wide adoption in industry and education
- Official documentation well-maintained

**Key APIs to Use**:

| API | Module | Version | Official Doc |
|-----|--------|---------|--------------|
| `rclpy` | Core Python client | humble | [docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) |
| `std_msgs` | Standard messages | humble | [docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) |
| `sensor_msgs` | Sensor messages | humble | [docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) |
| `geometry_msgs` | Geometry messages | humble | [docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) |
| `robot_state_publisher` | URDF visualization | humble | [wiki.ros.org](https://wiki.ros.org/robot_state_publisher) |
| `joint_state_publisher` | Joint state GUI | humble | [wiki.ros.org](https://wiki.ros.org/joint_state_publisher) |

**Deprecated/Avoided APIs**:
- `rospy` (ROS 1 only) — Use `rclpy`
- `roslaunch` (ROS 1) — Use `ros2 launch`
- `catkin` (ROS 1 build) — Use `colcon`

### URDF Specification

**Decision**: Use URDF (not SDF) for Module 1

**Rationale**:
- URDF is ROS-native and widely taught
- SDF is Gazebo-specific (covered in Module 2)
- URDF validation tools (`check_urdf`) are standard
- robot_state_publisher expects URDF

**URDF Elements to Cover**:
```xml
<robot name="humanoid">
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## 3. Python Package Structure

### Decision: Use `ros2 pkg create` with ament_python

**Rationale**:
- Standard ROS 2 package creation workflow
- `ament_python` is recommended for pure Python packages
- Consistent with official tutorials

**Package Structure**:
```text
humanoid_basics/
├── package.xml              # ROS 2 package manifest
├── setup.py                 # Python package setup
├── setup.cfg                # Entry point configuration
├── resource/
│   └── humanoid_basics      # Marker file for ament
├── humanoid_basics/
│   ├── __init__.py
│   ├── publisher_node.py
│   ├── subscriber_node.py
│   └── ...
└── test/
    └── test_*.py
```

**Key Files**:

```xml
<!-- package.xml -->
<package format="3">
  <name>humanoid_basics</name>
  <version>0.1.0</version>
  <description>ROS 2 basics for humanoid robotics</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

```python
# setup.py
from setuptools import setup

package_name = 'humanoid_basics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'publisher_node = humanoid_basics.publisher_node:main',
            'subscriber_node = humanoid_basics.subscriber_node:main',
        ],
    },
)
```

## 4. Visualization Tools

### Decision: RViz2 for robot visualization

**Rationale**:
- Standard ROS 2 visualization tool
- Included in `ros-humble-desktop` install
- Direct URDF visualization support
- Students likely already have it installed

**Required Nodes for URDF Visualization**:
```bash
# Terminal 1: Publish URDF to robot_description parameter
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat humanoid.urdf)"

# Terminal 2: GUI for joint states
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: RViz2
ros2 run rviz2 rviz2
```

## 5. RAG Content Extraction Strategy

### Decision: Heading-based chunking with code block preservation

**Rationale**:
- h2/h3 headings provide natural semantic boundaries
- Code blocks need special handling (language tag, complete context)
- ~500-1000 tokens per chunk optimal for embeddings
- Docusaurus MDX parses cleanly to AST

**Chunking Algorithm**:
```python
def extract_chunks(mdx_content: str) -> list[Chunk]:
    """Extract RAG-ready chunks from MDX content."""
    chunks = []
    current_chunk = None

    for node in parse_mdx(mdx_content):
        if node.type == 'heading' and node.depth <= 3:
            if current_chunk:
                chunks.append(current_chunk)
            current_chunk = Chunk(
                heading=node.text,
                depth=node.depth,
                text='',
                type='prose'
            )
        elif node.type == 'code':
            # Code blocks are separate chunks
            chunks.append(Chunk(
                heading=current_chunk.heading if current_chunk else None,
                text=node.value,
                type='code',
                language=node.lang
            ))
        else:
            if current_chunk:
                current_chunk.text += node.text

    return chunks
```

## 6. Testing Strategy

### Decision: Multi-layer validation

| Layer | Tool | What It Validates |
|-------|------|-------------------|
| Build | `npm run build` | Docusaurus compiles, MDX valid |
| Lint | `markdownlint` | Markdown style consistency |
| Links | `linkinator` | All internal/external links work |
| Code | Python AST | Python syntax in code blocks |
| URDF | `check_urdf` | URDF schema validity |
| RAG | Custom pytest | Chunks extractable, meet size requirements |

### CI Pipeline (GitHub Actions)

```yaml
# .github/workflows/validate.yml
name: Validate Book
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
      - run: npm ci
      - run: npm run build
      - run: npx markdownlint docs/**/*.md
      - run: npx linkinator ./build --recurse
```

## Summary of Decisions

| Topic | Decision | Confidence |
|-------|----------|------------|
| Book Platform | Docusaurus 3.x | High |
| ROS Version | Humble (LTS) | High |
| Python Client | rclpy | High |
| Robot Description | URDF (not SDF) | High |
| Package Type | ament_python | High |
| Visualization | RViz2 | High |
| RAG Chunking | h2/h3 headings | Medium |
| CI/CD | GitHub Actions | High |

## Open Questions (for later modules)

1. **Gazebo version**: Fortress vs Harmonic (Module 2 decision)
2. **Unity integration**: ros2-for-unity vs custom bridge (Module 2)
3. **Isaac version**: Isaac Sim 2023.1+ vs Isaac ROS (Module 7)
4. **Embedding model**: text-embedding-3-small vs alternatives (RAG feature)

## References

1. [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
2. [Docusaurus Documentation](https://docusaurus.io/docs)
3. [URDF Specification](https://wiki.ros.org/urdf/XML)
4. [rclpy API Reference](https://docs.ros2.org/humble/api/rclpy/)
5. [ROS 2 Python Package Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
