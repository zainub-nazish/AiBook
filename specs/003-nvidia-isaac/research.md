# Research Document: Module 3 — The AI-Robot Brain (NVIDIA Isaac)

**Feature**: 003-nvidia-isaac
**Date**: 2025-12-08
**Status**: Complete

## Research Tasks

### RT-001: Isaac Sim Installation and Setup Requirements

**Question**: What are the minimum hardware/software requirements for Isaac Sim 2023.x?

**Findings**:
- **GPU**: NVIDIA RTX 2070 or higher recommended (RTX 30/40 series ideal)
- **VRAM**: 8GB minimum, 16GB+ recommended for complex scenes
- **CPU**: Intel i7/AMD Ryzen 7 or higher
- **RAM**: 32GB recommended
- **Storage**: 50GB+ for Isaac Sim + assets
- **OS**: Ubuntu 20.04/22.04, Windows 10/11
- **Driver**: NVIDIA driver 525.60+ required
- **CUDA**: CUDA 11.8+ (bundled with Isaac Sim)

**Decision**: Target Ubuntu 22.04 with NVIDIA GPU having 8GB+ VRAM
**Rationale**: Aligns with ROS 2 Humble LTS and provides optimal Isaac ROS compatibility
**Source**: [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html)

---

### RT-002: Isaac ROS Package Ecosystem Overview

**Question**: Which Isaac ROS packages are most relevant for perception education?

**Findings**:
Core packages for perception pipeline:
1. **isaac_ros_visual_slam** - GPU-accelerated Visual SLAM using stereo cameras
2. **isaac_ros_depth_image_proc** - GPU depth image processing and point cloud generation
3. **isaac_ros_nvblox** - 3D reconstruction and occupancy mapping
4. **isaac_ros_apriltag** - GPU-accelerated AprilTag detection
5. **isaac_ros_image_pipeline** - GPU image processing (debayer, rectify)
6. **isaac_ros_freespace_segmentation** - Semantic segmentation for navigation

**Decision**: Focus on visual_slam, depth_image_proc, and nvblox as primary packages
**Rationale**: These three provide the complete perception pipeline from raw images to 3D maps
**Source**: [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)

---

### RT-003: Nav2 Humanoid Configuration Best Practices

**Question**: How should Nav2 be configured for bipedal humanoid robots?

**Findings**:
Key configuration areas:
1. **Footprint**: Use polygon footprint matching humanoid base, consider stance width
2. **Costmap resolution**: 0.05m resolution recommended for humanoid-scale obstacles
3. **Inflation radius**: Based on humanoid arm reach + safety margin
4. **Controller frequency**: 20Hz minimum for smooth humanoid motion
5. **Planner**: NavFn or Smac work well; ThetaStar for smoother paths

Humanoid-specific considerations:
- Turning radius constraints (humanoids can turn in place but slowly)
- Slope/stair handling (out of scope for basic Nav2)
- Balance considerations (handled by low-level controller, not Nav2)

**Decision**: Use Nav2 with custom polygon footprint and adjusted inflation
**Rationale**: Nav2 handles path planning; humanoid-specific locomotion is separate
**Source**: [Nav2 Documentation](https://docs.nav2.org/)

---

### RT-004: Domain Randomization Techniques

**Question**: What Domain Randomization techniques should be covered for synthetic data?

**Findings**:
Essential DR techniques:
1. **Lighting randomization**: Direction, intensity, color temperature
2. **Texture randomization**: Material swaps, procedural textures
3. **Object pose randomization**: Position, rotation within bounds
4. **Camera parameter randomization**: FOV, noise, exposure
5. **Distractors**: Random background objects

Isaac Sim Replicator API provides:
- `rep.create.camera()` - Configurable cameras
- `rep.randomizer.materials()` - Material randomization
- `rep.randomizer.scatter()` - Object scattering
- `rep.randomizer.light()` - Lighting randomization

**Decision**: Cover lighting, texture, pose, and camera randomization in Chapter 2
**Rationale**: These four cover the most impactful DR techniques for robotics perception
**Source**: [Isaac Sim Replicator Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)

---

### RT-005: VSLAM Algorithm Comparison

**Question**: How does Isaac ROS Visual SLAM compare to other VSLAM implementations?

**Findings**:
| Algorithm | GPU Accel | Accuracy | Latency | Loop Closure |
|-----------|-----------|----------|---------|--------------|
| Isaac VSLAM | Yes (cuVSLAM) | High | <10ms | Yes |
| ORB-SLAM3 | No | High | ~30ms | Yes |
| RTAB-Map | Partial | Medium | ~50ms | Yes |
| LSD-SLAM | No | Medium | ~40ms | No |

Isaac ROS Visual SLAM advantages:
- 10x faster than CPU alternatives
- Designed for Jetson and discrete GPUs
- Direct integration with Isaac ROS ecosystem
- Stereo and RGB-D support

**Decision**: Highlight Isaac VSLAM performance benefits in Chapter 3
**Rationale**: GPU acceleration is the key differentiator and educational focus
**Source**: [cuVSLAM Paper](https://arxiv.org/abs/2111.04413), Isaac ROS docs

---

### RT-006: Synthetic Data Export Formats

**Question**: What format should synthetic data be exported in for maximum usability?

**Findings**:
Common formats:
1. **COCO** - Widely supported, JSON annotations, good for detection/segmentation
2. **KITTI** - Standard for autonomous driving, complex structure
3. **BOP** - 6DoF pose estimation focused
4. **Custom** - Application-specific but requires tooling

Isaac Sim Replicator supports:
- COCO format (BasicWriter)
- KITTI format (KittiWriter)
- Custom writers via Python API

Depth data formats:
- PNG (16-bit) - Lossy but compact
- EXR (32-bit float) - Lossless, preferred for robotics
- NPY (NumPy) - Easy Python integration

**Decision**: Use COCO for annotations, EXR for depth
**Rationale**: Best balance of compatibility and precision for robotics applications
**Source**: Isaac Sim Replicator docs, dataset format specifications

---

### RT-007: Isaac Sim vs Gazebo for Perception Training

**Question**: When should students use Isaac Sim vs Gazebo for synthetic data?

**Findings**:
| Aspect | Isaac Sim | Gazebo |
|--------|-----------|--------|
| Rendering | RTX ray-tracing | OpenGL/OGRE |
| Physics | PhysX 5 | DART/ODE/Bullet |
| ML Focus | Designed for ML | Robotics-first |
| GPU Required | Yes (NVIDIA) | No |
| ROS Integration | Isaac ROS | Native |
| Asset Quality | Photorealistic | Functional |
| Learning Curve | Steeper | Gentler |

Use Isaac Sim when:
- Photorealistic synthetic data needed
- Domain randomization for ML training
- GPU-accelerated perception testing
- Production-quality visuals required

Use Gazebo when:
- Basic physics simulation sufficient
- No NVIDIA GPU available
- ROS integration priority
- Rapid prototyping

**Decision**: Position Isaac Sim as "next level" after Gazebo basics (Module 2)
**Rationale**: Students need Gazebo foundation before Isaac Sim complexity
**Source**: Practical experience, documentation comparison

---

## Summary of Decisions

| Research Task | Decision | Impact |
|---------------|----------|--------|
| RT-001 | Ubuntu 22.04 + NVIDIA 8GB+ | Hardware prereqs |
| RT-002 | visual_slam, depth_image_proc, nvblox | Core Isaac ROS packages |
| RT-003 | Nav2 + custom polygon footprint | Humanoid navigation approach |
| RT-004 | 4 DR techniques: light, texture, pose, camera | Synthetic data scope |
| RT-005 | Highlight GPU performance benefits | Educational focus |
| RT-006 | COCO + EXR format | Data export standard |
| RT-007 | Isaac Sim after Gazebo foundation | Curriculum progression |

## References

1. NVIDIA. (2024). *Isaac Sim Documentation*. https://docs.omniverse.nvidia.com/isaacsim/latest/
2. NVIDIA. (2024). *Isaac ROS Documentation*. https://nvidia-isaac-ros.github.io/
3. Open Robotics. (2024). *Nav2 Documentation*. https://docs.nav2.org/
4. Mur-Artal, R., & Tardós, J. D. (2017). ORB-SLAM2: An open-source SLAM system for monocular, stereo, and RGB-D cameras. *IEEE Transactions on Robotics*.
5. NVIDIA. (2024). *Replicator Documentation*. https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/
