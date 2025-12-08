# Module 3: The AI-Robot Brain (NVIDIA Isaac)

Welcome to Module 3! In this module, you'll learn how humanoid robots perceive and navigate their environments using the NVIDIA Isaac ecosystem. You'll explore the "AI-Robot Brain" - the perception and decision-making systems that enable autonomous robot behavior.

## Module Overview

This module covers four key areas of robot intelligence:

1. **Perception Pipelines** - How robots "see" and understand their world
2. **Synthetic Data Generation** - Using Isaac Sim for photorealistic simulation
3. **GPU-Accelerated Perception** - Isaac ROS for real-time visual processing
4. **Autonomous Navigation** - Nav2 path planning for bipedal humanoids

## Learning Objectives

By the end of this module, you will be able to:

- Explain how perception pipelines work from sensors to navigation
- Create synthetic training data using NVIDIA Isaac Sim
- Configure Isaac ROS for GPU-accelerated VSLAM and depth processing
- Set up Nav2 navigation for humanoid robots
- Build a complete perception-to-navigation pipeline

## Prerequisites

Before starting this module, you should have:

- Completed **Module 1: ROS 2 Fundamentals**
- Completed **Module 2: Digital Twin Simulation**
- A computer with **NVIDIA GPU** (8GB+ VRAM recommended)
- **Ubuntu 22.04** or compatible Linux distribution
- Basic Python programming knowledge
- Familiarity with command-line interfaces

## Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | NVIDIA GTX 1070 | NVIDIA RTX 3080+ |
| VRAM | 8 GB | 16 GB |
| RAM | 16 GB | 32 GB |
| Storage | 50 GB free | 100 GB SSD |
| CUDA | 11.8+ | 12.0+ |
| Driver | 525+ | 535+ |

## Module Structure

| Chapter | Topic | Duration |
|---------|-------|----------|
| [Chapter 1](./chapter-01-perception) | Introduction to Robot Perception | 45 min |
| [Chapter 2](./chapter-02-isaac-sim) | Isaac Sim for Synthetic Data | 60 min |
| [Chapter 3](./chapter-03-isaac-ros) | Isaac ROS Perception Nodes | 50 min |
| [Chapter 4](./chapter-04-nav2) | Nav2 for Humanoid Navigation | 55 min |
| [Chapter 5](./chapter-05-integration) | End-to-End Integration | 45 min |

**Total Module Duration**: ~4.5 hours of reading + hands-on exercises

## Key Technologies

### NVIDIA Isaac Ecosystem

```
┌─────────────────────────────────────────────────────────────┐
│                    NVIDIA Isaac Ecosystem                    │
├─────────────────┬─────────────────┬─────────────────────────┤
│   Isaac Sim     │   Isaac ROS     │      Isaac Lab          │
│   (Simulation)  │   (Perception)  │   (RL Training)         │
├─────────────────┴─────────────────┴─────────────────────────┤
│                    Omniverse Platform                        │
├─────────────────────────────────────────────────────────────┤
│                    CUDA / cuDNN / TensorRT                   │
└─────────────────────────────────────────────────────────────┘
```

### The Perception-Navigation Pipeline

```
Sensors → Perception → Mapping → Localization → Navigation → Motion
  │          │           │           │             │           │
Camera    VSLAM      Occupancy    Where am I?   Path Plan   Velocity
LiDAR     Depth       Grid        Pose Est.     Costmap     Commands
IMU       PointCloud  TSDF                      BT
```

## Getting Started

1. **Verify Prerequisites**: Ensure you have completed Modules 1 and 2
2. **Check Hardware**: Verify your NVIDIA GPU meets the requirements
3. **Install Software**: Follow the installation guides in each chapter
4. **Work Through Chapters**: Complete chapters in order (1 → 2 → 3 → 4 → 5)

## Resources

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [Nav2 Documentation](https://docs.nav2.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

---

Ready to begin? Start with [Chapter 1: Introduction to Robot Perception](./chapter-01-perception)!
