# Module 2: The Digital Twin (Gazebo & Unity)

Welcome to Module 2 of the Physical AI & Humanoid Robotics textbook. In this module, you will learn to build **digital twins**—virtual replicas of physical robots—using industry-standard simulation tools.

## What You Will Learn

This module covers three essential aspects of robot simulation:

1. **Physics Simulation in Gazebo** - Create realistic virtual environments with accurate physics
2. **Unity for Human-Robot Interaction** - Build photorealistic scenes for perception and HRI testing
3. **Sensor Simulation** - Emulate LiDAR, depth cameras, and IMUs for perception development

## Prerequisites

Before starting this module, you should have completed:

- **Module 1: ROS 2 Fundamentals** - Understanding of nodes, topics, services, and launch files
- Basic familiarity with 3D coordinate systems (X, Y, Z axes)
- Python programming experience

## Module Overview

| Chapter | Topic | Estimated Time |
|---------|-------|----------------|
| 1 | Physics Simulation in Gazebo | 45-60 minutes |
| 2 | Unity for Human-Robot Interaction | 50-65 minutes |
| 3 | Sensor Simulation: LiDAR, Depth Cameras, IMUs | 60-75 minutes |

## Why Digital Twins?

Digital twins enable robotics developers to:

- **Test safely** - Validate robot behaviors without risking hardware damage
- **Iterate quickly** - Make changes and see results in seconds, not hours
- **Generate data** - Create synthetic training data for perception algorithms
- **Scale testing** - Run thousands of simulations in parallel

## Tools Covered

### Gazebo Harmonic
The latest generation of the Gazebo simulator, featuring:
- High-fidelity physics engines (ODE, DART, Bullet)
- SDF 1.10 world description format
- Native ROS 2 integration via ros_gz packages

### Unity with Robotics Hub
Unity's official robotics packages provide:
- URDF model importing with automatic coordinate transforms
- ROS 2 communication via ROS-TCP-Connector
- Photorealistic rendering for vision-based AI

## Getting Started

Start with [Chapter 1: Physics Simulation in Gazebo](./chapter-01-gazebo-physics) to learn the fundamentals of simulation world creation.

:::tip Before You Begin
Make sure you have Gazebo Harmonic and Unity 2022 LTS installed. See the [Quickstart Guide](../../quickstart) for installation instructions.
:::
