# Chapter Outline: Unity for Human-Robot Interaction Environments

**Chapter ID**: chapter-02-unity-rendering
**Module**: 02 - The Digital Twin
**User Story**: US2 (P2)
**Estimated Reading Time**: 50-65 minutes

## Learning Objectives

By the end of this chapter, students will be able to:

1. Set up a Unity project with Robotics Hub packages
2. Import URDF robot models with correct coordinate transformations
3. Configure lighting for realistic indoor environments
4. Apply PBR materials and textures to scene objects
5. Connect Unity to ROS 2 for bidirectional communication

## Prerequisites

- Chapter 1: Physics Simulation in Gazebo (coordinate system concepts)
- Basic familiarity with Unity Editor interface (optional but helpful)
- ROS 2 workspace from Module 1

## Chapter Structure

### 1. Unity Robotics Environment Setup (Tutorial)

**Content**:
- Installing Unity Hub and Unity 2022 LTS
- Creating a new 3D project
- Adding Robotics Hub packages via Package Manager
  - com.unity.robotics.urdf-importer
  - com.unity.robotics.ros-tcp-connector
- Project settings for simulation (physics timestep, quality)
- Understanding Unity's coordinate system (left-handed, Y-up)

**Code Examples**:
```json
// manifest.json
// Package dependencies for robotics
```

**Diagram**: Unity vs ROS/Gazebo coordinate systems

**Admonition**: :::warning Coordinate system differences are the #1 source of errors

**Hands-on**: Create a new Unity robotics project with required packages

### 2. Importing Robot Models (Tutorial)

**Content**:
- URDF Importer workflow step-by-step
- How the importer handles coordinate transforms
- Collision vs visual mesh assignment
- Joint configuration: limits, axes, types
- Common import issues and fixes
- Articulation bodies vs Rigidbodies

**Code Examples**:
```csharp
// ImportRobot.cs
// Programmatic URDF import (optional)
```

**Diagram**: URDF Import workflow in Unity

**Hands-on**: Import the humanoid URDF from Module 1 into Unity

### 3. Building Indoor Environments (Tutorial)

**Content**:
- Creating basic geometry with Unity primitives
- Using ProBuilder for quick room modeling
- Importing external 3D assets (FBX, OBJ)
- Scale considerations: meters in robotics vs Unity units
- Organizing scene hierarchy for clarity
- Prefabs for reusable environment elements

**Diagram**: Scene hierarchy best practices

**Hands-on**: Create a simple room (4 walls, floor, ceiling) at realistic scale

### 4. Lighting and Materials (Tutorial)

**Content**:
- Light types and their uses:
  - Directional (sun)
  - Point (light bulbs)
  - Spot (focused)
  - Area (soft box)
- Global illumination: baked vs real-time
- PBR (Physically Based Rendering) materials
  - Albedo, metallic, smoothness, normal maps
- Reflection probes for realistic surfaces
- Light probes for indirect lighting on dynamic objects

**Code Examples**:
```csharp
// MaterialSetup.cs
// Creating materials programmatically
```

**Diagram**: PBR material properties visualization

**Hands-on**: Light the room scene with a directional light (window) and point lights (ceiling)

### 5. ROS 2 Integration (Tutorial)

**Content**:
- ROS-TCP-Connector architecture
- Setting up ROS-TCP-Endpoint on ROS 2 side
- ROSConnection singleton pattern
- Publishing messages from Unity (joint states)
- Subscribing to messages in Unity (commands)
- Clock synchronization between Unity and ROS 2
- Handling network configuration

**Code Examples**:
```csharp
// ROSConnection.cs
// Connection setup and configuration
```

```csharp
// JointStatePublisher.cs
// Publishing current joint positions
```

```python
# ros_tcp_endpoint launch
# ROS 2 side setup
```

**Diagram**: Unity-ROS 2 communication architecture

**Hands-on**: Publish robot joint states from Unity and visualize in RViz2

## Exercises

1. **Robot Import**: Import the humanoid robot from Module 1, verify all joints move correctly, and position it in your room scene.

2. **Scene Composition**: Create a living room environment with:
   - A table (0.75m height)
   - A chair
   - A lamp (point light source)
   - Window with directional light

3. **ROS 2 Connection**:
   - Publish joint states from Unity at 30Hz
   - Visualize the robot in RViz2
   - Send a joint command from ROS 2 and verify the Unity robot responds

## Assessment Questions

1. How does Unity's coordinate system differ from ROS/Gazebo? What transformation is applied when importing URDF?

2. What is the difference between baked and real-time global illumination? When would you use each?

3. Explain the role of reflection probes in realistic rendering.

4. Why might joint orientations appear incorrect after URDF import? How would you debug this?

5. What is the ROS-TCP-Endpoint and why is it necessary for Unity-ROS 2 communication?

## Code Files Delivered

| File | Description | Location |
|------|-------------|----------|
| `manifest.json` | Package dependencies | `examples/unity/Packages/` |
| `ImportRobot.cs` | URDF import helper | `examples/unity/Assets/Scripts/` |
| `MaterialSetup.cs` | Material creation | `examples/unity/Assets/Scripts/` |
| `ROSConnection.cs` | ROS setup | `examples/unity/Assets/Scripts/ROS/` |
| `JointStatePublisher.cs` | Joint state publishing | `examples/unity/Assets/Scripts/ROS/` |
| `ros_tcp_endpoint.launch.py` | ROS 2 launch file | `examples/ros2_ws/src/unity_integration/` |

## Diagrams Required

1. **Coordinate Systems**: Unity (Y-up) vs ROS (Z-up) comparison
2. **URDF Import Workflow**: Steps from file to scene
3. **PBR Properties**: Visual explanation of material parameters
4. **Unity-ROS Architecture**: Communication flow diagram

## References

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF Importer Documentation](https://github.com/Unity-Technologies/URDF-Importer)
- [Unity Manual: Lighting](https://docs.unity3d.com/Manual/Lighting.html)
- [Unity Manual: Materials](https://docs.unity3d.com/Manual/Materials.html)
