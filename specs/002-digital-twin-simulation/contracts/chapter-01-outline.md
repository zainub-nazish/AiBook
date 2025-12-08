# Chapter Outline: Physics Simulation in Gazebo

**Chapter ID**: chapter-01-gazebo-physics
**Module**: 02 - The Digital Twin
**User Story**: US1 (P1)
**Estimated Reading Time**: 45-60 minutes

## Learning Objectives

By the end of this chapter, students will be able to:

1. Explain the architecture and components of Gazebo Harmonic
2. Create SDF world files with ground planes, lighting, and objects
3. Configure physics parameters including gravity, friction, and collisions
4. Load and position robot models from URDF/SDF files
5. Verify physics behavior through simulation inspection and testing

## Prerequisites

- Module 1: ROS 2 Fundamentals (nodes, topics, launch files)
- Basic understanding of 3D coordinate systems
- Familiarity with XML syntax

## Chapter Structure

### 1. Introduction to Gazebo Harmonic (Theory)

**Content**:
- What is Gazebo? History and naming (Classic vs Harmonic)
- Architecture overview: gz-sim, gz-physics, gz-rendering, gz-sensors
- Integration with ROS 2 via ros_gz packages
- Use cases: testing, training data, development iteration

**Diagram**: Gazebo Architecture (gz ecosystem components)

**Key Terms**:
- SDF (Simulation Description Format)
- Physics engine
- World
- Model
- Plugin

### 2. SDF World File Structure (Tutorial)

**Content**:
- SDF 1.10 specification and documentation
- `<world>` element and required children
- Including models from Gazebo Fuel
- Pose specification (`<pose>` element)
- Coordinate frames and transforms

**Code Examples**:
```xml
<!-- minimal-world.sdf -->
<!-- World with ground plane and sun -->
```

```xml
<!-- world-with-objects.sdf -->
<!-- Adding static objects (boxes, spheres) -->
```

**Hands-on**: Create a world with a ground plane and three different objects

### 3. Configuring Physics Properties (Tutorial)

**Content**:
- Physics engines comparison: ODE (default), DART, Bullet
- `<physics>` element configuration
- Step size and real-time factor trade-offs
- Gravity vector (`<gravity>`)
- Surface properties: friction coefficients, contact parameters
- Collision geometries: box, cylinder, sphere, mesh

**Code Examples**:
```xml
<!-- physics-config.sdf -->
<!-- Physics engine selection and parameters -->
```

```xml
<!-- collision-demo.sdf -->
<!-- Objects with different friction values -->
```

**Diagram**: Physics timestep vs accuracy trade-off

**Hands-on**: Experiment with friction values and observe object sliding behavior

### 4. Loading Robot Models (Tutorial)

**Content**:
- Including URDF models in SDF worlds
- Converting URDF to SDF (gz sdf command)
- Model spawning via gz service
- Initial pose configuration
- Joint state initialization
- Visualizing joint positions in Gazebo GUI

**Code Examples**:
```xml
<!-- humanoid-world.sdf -->
<!-- World with humanoid robot from Module 1 -->
```

```bash
# spawn-robot.bash
# Spawning models at runtime
```

**Diagram**: URDF to SDF conversion workflow

**Hands-on**: Load the humanoid robot from Module 1 into a Gazebo world

### 5. Verifying Physics Behavior (Tutorial)

**Content**:
- Using the Gazebo GUI effectively
- Applying forces and torques interactively
- Recording and playback for debugging
- Common issues: objects falling through floor, unstable contacts
- Debugging techniques: step-by-step, slow motion

**Admonitions**:
- :::warning Physics instability signs
- :::tip Debugging with recording

**Hands-on**: Apply forces to robot links and verify expected motion

## Exercises

1. **World Creation**: Create a world file with a table (box) at height 0.8m, place three objects on it, and verify they don't fall through.

2. **Friction Experiment**: Create two ramps with different friction coefficients. Roll a sphere down each and observe the difference in behavior.

3. **Pendulum Validation**: Create a simple pendulum and measure its period. Calculate expected period from physics and compare.

## Assessment Questions

1. What is the difference between Gazebo Harmonic and Gazebo Classic? When would you use each?

2. How does physics step size (`max_step_size`) affect simulation accuracy and performance? What are the trade-offs?

3. Why might objects pass through each other during simulation? How can you prevent this?

4. Explain the difference between visual and collision geometries in SDF.

5. How do you spawn a robot model at a specific location in an already-running simulation?

## Code Files Delivered

| File | Description | Location |
|------|-------------|----------|
| `minimal-world.sdf` | Basic world template | `examples/gazebo/worlds/` |
| `world-with-objects.sdf` | World with static objects | `examples/gazebo/worlds/` |
| `physics-config.sdf` | Physics parameter examples | `examples/gazebo/worlds/` |
| `humanoid-world.sdf` | World with humanoid robot | `examples/gazebo/worlds/` |
| `spawn-robot.bash` | Model spawning script | `examples/gazebo/scripts/` |

## Diagrams Required

1. **Gazebo Architecture**: Components and their relationships
2. **Coordinate Systems**: SDF/Gazebo frame conventions
3. **Physics Timestep Trade-off**: Accuracy vs performance graph
4. **URDF to SDF Workflow**: Conversion process

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [SDF Specification 1.10](http://sdformat.org/spec)
- [ros_gz Integration](https://github.com/gazebosim/ros_gz)
