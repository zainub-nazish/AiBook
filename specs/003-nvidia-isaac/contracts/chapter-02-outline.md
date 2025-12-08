# Chapter 2 Outline: Isaac Sim for Synthetic Data

**User Story**: US2 - Isaac Sim Synthetic Data Workflow (P2)
**Estimated Time**: 60 minutes reading + 45 minutes exercises

## Learning Objectives

By the end of this chapter, students will be able to:

1. Install and launch NVIDIA Isaac Sim
2. Create a basic photorealistic indoor environment
3. Configure sensors (camera, LiDAR) on a robot model
4. Apply Domain Randomization for training data diversity
5. Export synthetic datasets in COCO format

## Prerequisites

- Completed Chapter 1: Introduction to Robot Perception
- NVIDIA GPU with 8GB+ VRAM
- Ubuntu 22.04 or Windows 10/11
- Basic Python programming

## Section Structure

### 2.1 Why Synthetic Data? (Theory)
- The data problem in robotics ML
- Real vs synthetic data trade-offs
- Domain gap and transfer learning
- Success stories: synthetic data in production
- **Diagram**: synthetic-data-benefits.svg

### 2.2 Isaac Sim Installation and Setup (Tutorial)
- System requirements verification
- Omniverse Launcher installation
- Isaac Sim installation
- First launch and interface tour
- Nucleus asset browser
- **Code Example**: Installation verification script

### 2.3 Creating Photorealistic Environments (Tutorial)
- Loading warehouse/indoor assets
- Placing objects and furniture
- Adding lighting (dome, area, spot)
- PBR materials and textures
- **Code Example**: Python scene creation script

### 2.4 Adding Robots and Sensors (Tutorial)
- Importing URDF models
- Adding RGB camera
- Adding depth camera
- Adding LiDAR sensor
- Sensor parameter configuration
- **Diagram**: sensor-placement.svg
- **Code Example**: Sensor configuration Python script

### 2.5 Domain Randomization with Replicator (Tutorial)
- What is Domain Randomization?
- Replicator API overview
- Lighting randomization
- Texture/material randomization
- Object pose randomization
- Camera parameter randomization
- **Diagram**: domain-randomization.svg
- **Code Example**: Domain randomization script

### 2.6 Generating and Exporting Data (Tutorial)
- RGB image generation
- Depth image generation
- Semantic segmentation masks
- Bounding box annotations
- COCO format export
- **Code Example**: Dataset generation script

### 2.7 Isaac Sim vs Gazebo Comparison (Theory)
- Rendering quality differences
- Physics engine comparison
- ROS integration approaches
- When to use each simulator
- **Table**: Feature comparison matrix

## Exercises

### Exercise 2.1: Environment Setup (20 min)
**Objective**: Verify Isaac Sim installation
**Task**: Install Isaac Sim and launch the default warehouse scene
**Success Criteria**: Scene loads and robot can be teleported

### Exercise 2.2: Custom Scene Creation (25 min)
**Objective**: Build environment from assets
**Task**: Create a simple room with:
- 4 walls
- 2 tables with objects
- Overhead lighting
- A humanoid robot
**Success Criteria**: Scene renders with shadows visible

### Exercise 2.3: Synthetic Dataset Generation (30 min)
**Objective**: Generate ML training data
**Task**: Configure sensors and generate:
- 100 RGB images
- 100 depth images
- COCO annotations
**Success Criteria**: Dataset exports with valid JSON annotations

## Assessment Questions

1. **[Multiple Choice]** What is the primary purpose of Domain Randomization?
   - A) Make images look more realistic
   - B) Reduce the domain gap between synthetic and real data
   - C) Speed up rendering
   - D) Reduce file sizes

2. **[Short Answer]** Explain why EXR format is preferred over PNG for depth images in robotics applications.

3. **[Multiple Choice]** Which Isaac Sim API is used for programmatic scene creation and data generation?
   - A) Isaac SDK
   - B) Omniverse Kit
   - C) Replicator
   - D) PhysX

4. **[Short Answer]** List three types of randomization you would apply to improve the generalization of an object detection model trained on synthetic data.

5. **[Practical]** Given a URDF robot model, describe the steps to add a depth camera sensor in Isaac Sim.

## Key Diagrams

| Diagram | Description | Location |
|---------|-------------|----------|
| synthetic-data-benefits.svg | Real vs synthetic comparison | assets/diagrams/ |
| isaac-sim-workflow.svg | End-to-end Isaac Sim pipeline | assets/diagrams/ |
| domain-randomization.svg | DR techniques visualization | assets/diagrams/ |
| sensor-placement.svg | Sensor mounting on robot | assets/diagrams/ |

## Code Examples

| File | Description | Location |
|------|-------------|----------|
| verify_install.py | Installation verification | examples/isaac/synthetic_data/ |
| create_scene.py | Programmatic scene creation | examples/isaac/synthetic_data/ |
| generate_rgb_depth.py | Data generation script | examples/isaac/synthetic_data/ |
| domain_randomizer.py | Domain randomization setup | examples/isaac/synthetic_data/ |
| export_dataset.py | COCO format export | examples/isaac/synthetic_data/ |

## References

1. NVIDIA. (2024). *Isaac Sim Documentation*. https://docs.omniverse.nvidia.com/isaacsim/latest/
2. NVIDIA. (2024). *Replicator Documentation*. https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/
3. Tobin, J., et al. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World." *IROS*.
4. Tremblay, J., et al. (2018). "Training Deep Networks with Synthetic Data." *CVPR Workshops*.
