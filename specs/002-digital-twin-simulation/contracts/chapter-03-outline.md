# Chapter Outline: Sensor Simulation — LiDAR, Depth Cameras, IMUs

**Chapter ID**: chapter-03-sensor-simulation
**Module**: 02 - The Digital Twin
**User Story**: US3 (P3)
**Estimated Reading Time**: 60-75 minutes

## Learning Objectives

By the end of this chapter, students will be able to:

1. Configure LiDAR sensors in Gazebo with appropriate range, resolution, and noise
2. Set up depth cameras in both Gazebo and Unity
3. Implement IMU simulation with realistic noise models
4. Visualize sensor data using RViz2 and custom tools
5. Understand ROS 2 sensor message formats and their fields

## Prerequisites

- Chapter 1: Physics Simulation in Gazebo (world files, SDF)
- Chapter 2: Unity for Human-Robot Interaction (ROS 2 integration)
- ROS 2 message types from Module 1

## Chapter Structure

### 1. Introduction to Robot Sensors (Theory)

**Content**:
- Proprioceptive vs exteroceptive sensors
- Role of sensors in the perception pipeline
- Why simulate sensors? Development speed, safety, data generation
- Real vs simulated sensor characteristics
- ROS 2 sensor message types overview:
  - `sensor_msgs/PointCloud2`
  - `sensor_msgs/Image`
  - `sensor_msgs/Imu`

**Diagram**: Sensor types on a humanoid robot

**Key Terms**:
- Point cloud
- Depth map
- Inertial measurement
- Sensor noise model
- Update rate / scan rate

### 2. LiDAR Simulation in Gazebo (Tutorial)

**Content**:
- GPU LiDAR sensor in Gazebo Harmonic
- Configuration parameters:
  - Horizontal/vertical samples
  - Field of view (horizontal and vertical)
  - Range limits (min/max)
  - Update rate
- Noise model: Gaussian noise on range measurements
- Output: gz-transport topic → ros_gz_bridge → ROS 2 PointCloud2
- Common LiDAR configurations (Velodyne VLP-16, Ouster OS1)

**Code Examples**:
```xml
<!-- lidar-sensor.sdf -->
<!-- LiDAR configuration in robot model -->
```

```yaml
# ros-gz-lidar-bridge.yaml
# Bridge configuration for LiDAR
```

**Diagram**: LiDAR scan pattern (horizontal rays, vertical layers)

**Hands-on**: Add a LiDAR to the humanoid robot head and visualize in RViz2

### 3. Depth Camera Simulation (Tutorial)

**Content**:
- Gazebo RGBD camera sensor
- Unity depth camera implementation
- Intrinsic parameters:
  - Focal length
  - Principal point
  - Distortion coefficients (k1, k2, p1, p2)
- Depth encoding: 16-bit, 32-bit float
- Near/far clipping planes
- Noise models for depth sensors

**Code Examples (Gazebo)**:
```xml
<!-- depth-camera.sdf -->
<!-- RGBD camera configuration -->
```

```yaml
# ros-gz-camera-bridge.yaml
# Bridge for RGB + Depth images
```

**Code Examples (Unity)**:
```csharp
// DepthCameraController.cs
// Depth rendering and ROS publishing
```

**Diagram**: Depth camera projection model (pinhole camera)

**Hands-on**: Configure a depth camera and measure distance to a known object

### 4. IMU Simulation (Tutorial)

**Content**:
- IMU sensor in Gazebo: acceleration + angular velocity + orientation
- Unity IMU via Rigidbody component
- Sensor frame conventions
- Noise models:
  - Gaussian noise on measurements
  - Bias (constant offset)
  - Random walk (time-varying bias)
- Gravity compensation in acceleration
- Orientation representations: quaternion vs Euler

**Code Examples (Gazebo)**:
```xml
<!-- imu-sensor.sdf -->
<!-- IMU configuration with noise -->
```

**Code Examples (Unity)**:
```csharp
// IMUSensor.cs
// Rigidbody-based IMU with noise
```

**Diagram**: IMU sensor axes and measurements

**Hands-on**: Add an IMU to the robot torso and verify readings match motion

### 5. Sensor Data Visualization (Tutorial)

**Content**:
- RViz2 displays for each sensor type:
  - PointCloud2 display for LiDAR
  - Image and Camera displays for depth/RGB
  - IMU display (with imu_tools or custom)
- Color mapping for point clouds (intensity, height, range)
- Depth image color mapping
- Orientation visualization (axes, path)
- Recording sensor data with rosbag2

**Code Examples**:
```yaml
# rviz-sensor-config.yaml
# RViz configuration for all sensors
```

**Diagram**: RViz2 sensor visualization layout

**Hands-on**: Create an RViz2 config showing all three sensors simultaneously

### 6. Building a Complete Sensor Suite (Tutorial)

**Content**:
- Multi-sensor robot configuration
- Sensor mounting positions and transforms
- TF tree for sensor frames
- Sensor fusion concepts (preview for later modules)
- Performance considerations:
  - GPU vs CPU sensors
  - Update rate impact on system load
  - Point cloud size management
- Common issues and debugging

**Code Examples**:
```xml
<!-- multi-sensor-robot.sdf -->
<!-- Robot with LiDAR, depth camera, and IMU -->
```

```python
# sensor_launch.py
# Launch file for all sensors and bridges
```

**Diagram**: TF tree for multi-sensor robot

**Hands-on**: Create a complete sensor suite and verify all data in RViz2

## Exercises

1. **LiDAR Configuration**: Add a 16-beam LiDAR to the robot's head. Configure it with 360° horizontal FOV, 30° vertical FOV, 0.1-30m range, and 10Hz update rate. Visualize the point cloud in RViz2.

2. **Depth Camera Setup**:
   - Add a depth camera to the robot's torso
   - Configure 640x480 resolution, 60° horizontal FOV
   - Measure the distance to an object at 2m and verify accuracy

3. **IMU Validation**:
   - Add an IMU to the robot's pelvis
   - Move the robot and verify acceleration/angular velocity readings
   - Apply a constant force and verify expected acceleration

4. **Complete Sensor Suite**:
   - Combine all three sensors on one robot
   - Create a launch file that starts all bridges
   - Create an RViz2 config showing all sensor data
   - Record 10 seconds of sensor data with rosbag2

## Assessment Questions

1. What parameters affect the density of a LiDAR point cloud? How would you configure a LiDAR for outdoor navigation vs indoor manipulation?

2. Explain the difference between depth camera intrinsic and extrinsic parameters. Which would you need to change if mounting the camera at a different location?

3. Why is it important to model IMU bias in simulation? How does bias affect state estimation algorithms?

4. How does Gaussian noise in simulation differ from real sensor noise? What other noise characteristics might be important?

5. What is the TF tree and why is it important for multi-sensor robots?

## Code Files Delivered

| File | Description | Location |
|------|-------------|----------|
| `lidar-sensor.sdf` | LiDAR configuration snippet | `examples/gazebo/sensors/` |
| `depth-camera.sdf` | Depth camera configuration | `examples/gazebo/sensors/` |
| `imu-sensor.sdf` | IMU configuration | `examples/gazebo/sensors/` |
| `multi-sensor-robot.sdf` | Complete sensor suite | `examples/gazebo/models/` |
| `ros-gz-sensor-bridge.yaml` | All sensor bridges | `examples/gazebo/config/` |
| `DepthCameraController.cs` | Unity depth camera | `examples/unity/Assets/Scripts/Sensors/` |
| `IMUSensor.cs` | Unity IMU | `examples/unity/Assets/Scripts/Sensors/` |
| `LiDARSimulator.cs` | Unity LiDAR (raycast) | `examples/unity/Assets/Scripts/Sensors/` |
| `rviz-sensor-config.yaml` | RViz visualization | `examples/ros2_ws/config/` |
| `sensor_launch.py` | Sensor launch file | `examples/ros2_ws/launch/` |

## Diagrams Required

1. **Sensor Types**: Humanoid robot with sensor locations labeled
2. **LiDAR Scan Pattern**: Horizontal and vertical ray distribution
3. **Pinhole Camera Model**: Depth projection geometry
4. **IMU Axes**: Sensor frame and measurement directions
5. **TF Tree**: Multi-sensor frame relationships
6. **RViz Layout**: Recommended visualization configuration

## References

- [Gazebo Sensors Documentation](https://gazebosim.org/api/sensors/8/index.html)
- [ROS 2 Sensor Messages](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs)
- [ros_gz_bridge Documentation](https://github.com/gazebosim/ros_gz)
- [Unity Perception Package](https://docs.unity3d.com/Packages/com.unity.perception@latest)
- [REP-103: Standard Units of Measure](https://www.ros.org/reps/rep-0103.html)
- [REP-145: Conventions for IMU Sensor Drivers](https://www.ros.org/reps/rep-0145.html)
