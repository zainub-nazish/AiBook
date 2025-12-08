# Quickstart: Module 2 Development Environment

**Feature**: 002-digital-twin-simulation
**Date**: 2025-12-08
**Purpose**: Set up development environment for writing and testing Module 2 content.

## Prerequisites

Before starting Module 2 development, ensure you have:

1. **Module 1 Environment** - ROS 2 Humble workspace from Module 1
2. **Docusaurus** - Book build system (already configured)
3. **Git** - Version control

## Gazebo Harmonic Setup

### Installation (Ubuntu 22.04)

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic

# Install ROS 2 Gazebo integration
sudo apt install ros-humble-ros-gz
```

### Verify Installation

```bash
# Check Gazebo version
gz sim --version
# Expected: Gazebo Sim, version 8.x.x

# Launch empty world
gz sim empty.sdf

# Test ROS 2 bridge
ros2 run ros_gz_bridge parameter_bridge --help
```

## Unity Setup

### Installation

1. **Install Unity Hub**: Download from [unity.com/download](https://unity.com/download)

2. **Install Unity 2022 LTS**:
   - Open Unity Hub → Installs → Add
   - Select Unity 2022.3.x LTS
   - Include modules: Linux Build Support (if on Linux)

3. **Create New Project**:
   - Open Unity Hub → Projects → New Project
   - Select "3D" template
   - Name: `digital-twin-simulation`

### Add Robotics Packages

1. Open project in Unity Editor
2. Window → Package Manager
3. Click "+" → Add package from git URL
4. Add these packages:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

### Verify Unity Setup

1. Create empty GameObject
2. Add Component → search "ROS Connection"
3. Should find `ROSConnection` script from package

## ROS 2 Workspace Extension

Extend your Module 1 workspace with simulation packages:

```bash
cd ~/ros2_ws/src

# Create simulation package
ros2 pkg create --build-type ament_python digital_twin_examples \
  --dependencies rclpy sensor_msgs geometry_msgs

# Create directories
mkdir -p digital_twin_examples/config
mkdir -p digital_twin_examples/launch
mkdir -p digital_twin_examples/worlds
mkdir -p digital_twin_examples/urdf
```

### Package Structure

```text
digital_twin_examples/
├── package.xml
├── setup.py
├── config/
│   ├── ros_gz_bridge.yaml
│   └── rviz_sensors.yaml
├── launch/
│   ├── gazebo_world.launch.py
│   ├── unity_bridge.launch.py
│   └── sensor_suite.launch.py
├── worlds/
│   ├── minimal_world.sdf
│   └── humanoid_world.sdf
├── urdf/
│   └── (linked from Module 1)
└── digital_twin_examples/
    ├── __init__.py
    └── sensor_publisher.py
```

### Build Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select digital_twin_examples
source install/setup.bash
```

## Docusaurus Content Location

Module 2 content goes in:

```text
book/docs/module-02-digital-twin/
├── _category_.json
├── index.md
├── chapter-01-gazebo-physics.mdx
├── chapter-02-unity-rendering.mdx
├── chapter-03-sensor-simulation.mdx
└── assets/
    ├── diagrams/
    └── code/
```

### Create Module Directory

```bash
cd book/docs
mkdir -p module-02-digital-twin/assets/diagrams
mkdir -p module-02-digital-twin/assets/code
```

### Create Category File

```json
// book/docs/module-02-digital-twin/_category_.json
{
  "label": "Module 2: The Digital Twin",
  "position": 3,
  "link": {
    "type": "doc",
    "id": "module-02-digital-twin/index"
  },
  "collapsible": true,
  "collapsed": true
}
```

## Development Workflow

### Writing Content

1. Create/edit `.mdx` files in `book/docs/module-02-digital-twin/`
2. Preview locally:
   ```bash
   cd book
   npm run start
   ```
3. Browser opens at http://localhost:3000

### Testing Code Examples

**Gazebo Examples**:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch Gazebo with world file
gz sim worlds/humanoid_world.sdf

# In another terminal, launch ROS bridge
ros2 launch digital_twin_examples gazebo_world.launch.py
```

**Unity Examples**:
```bash
# Terminal 1: Start ROS TCP Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: Open Unity and press Play
# Verify connection in ROS TCP Endpoint output
```

### Validating Content

```bash
# Build Docusaurus
cd book
npm run build

# Check for broken links
npx linkinator ./build --recurse

# Validate SDF files (Gazebo)
gz sdf -p worlds/humanoid_world.sdf

# Validate URDF (if modified)
check_urdf urdf/humanoid_robot.urdf
```

## Environment Variables

Add to `~/.bashrc`:

```bash
# ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Gazebo
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src/digital_twin_examples/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib

# Unity (optional, for CLI builds)
export UNITY_PATH="/path/to/Unity/Hub/Editor/2022.3.x/Editor/Unity"
```

## Troubleshooting

### Gazebo Won't Start

```bash
# Check OpenGL
glxinfo | grep "OpenGL version"

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gz sim empty.sdf
```

### Unity ROS Connection Fails

1. Check firewall allows port 10000
2. Verify ROS_IP is correct
3. Check ROS TCP Endpoint is running
4. Try `localhost` if same machine

### Bridge Topics Not Publishing

```bash
# List Gazebo topics
gz topic -l

# List ROS topics
ros2 topic list

# Check bridge config matches topic names
```

## Quick Reference

| Task | Command |
|------|---------|
| Start Gazebo | `gz sim world.sdf` |
| Launch ROS bridge | `ros2 launch ros_gz_bridge bridge.launch.py` |
| Preview book | `cd book && npm run start` |
| Build book | `cd book && npm run build` |
| Build ROS packages | `cd ~/ros2_ws && colcon build` |
| Validate SDF | `gz sdf -p file.sdf` |
| Record sensor data | `ros2 bag record -a` |
