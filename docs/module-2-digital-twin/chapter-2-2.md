---
id: module-2-chapter-2
title: "Gazebo Fundamentals"
sidebar_label: "2.2 Gazebo Fundamentals"
sidebar_position: 2
description: "Gazebo simulator fundamentals: world creation, SDF format, model integration, physics configuration, and ROS 2 integration for robotic simulation"
keywords: [gazebo, sdf, world-files, physics-configuration, ros2-integration, simulation]
estimated_time: 90
prerequisites:
  - module-2-chapter-1
  - module-1-chapter-4
learning_outcomes:
  - Create Gazebo world files using SDF format
  - Configure physics engines and simulation parameters
  - Spawn and control robots in Gazebo
  - Integrate Gazebo with ROS 2 using ros_gz_bridge
  - Troubleshoot common Gazebo simulation issues
hardware_tier: miniature
---

# Chapter 2.2: Gazebo Fundamentals

Gazebo is the most widely-used open-source robotics simulator, providing a robust platform for testing ROS 2 applications. This chapter covers the fundamentals of working with Gazebo, from creating simulation environments to integrating with ROS 2.

## Gazebo Architecture

Gazebo consists of several key components:

- **Gazebo Server (gzserver)**: Physics simulation and sensor generation
- **Gazebo Client (gzclient)**: Visualization and GUI
- **SDF Models**: Robot and environment descriptions
- **Plugins**: Extend functionality (sensors, actuators, custom behaviors)

### Gazebo Classic vs. Gazebo (Harmonic)

| Feature | Gazebo Classic | Gazebo (Harmonic) |
|---------|----------------|-------------------|
| Release | Legacy (ROS 1/2) | Modern (ROS 2) |
| Physics | ODE, Bullet, DART | DART (default) |
| Rendering | OGRE 1.x | OGRE 2.x, Optix |
| Format | SDF 1.6 | SDF 1.8+ |
| ROS 2 Integration | gazebo_ros_pkgs | ros_gz |
| Performance | Moderate | Improved |

**Recommendation**: Use Gazebo Harmonic for new projects.

## SDF (Simulation Description Format)

SDF is an XML-based format for describing simulation environments.

### Basic World File Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Model Description

Models represent objects in the simulation:

```xml
<model name="simple_box">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="box_link">
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.083</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.083</iyy>
        <iyz>0.0</iyz>
        <izz>0.083</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Physics Configuration

Physics engines determine how objects interact in simulation.

### Physics Parameters

```xml
<physics name="1ms" type="dart">
  <!-- Time step for simulation updates -->
  <max_step_size>0.001</max_step_size>

  <!-- Target real-time factor (1.0 = real-time) -->
  <real_time_factor>1</real_time_factor>

  <!-- Maximum number of contacts per collision -->
  <max_contacts>10</max_contacts>

  <!-- Solver configuration -->
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

### Common Physics Issues

**Issue**: Robot falls through ground plane
- **Cause**: Collision geometry missing or incorrect
- **Solution**: Verify collision elements match visual geometry

**Issue**: Simulation runs slower than real-time
- **Cause**: Physics too complex or time step too small
- **Solution**: Simplify collision meshes, increase time step

**Issue**: Unstable behavior (jittering, bouncing)
- **Cause**: Stiff constraints, inappropriate contact parameters
- **Solution**: Adjust damping, friction, contact stiffness

## Spawning Robots in Gazebo

### Method 1: Include in World File

```xml
<world name="robot_world">
  <include>
    <uri>model://my_robot</uri>
    <pose>0 0 0.5 0 0 0</pose>
  </include>
</world>
```

### Method 2: Spawn from ROS 2

```bash
# Using spawn_entity.py from ros_gz_sim
ros2 run ros_gz_sim spawn_entity.py \
    -name my_robot \
    -file /path/to/model.sdf \
    -x 0 -y 0 -z 0.5
```

### Method 3: Launch File Integration

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            'path/to/gazebo.launch.py',
            launch_arguments={'world': 'my_world.sdf'}.items()
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', 'model.sdf']
        )
    ])
```

## ROS 2 Integration with ros_gz_bridge

The bridge enables communication between Gazebo and ROS 2.

### Installing ros_gz

```bash
sudo apt update
sudo apt install ros-humble-ros-gz
```

### Bridging Topics

```bash
# Bridge a single topic
ros2 run ros_gz_bridge parameter_bridge \
    /model/my_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# Bridge multiple topics (create a YAML config)
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:=bridge_config.yaml
```

### Bridge Configuration File

```yaml
# bridge_config.yaml
- topic_name: "/model/my_robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- topic_name: "/model/my_robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- topic_name: "/model/my_robot/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

## Gazebo Plugins

Plugins extend Gazebo's functionality for sensors and actuators.

### Common Plugin Types

- **Sensor Plugins**: Camera, LiDAR, IMU, GPS
- **Actuator Plugins**: Differential drive, joint controllers
- **World Plugins**: Custom physics, environmental effects
- **Model Plugins**: Robot-specific behaviors

### Example: Differential Drive Plugin

```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <ros>
    <namespace>/robot</namespace>
  </ros>
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.2</wheel_diameter>
  <max_wheel_torque>20</max_wheel_torque>
  <max_wheel_acceleration>1.0</max_wheel_acceleration>

  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <publish_wheel_tf>true</publish_wheel_tf>

  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

## Practical Exercise: Create a Gazebo World

Create a custom Gazebo environment:

1. **Create World File**: Define ground, lighting, and obstacles
2. **Add Physics**: Configure DART physics engine with 1ms time step
3. **Spawn Robot**: Use your URDF from Module 1
4. **Bridge Topics**: Connect cmd_vel and sensor data to ROS 2
5. **Test Control**: Drive the robot using teleop

**Validation**:
- [ ] World loads without errors
- [ ] Robot spawns at correct position
- [ ] Topics bridge successfully (check with `ros2 topic list`)
- [ ] Robot responds to velocity commands

## Troubleshooting Guide

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Gazebo crashes on launch | Missing model dependencies | Check model paths in `GZ_SIM_RESOURCE_PATH` |
| Robot not visible | Incorrect model URI | Verify `<uri>` in world file |
| No ROS 2 topics | Bridge not running | Launch ros_gz_bridge with correct config |
| Physics unstable | Time step too large | Reduce `<max_step_size>` to 0.001 |
| Slow simulation | Complex collision meshes | Simplify geometry or use primitive shapes |

## Next Chapter

In [Chapter 2.3: Sensor Simulation](./chapter-2-3.md), you'll learn to simulate cameras, LiDAR, and IMUs with realistic noise models for perception algorithm development.
