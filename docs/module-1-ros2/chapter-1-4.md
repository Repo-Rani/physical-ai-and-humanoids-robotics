---
id: module-1-chapter-4
title: "URDF: Describing Robot Structure"
sidebar_label: "1.4 URDF Robot Description"
sidebar_position: 4
description: "Learn URDF syntax for robot structure: links, joints, kinematic chains, Xacro macros, collision vs visual geometry, and robot visualization in RViz2"
keywords: [urdf, xacro, robot-description, links, joints, kinematics, rviz2]
estimated_time: 90
prerequisites:
  - module-1-chapter-3
  - basic-robotics-concepts
learning_outcomes:
  - Create robot descriptions using URDF syntax for links and joints
  - Implement Xacro macros for reusable robot components
  - Distinguish between collision and visual geometry in robot models
  - Visualize robot models in RViz2 with joint state publishers
  - Validate URDF models and debug common errors
hardware_tier: proxy
---

# Chapter 1.4: URDF: Describing Robot Structure

URDF (Unified Robot Description Format) is the standard for describing robot models in ROS. This chapter covers the fundamentals of creating robot descriptions, from basic link and joint definitions to complex kinematic chains and visualization.

## Understanding URDF Structure

URDF is an XML-based format that describes a robot's physical and visual properties. A URDF file contains:

- **Links**: Rigid bodies that make up the robot
- **Joints**: Connections between links that allow relative motion
- **Visual**: How the robot appears in simulation and visualization
- **Collision**: How the robot interacts with the physical world
- **Inertial**: Mass properties for physics simulation

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Link Elements

Links represent rigid bodies in the robot. Each link can have multiple child elements that define its properties.

### Visual Properties

The `<visual>` element defines how a link appears in visualization and simulation:

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Choose one geometry type -->
      <box size="1 2 3"/>
      <!-- OR -->
      <cylinder radius="0.1" length="0.5"/>
      <!-- OR -->
      <sphere radius="0.1"/>
      <!-- OR -->
      <mesh filename="package://my_robot/meshes/link_name.dae"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
</link>
```

### Collision Properties

The `<collision>` element defines how the link interacts with the physical world:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Often simplified compared to visual geometry for performance -->
    <box size="1 2 3"/>
  </geometry>
</collision>
```

### Inertial Properties

The `<inertial>` element defines the physical properties for dynamics simulation:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.4"/>
</inertial>
```

For a solid sphere: `ixx = iyy = izz = 0.4 * mass * radius²`
For a solid cylinder (along z-axis): `ixx = iyy = mass * (3*radius² + length²) / 12`, `izz = 0.5 * mass * radius²`

## Joint Elements

Joints define the connection between links and specify the allowed motion.

### Joint Types

```xml
<!-- Fixed joint (no motion) -->
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>

<!-- Revolute joint (single axis rotation) -->
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>

<!-- Continuous joint (unlimited rotation) -->
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>

<!-- Prismatic joint (linear motion) -->
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
</joint>

<!-- Floating joint (6 DOF) -->
<joint name="floating_joint" type="floating">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

## Xacro for Reusable Robot Models

Xacro (XML Macros) allows you to create parameterized, reusable robot descriptions using macros and properties.

### Basic Xacro Structure

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="robot_radius" value="0.1" />
  <xacro:property name="robot_height" value="0.3" />

  <!-- Define a macro for a wheel -->
  <xacro:macro name="wheel" params="prefix parent x y z">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x} ${y} ${z}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.05"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create wheels -->
  <xacro:wheel prefix="front_left" parent="base_link" x="0.2" y="0.2" z="-0.1"/>
  <xacro:wheel prefix="front_right" parent="base_link" x="0.2" y="-0.2" z="-0.1"/>
  <xacro:wheel prefix="rear_left" parent="base_link" x="-0.2" y="0.2" z="-0.1"/>
  <xacro:wheel prefix="rear_right" parent="base_link" x="-0.2" y="-0.2" z="-0.1"/>

  <!-- Base link definition -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="${robot_radius}" length="${robot_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${robot_radius}" length="${robot_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

</robot>
```

### Advanced Xacro Features

```xml
<!-- Conditional statements -->
<xacro:if value="true">
  <link name="conditional_link"/>
</xacro:if>

<!-- Include other Xacro files -->
<xacro:include filename="$(find my_robot_description)/urdf/sensors.urdf.xacro"/>

<!-- Mathematical expressions -->
<xacro:property name="wheel_diameter" value="2 * 0.05" />
<xacro:property name="half_wheel_diameter" value="${wheel_diameter / 2}" />
```

## Collision vs Visual Geometry

Understanding the difference between collision and visual geometry is crucial for performance and accuracy:

### Visual Geometry
- Defines how the robot appears in RViz2 and simulation
- Can include detailed meshes for realistic appearance
- Used for rendering and visualization
- Can be more complex than collision geometry

### Collision Geometry
- Defines how the robot interacts with the physical world
- Should be simplified for performance
- Used for collision detection and physics simulation
- Often uses primitive shapes (boxes, cylinders, spheres)

## Robot Visualization in RViz2

RViz2 is the 3D visualization tool for ROS 2 that can display robot models.

### Launching Robot Visualization

```bash
# Launch robot state publisher to broadcast transforms
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat my_robot.urdf)

# Launch RViz2
ros2 run rviz2 rviz2
```

### Essential RViz2 Plugins for Robot Visualization

1. **RobotModel**: Displays the robot model using URDF
2. **TF**: Shows coordinate frames and their relationships
3. **JointState**: Displays joint positions (requires joint_state_publisher)

### Joint State Publisher for Animation

To see the robot move in RViz2, you need to publish joint states:

```bash
# Interactive joint state publisher
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Or non-interactive
ros2 run joint_state_publisher joint_state_publisher
```

## Validating URDF Models

Several tools help validate and debug URDF models:

### Command Line Validation

```bash
# Check URDF syntax
check_urdf my_robot.urdf

# View robot tree structure
urdf_to_graphiz my_robot.urdf

# Parse and display robot information
ros2 run xacro xacro my_robot.urdf.xacro
```

### Common URDF Errors and Solutions

1. **Missing parent/child links**: Ensure all joints reference existing links
2. **Invalid geometry**: Check geometry parameters (e.g., radius > 0)
3. **Duplicate names**: All links and joints must have unique names
4. **Disconnected components**: All links must be connected through joints
5. **Invalid inertial values**: Ensure inertia matrix is positive definite

## Practical Exercise: Create a Simple Robot Model

Create a complete robot model with:

1. A base link with visual and collision geometry
2. Multiple joints connecting various components
3. Use Xacro macros to avoid duplication
4. Validate the model using check_urdf
5. Visualize the robot in RViz2

This exercise will help you understand the complete workflow for creating and validating robot models in URDF.

## Next Chapter

In the next chapter, [Chapter 1.5: Launch Files and Parameter Management](./chapter-1-5.md), you'll learn advanced techniques for managing complex robotic systems with launch files and parameters.