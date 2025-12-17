---
id: module-2-chapter-4
title: "URDF to SDF Conversion"
sidebar_label: "2.4 URDF to SDF Conversion"
sidebar_position: 4
description: "Convert ROS 2 URDF robot descriptions to Gazebo SDF format, handle inertial properties, configure Gazebo-specific tags, and maintain cross-compatibility"
keywords: [urdf, sdf, conversion, robot-description, inertial-properties, gazebo-tags]
estimated_time: 60
prerequisites:
  - module-1-chapter-4
  - module-2-chapter-2
learning_outcomes:
  - Convert URDF files to SDF format for Gazebo
  - Configure inertial properties for accurate physics
  - Add Gazebo-specific tags to URDF
  - Troubleshoot common conversion issues
  - Maintain single source-of-truth for robot descriptions
hardware_tier: proxy
---

## Chapter 2.4: URDF to SDF Conversion

### Overview

Bridging the gap between **ROS 2’s URDF (Unified Robot Description Format)** and **Gazebo’s SDF (Simulation Description Format)** is a critical step in building reliable digital twins for humanoid robots. While URDF is excellent for describing robot structure and kinematics in ROS, Gazebo relies on SDF for accurate physics, sensors, and simulation-specific features. This chapter explains how to convert URDF to SDF, why the conversion is needed, and how to ensure physically realistic simulation results.

---

### 2.4.1 URDF vs SDF: Key Differences

Before conversion, it is important to understand why URDF alone is not sufficient for full simulation.

**URDF characteristics:**

* Focuses on links, joints, visuals, and collisions
* Designed mainly for ROS-based kinematics and control
* Limited support for advanced physics and sensors

**SDF characteristics:**

* Native format for Gazebo
* Supports detailed physics properties
* Handles sensors, plugins, and world interactions
* Allows multiple models and environments

**Why conversion is needed:**
URDF defines *what the robot looks like and how it moves*, while SDF defines *how the robot behaves physically* inside a simulated world.

---

### 2.4.2 URDF to SDF Conversion Workflow

The typical conversion pipeline is:

```
URDF (ROS 2)
   ↓
URDF Parser
   ↓
SDF Model
   ↓
Gazebo Simulation
```

This process ensures that the robot designed for ROS can be simulated accurately in Gazebo.

---

### 2.4.3 Conversion Tools and Methods

#### 1. Gazebo Automatic Conversion

Gazebo can automatically convert URDF to SDF when spawning a robot:

```bash
gazebo --verbose robot.urdf
```

or using ROS:

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity humanoid \
  -file robot.urdf
```

**Pros:**

* Fast and simple
* No manual SDF editing

**Cons:**

* Limited control over physics parameters
* May ignore advanced Gazebo extensions

---

#### 2. `gz sdf -p` Command (Recommended)

To explicitly convert and preview the SDF output:

```bash
gz sdf -p robot.urdf > robot.sdf
```

This method allows inspection and manual refinement of the generated SDF file.

---

#### 3. Manual SDF Refinement

After conversion, engineers often refine the SDF to:

* Tune physics
* Add sensors
* Improve stability

This is especially important for humanoid robots, where balance and contact forces are critical.

---

### 2.4.4 Inertial Property Calculation

Accurate inertial parameters are essential for stable simulation.

Each link must define:

* **Mass**
* **Center of Mass (CoM)**
* **Inertia Matrix**

**URDF inertial block example:**

```xml
<inertial>
  <mass value="2.5"/>
  <origin xyz="0 0 0.1"/>
  <inertia ixx="0.02" iyy="0.02" izz="0.03"
           ixy="0" ixz="0" iyz="0"/>
</inertial>
```

**Best practices:**

* Avoid zero or extremely small masses
* Keep inertia values consistent with link geometry
* Use CAD tools or inertia calculators when possible

Poor inertial data is a common cause of simulation instability.

---

### 2.4.5 Gazebo-Specific Extensions

SDF allows features that URDF does not natively support.

#### Sensors

Examples include:

* Cameras
* IMU
* Force/Torque sensors
* LiDAR

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
</sensor>
```

---

#### Plugins

Plugins connect the simulated robot to ROS 2 controllers.

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so"/>
```

Plugins are essential for humanoid walking, balance control, and feedback loops.

---

### 2.4.6 Common Conversion Issues and Fixes

| Issue               | Cause                  | Solution                     |
| ------------------- | ---------------------- | ---------------------------- |
| Robot falls apart   | Incorrect inertia      | Recalculate mass and inertia |
| Unstable walking    | Joint limits missing   | Define limits and damping    |
| Sensors not working | Missing SDF tags       | Add Gazebo sensor blocks     |
| Collision glitches  | Bad collision geometry | Simplify collision meshes    |

---

### 2.4.7 Best Practices for Humanoid Digital Twins

* Always validate URDF before conversion
* Convert URDF → SDF early in development
* Tune inertia and friction parameters
* Test individual limbs before full-body simulation
* Maintain a clean separation between URDF (structure) and SDF (physics)

---

### Summary

URDF to SDF conversion is a foundational step in building a humanoid robot digital twin. While URDF defines structure and kinematics for ROS 2, SDF enables realistic physics, sensors, and Gazebo integration. By using proper conversion tools, validating inertial properties, and leveraging Gazebo-specific extensions, developers can achieve stable and high-fidelity humanoid simulations.

In the next chapter, we build on this foundation by integrating sensors and controllers into the converted SDF model for closed-loop simulation.
