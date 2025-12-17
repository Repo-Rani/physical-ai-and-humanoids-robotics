---
id: module-1-chapter-3
title: "Building ROS 2 Packages with Python"
sidebar_label: "1.3 Building Packages with Python"
sidebar_position: 3
description: "Create ROS 2 packages with proper structure: package.xml, setup.py, launch files, configuration, and testing for Python-based robotic applications"
keywords: [ros2, packages, python, colcon, package-xml, setup-py, launch-files]
estimated_time: 90
prerequisites:
  - module-1-chapter-2
  - python-programming
learning_outcomes:
  - Create properly structured ROS 2 packages with package.xml and setup.py
  - Use Colcon build system for package management and building
  - Implement dependency management with rosdep
  - Organize code with proper launch files and configuration directories
  - Write and run tests for ROS 2 Python packages
hardware_tier: proxy
---

# Chapter 1.3: Building ROS 2 Packages with Python

This chapter covers the essential skills for creating well-structured ROS 2 packages using Python. You'll learn about the proper directory structure, configuration files, build system, and best practices for package development.

## ROS 2 Package Structure

A properly structured ROS 2 package follows a standard directory layout that enables easy sharing, building, and maintenance:

```
my_robot_package/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python package configuration
├── setup.cfg            # Installation configuration
├── CMakeLists.txt       # CMake build configuration (for mixed packages)
├── resource/            # Resource files for package discovery
├── launch/              # Launch files for starting multiple nodes
├── config/              # Configuration files (YAML, parameters)
├── test/                # Unit and integration tests
├── my_robot_package/    # Python source code
│   ├── __init__.py
│   ├── main.py
│   └── modules/
└── scripts/             # Standalone executable scripts
```

## Creating the package.xml File

The `package.xml` file contains metadata about your package and its dependencies. This file is essential for the ROS 2 build system and package manager.

### Basic package.xml Structure

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.1</version>
  <description>ROS 2 package for my robot functionality</description>
  <maintainer email="maintainer@todo.todo">Maintainer Name</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Key Elements Explained

- **`<name>`**: Unique identifier for the package
- **`<version>`**: Semantic versioning (MAJOR.MINOR.PATCH)
- **`<description>`**: Brief description of package functionality
- **`<maintainer>`**: Contact information for package maintainer
- **`<license>`**: Software license (Apache-2.0 recommended for ROS 2)
- **`<exec_depend>`**: Runtime dependencies required by the package
- **`<test_depend>`**: Dependencies only needed for testing
- **`<export>`**: Specifies the build type (ament_python for pure Python packages)

## Creating setup.py for Python Packages

The `setup.py` file configures how your Python package is installed and provides entry points for executables.

### Basic setup.py Structure

```python
from setuptools import setup
from glob import glob
import os

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@todo.todo',
    description='ROS 2 package for my robot functionality',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_package.robot_controller:main',
            'sensor_processor = my_robot_package.sensor_processor:main',
            'data_analyzer = my_robot_package.data_analyzer:main',
        ],
    },
)
```

### Entry Points Configuration

The `console_scripts` section defines executable commands that will be available after installation:

```python
entry_points={
    'console_scripts': [
        'command_name = package.module:function',
    ],
}
```

This creates a command-line executable that can be run as:
```bash
ros2 run my_robot_package command_name
```

## Using the Colcon Build System

Colcon is the build system used in ROS 2 for building and installing packages.

### Basic Colcon Commands

```bash
# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create or copy your package to the src directory
# Then build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace to use the built packages
source install/setup.bash
```

### Building Specific Packages

```bash
# Build only specific packages
colcon build --packages-select my_robot_package

# Build with additional options
colcon build --packages-select my_robot_package --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Understanding the Build Process

1. **Discovery Phase**: Colcon scans the `src` directory to find packages
2. **Dependency Resolution**: Determines build order based on package dependencies
3. **Build Phase**: Compiles code and processes configuration files
4. **Install Phase**: Copies built artifacts to the `install` directory
5. **Overlay Phase**: Sources the built packages into the environment

## Dependency Management with rosdep

rosdep is a tool that helps manage system dependencies for ROS packages.

### rosdep Commands

```bash
# Install dependencies for all packages in workspace
rosdep install --from-paths src --ignore-src -r -y

# Install dependencies for a specific package
rosdep install --from-paths src/my_robot_package --ignore-src -r -y
```

### Adding Dependencies to package.xml

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rclpy</build_depend>
<exec_depend>std_msgs</exec_depend>
<test_depend>ament_copyright</test_depend>
```

## Launch Files for System Management

Launch files allow you to start multiple nodes with a single command, making system management much easier.

### Python Launch File Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory('my_robot_package')

    # Include a configuration file
    config = os.path.join(package_dir, 'config', 'robot_config.yaml')

    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            parameters=[config],
            output='screen',
            respawn=True,  # Restart if the node dies
        ),
        Node(
            package='my_robot_package',
            executable='sensor_processor',
            name='sensor_processor',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='my_robot_package',
            executable='data_analyzer',
            name='data_analyzer',
            parameters=[config],
            output='screen',
        ),
    ])
```

### Launch File with Arguments

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace'
    )

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        namespace_arg,
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            namespace=namespace,
            output='screen',
        ),
    ])
```

## Configuration Management

Configuration files allow you to change node behavior without recompiling code.

### YAML Configuration Example

```yaml
# config/robot_config.yaml
/**:  # Applies to all nodes
  ros__parameters:
    robot_name: "my_robot"
    update_rate: 10.0
    safety_limits:
      max_velocity: 1.0
      max_acceleration: 2.0
    sensors:
      lidar_enabled: true
      camera_enabled: true
      imu_enabled: true

robot_controller:  # Applies only to robot_controller node
  ros__parameters:
    control_mode: "velocity"
    position_tolerance: 0.01
    velocity_tolerance: 0.05
```

### Loading Parameters in Code

```python
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('safety_limits.max_velocity', 1.0)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_velocity = self.get_parameter('safety_limits.max_velocity').value
```

## Testing ROS 2 Packages

Testing is crucial for ensuring the reliability of robotic systems.

### Unit Testing with pytest

```python
# test/test_robot_controller.py
import pytest
import rclpy
from my_robot_package.robot_controller import RobotController

def test_robot_controller_initialization():
    rclpy.init()
    try:
        controller = RobotController()
        assert controller is not None
        assert controller.robot_name == 'default_robot'
    finally:
        rclpy.shutdown()

def test_parameter_setting():
    rclpy.init()
    try:
        controller = RobotController()
        # Test parameter setting logic
        assert controller.update_rate == 10.0
    finally:
        rclpy.shutdown()
```

### Integration Testing

```python
# test/test_integration.py
import rclpy
from rclpy.executors import SingleThreadedExecutor
from my_robot_package.robot_controller import RobotController
from my_robot_package.sensor_processor import SensorProcessor

def test_sensor_controller_integration():
    rclpy.init()
    try:
        # Create nodes
        controller = RobotController()
        processor = SensorProcessor()

        # Create executor and add nodes
        executor = SingleThreadedExecutor()
        executor.add_node(controller)
        executor.add_node(processor)

        # Test integration (this would involve more complex logic)
        # For example: publish test messages and verify they're processed correctly

    finally:
        controller.destroy_node()
        processor.destroy_node()
        rclpy.shutdown()
```

## Workspace Organization Best Practices

### Multiple Package Workspaces

For complex projects, you might have multiple packages:

```
robot_project_ws/
├── src/
│   ├── robot_control/
│   ├── robot_sensors/
│   ├── robot_navigation/
│   └── robot_interfaces/
```

### Overlaying Workspaces

You can overlay multiple workspaces to combine packages:

```bash
# Source system installation first
source /opt/ros/humble/setup.bash

# Then source your custom workspace
source ~/robot_project_ws/install/setup.bash

# Then source your development workspace
source ~/robot_dev_ws/install/setup.bash
```

## Practical Exercise: Creating a Complete Package

Create a complete ROS 2 package that includes:

1. Proper directory structure with all necessary files
2. A publisher node that publishes robot state
3. A subscriber node that processes sensor data
4. A launch file to start both nodes
5. A configuration file with parameters
6. Basic unit tests

This exercise will help you understand the complete workflow for creating and managing ROS 2 packages.

## Next Chapter

In the next chapter, [Chapter 1.4: URDF: Describing Robot Structure](./chapter-1-4.md), you'll learn about URDF (Unified Robot Description Format) for describing robot kinematics, dynamics, and visual properties.