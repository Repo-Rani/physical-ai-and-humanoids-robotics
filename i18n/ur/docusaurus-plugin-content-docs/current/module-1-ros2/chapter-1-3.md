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

# باب 1.3: ROS 2 پیکیجز کو پائیتھون کے ساتھ بنانا

یہ باب ROS 2 پیکیجز کو پائیتھون کے ساتھ بنانے کے لیے ضروری مہارتیں پر محیط ہے۔ آپ مناسب ڈائریکٹری ساخت، کنفیگریشن فائلز، بلڈ سسٹم، اور پیکیج ڈویلپمنٹ کے لیے بہترین طریقوں کے بارے میں سیکھیں گے۔

## ROS 2 پیکیج ساخت

ایک مناسب ساخت والا ROS 2 پیکیج ایک معیاری ڈائریکٹری لی آؤٹ پر عمل کرتا ہے جو آسان شیئرنگ، بلڈنگ، اور مینٹیننس کو فعال بناتا ہے:

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

## package.xml فائل بنانا

`package.xml` فائل آپ کے پیکیج اور اس کی ڈپینڈنسیز کے بارے میں میٹا ڈیٹا رکھتی ہے۔ یہ فائل ROS 2 بلڈ سسٹم اور پیکیج مینیجر کے لیے ضروری ہے۔

### بنیادی package.xml ساخت

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

### اہم عناصر کی وضاحت

- **`<name>`**: پیکیج کے لیے منفرد شناخت کنندہ
- **`<version>`**: سیمیٹک ورژننگ (MAJOR.MINOR.PATCH)
- **`<description>`**: پیکیج فنکشنلٹی کا مختصر بیان
- **`<maintainer>`**: پیکیج مینٹینر کے لیے رابطہ کی معلومات
- **`<license>`**: سافٹ ویئر لائسنس (ROS 2 کے لیے Apache-2.0 تجویز کیا جاتا ہے)
- **`<exec_depend>`**: پیکیج کے ذریعہ درکار رن ٹائم ڈپینڈنسیز
- **`<test_depend>`**: ٹیسٹنگ کے لیے صرف درکار ڈپینڈنسیز
- **`<export>`**: بلڈ ٹائپ کی وضاحت کرتا ہے (خالص پائیتھون پیکیجز کے لیے ament_python)

## پائیتھون پیکیجز کے لیے setup.py بنانا

`setup.py` فائل اس بات کو کنفیگر کرتی ہے کہ آپ کا پائیتھون پیکیج کیسے انسٹال ہوتا ہے اور ایکزیکیوٹبلز کے لیے انٹری پوئنٹس فراہم کرتا ہے۔

### بنیادی setup.py ساخت

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

### انٹری پوئنٹس کنفیگریشن

`console_scripts` سیکشن انسٹالیشن کے بعد دستیاب ایکزیکیوٹبل کمانڈز کی وضاحت کرتا ہے:

```python
entry_points={
    'console_scripts': [
        'command_name = package.module:function',
    ],
}
```

یہ ایک کمانڈ لائن ایکزیکیوٹبل بناتا ہے جو اس طرح چلایا جا سکتا ہے:
```bash
ros2 run my_robot_package command_name
```

## Colcon بلڈ سسٹم کا استعمال

Colcon ROS 2 میں پیکیجز کو بلڈ اور انسٹال کرنے کے لیے استعمال ہونے والا بلڈ سسٹم ہے۔

### بنیادی Colcon کمانڈز

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

### مخصوص پیکیجز کو بلڈ کرنا

```bash
# Build only specific packages
colcon build --packages-select my_robot_package

# Build with additional options
colcon build --packages-select my_robot_package --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### بلڈ پروسیس کو سمجھنا

1. **ڈسکووری فیز**: Colcon `src` ڈائریکٹری کو پیکیجز تلاش کرنے کے لیے اسکین کرتا ہے
2. **ڈپینڈنسی ریزولیوشن**: پیکیج ڈپینڈنسیز کے بنیاد پر بلڈ آرڈر کا تعین کرتا ہے
3. **بلڈ فیز**: کوڈ کو کمپائل کرتا ہے اور کنفیگریشن فائلز کو پروسیس کرتا ہے
4. **انسٹال فیز**: بلڈ آرٹیفیکٹس کو `install` ڈائریکٹری میں کاپی کرتا ہے
5. **اوورلے فیز**: بلڈ شدہ پیکیجز کو ماحول میں سورس کرتا ہے

## rosdep کے ساتھ ڈپینڈنسی مینجمنٹ

rosdep ایک ٹول ہے جو ROS پیکیجز کے لیے سسٹم ڈپینڈنسیز کو مینج کرنے میں مدد کرتا ہے۔

### rosdep کمانڈز

```bash
# Install dependencies for all packages in workspace
rosdep install --from-paths src --ignore-src -r -y

# Install dependencies for a specific package
rosdep install --from-paths src/my_robot_package --ignore-src -r -y
```

### package.xml میں ڈپینڈنسیز شامل کرنا

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rclpy</build_depend>
<exec_depend>std_msgs</exec_depend>
<test_depend>ament_copyright</test_depend>
```

## سسٹم مینجمنٹ کے لیے لانچ فائلز

لانچ فائلز آپ کو ایک ہی کمانڈ کے ساتھ متعدد نوڈز شروع کرنے کی اجازت دیتی ہیں، جس سے سسٹم مینجمنٹ بہت آسان ہو جاتا ہے۔

### پائیتھون لانچ فائل مثال

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

### آرگیومنٹس کے ساتھ لانچ فائل

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

## کنفیگریشن مینجمنٹ

کنفیگریشن فائلز آپ کو کوڈ کو دوبارہ کمپائل کیے بغیر نوڈ بیہیویئر کو تبدیل کرنے کی اجازت دیتی ہیں۔

### YAML کنفیگریشن مثال

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

### کوڈ میں پیرامیٹرز لوڈ کرنا

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

## ROS 2 پیکیجز کا ٹیسٹنگ

ٹیسٹنگ روبوٹک سسٹمز کی اعتمادیت کو یقینی بنانے کے لیے ضروری ہے۔

### pytest کے ساتھ یونٹ ٹیسٹنگ

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

### انٹیگریشن ٹیسٹنگ

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

## ورک اسپیس آرگنائزیشن کے بہترین طریقے

### متعدد پیکیج ورک اسپیس

مختلف پراجیکٹس کے لیے، آپ کے پاس متعدد پیکیجز ہو سکتے ہیں:

```
robot_project_ws/
├── src/
│   ├── robot_control/
│   ├── robot_sensors/
│   ├── robot_navigation/
│   └── robot_interfaces/
```

### ورک اسپیسز کو اوورلے کرنا

آپ متعدد ورک اسپیسز کو پیکیجز کو جوڑنے کے لیے اوورلے کر سکتے ہیں:

```bash
# Source system installation first
source /opt/ros/humble/setup.bash

# Then source your custom workspace
source ~/robot_project_ws/install/setup.bash

# Then source your development workspace
source ~/robot_dev_ws/install/setup.bash
```

## عملی مشق: ایک مکمل پیکیج بنانا

ایک مکمل ROS 2 پیکیج بنائیں جو شامل کرے:

1. تمام ضروری فائلز کے ساتھ مناسب ڈائریکٹری ساخت
2. ایک پبلشر نوڈ جو روبوٹ کی حالت پبلش کرتا ہے
3. ایک سبسکرائبر نوڈ جو سینسر ڈیٹا کو پروسیس کرتا ہے
4. دونوں نوڈز کو شروع کرنے کے لیے ایک لانچ فائل
5. پیرامیٹرز کے ساتھ ایک کنفیگریشن فائل
6. بنیادی یونٹ ٹیسٹس

یہ مشق آپ کو ROS 2 پیکیجز بنانے اور مینج کرنے کے لیے مکمل ورک فلو کو سمجھنے میں مدد کرے گی۔

## اگلا باب

اگلے باب میں، [باب 1.4: URDF: روبوٹ کی ساخت کا بیان](./chapter-1-4.md)، آپ URDF (Unified Robot Description Format) کے بارے میں سیکھیں گے جو روبوٹ کی کنیمیٹکس، ڈائنامکس، اور بصری خصوصیات کا بیان کرتا ہے۔