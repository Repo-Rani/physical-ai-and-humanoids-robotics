---
id: module-1-chapter-5
title: "Launch Files and Parameter Management"
sidebar_label: "1.5 Launch Files & Parameters"
sidebar_position: 5
description: "Master Python-based launch files, node composition, YAML parameter management, conditional logic, and event handlers for complex robotic system orchestration"
keywords: [ros2, launch-files, parameters, yaml, node-composition, event-handlers]
estimated_time: 90
prerequisites:
  - module-1-chapter-4
  - python-programming
learning_outcomes:
  - Create Python-based launch files for complex system orchestration
  - Implement node composition for efficient multi-node systems
  - Manage parameters with YAML configuration files
  - Use conditional logic and event handlers in launch files
  - Handle node lifecycle events and error recovery
hardware_tier: proxy
---

# باب 1.5: لانچ فائلز اور پیرامیٹر مینجمنٹ

لانچ فائلز پیچیدہ روبوٹک سسٹمز کو متعدد نوڈز کے ساتھ منیج کرنے کے لیے ضروری ہیں۔ یہ باب لانچ فائلز کی اعلیٰ تکنیکوں، پیرامیٹر مینجمنٹ، اور سسٹم آرکیسٹریشن کی حکمت عملیوں پر محیط ہے۔

## پائتھون بیسڈ لانچ فائلز

ROS 2 XML کی بجائے پائتھون بیسڈ لانچ فائلز استعمال کرتا ہے، جو زیادہ لچک اور پروگرام ایبلٹی فراہم کرتا ہے۔

### بنیادی لانچ فائل ڈھانچہ

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get package share directory
    pkg_share = get_package_share_directory('my_robot_package')

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments to the description
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    ))

    # Add nodes to the launch description
    ld.add_action(Node(
        package='my_robot_package',
        executable='robot_controller',
        name='robot_controller',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'robot_config.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    ))

    return ld
```

### لانچ کنفیگریشن اور آرگیومنٹس

لانچ آرگیومنٹس لانچ فائلز کی رن ٹائم کسٹمائزیشن کی اجازت دیتے ہیں:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get launch configuration values
    robot_name = LaunchConfiguration('robot_name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    nodes = []

    # Create nodes based on launch arguments
    nodes.append(Node(
        package='my_robot_package',
        executable='robot_controller',
        name=f'{robot_name}_controller',
        namespace=namespace,
        parameters=[{'robot_name': robot_name}],
        output='screen'
    ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='my_robot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot nodes'
        ),
        OpaqueFunction(function=launch_setup)
    ])
```

## کارکردگی کے لیے نوڈ کمپوزیشن

نوڈ کمپوزیشن متعدد نوڈز کو ایک ہی پروسیس میں چلانے کی اجازت دیتا ہے، جس سے کمیونیکیشن اوورہیڈ کم ہوتا ہے اور کارکردگی بہتر ہوتی ہے۔

### کمپونینٹس کے ساتھ کمپوزیشن

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Create a container for composed nodes
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_robot_package',
                plugin='my_robot_package::SensorProcessor',
                name='sensor_processor',
                parameters=[{'sensor_rate': 10.0}]
            ),
            ComposableNode(
                package='my_robot_package',
                plugin='my_robot_package::Controller',
                name='controller',
                parameters=[{'control_rate': 50.0}]
            ),
            ComposableNode(
                package='my_robot_package',
                plugin='my_robot_package::DataLogger',
                name='data_logger',
                parameters=[{'log_rate': 1.0}]
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

### نوڈ کمپوزیشن کے فوائد

- **کمیونیکیشن اوورہیڈ میں کمی**: انٹرا پروسیس کمیونیکیشن انٹر پروسیس سے تیز ہوتی ہے
- **کم میموری استعمال**: شیئرڈ لائبریریز اور وسائل
- **بہتر کارکردگی**: کم کنٹیکسٹ سوئچنگ
- **آسان ڈیبگنگ**: تمام نوڈز ایک ہی پروسیس میں

## اعلیٰ پیرامیٹر مینجمنٹ

پیچیدہ روبوٹک سسٹمز کی کنفیگریشن کے لیے موثر پیرامیٹر مینجمنٹ ضروری ہے۔

### YAML پیرامیٹر فائلز

```yaml
# config/robot_config.yaml
/**:  # Applies to all nodes
  ros__parameters:
    robot_name: "my_robot"
    use_sim_time: false
    update_rate: 50.0

    sensors:
      lidar:
        enabled: true
        frame_id: "lidar_link"
        topic_name: "/scan"
      camera:
        enabled: true
        frame_id: "camera_link"
        topic_name: "/image_raw"

    control:
      max_velocity: 1.0
      max_acceleration: 2.0
      position_tolerance: 0.01

robot_controller:  # Applies only to robot_controller node
  ros__parameters:
    control_mode: "velocity"
    feedback_rate: 100.0
    safety_limits:
      max_effort: 100.0
      max_jerk: 5.0

sensor_processor:  # Applies only to sensor_processor node
  ros__parameters:
    processing_rate: 30.0
    filter_params:
      low_pass_gain: 0.1
      high_pass_gain: 0.05
```

### متعدد ذرائع سے پیرامیٹرز لوڈ کرنا

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get package directory
    pkg_share = get_package_share_directory('my_robot_package')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'robot_config.yaml']),
            description='Path to parameter file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            parameters=[
                LaunchConfiguration('config_file'),  # Primary config file
                {'use_sim_time': use_sim_time},     # Runtime override
                {'robot_name': 'configured_robot'}   # Additional parameters
            ],
            output='screen'
        )
    ])
```

## لانچ فائلز میں شرطی منطق

لانچ فائلز لانچ آرگیومنٹس یا دیگر شرائط کی بنیاد پر شرطی ایگزیکیشن سپورٹ کرتی ہیں۔

### شرطی نوڈ لانچنگ

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='false')

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable lidar sensor'
    ))
    ld.add_action(DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable camera sensor'
    ))

    # Conditionally add nodes
    ld.add_action(Node(
        package='lidar_driver',
        executable='lidar_node',
        name='lidar_driver',
        condition=IfCondition(enable_lidar),
        output='screen'
    ))

    ld.add_action(Node(
        package='camera_driver',
        executable='camera_node',
        name='camera_driver',
        condition=IfCondition(enable_camera),
        output='screen'
    ))

    return ld
```

### پیچیدہ شرطی منطق

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def conditional_launch(context: LaunchContext):
    """Function to implement complex conditional logic"""
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)

    nodes = []

    if sim_mode == 'gazebo':
        nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', robot_type, '-file', f'$(find {robot_type}_description)/urdf/{robot_type}.urdf']
        ))

    # Add robot-specific nodes
    if robot_type == 'turtlebot3':
        nodes.append(Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[{'use_sim_time': sim_mode == 'gazebo'}]
        ))
    elif robot_type == 'my_robot':
        nodes.extend([
            Node(
                package='my_robot_package',
                executable='robot_controller',
                parameters=[{'robot_type': robot_type}]
            ),
            Node(
                package='my_robot_package',
                executable='sensor_processor'
            )
        ])

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='real',
            choices=['real', 'gazebo', 'sim'],
            description='Simulation mode'
        ),
        DeclareLaunchArgument(
            'robot_type',
            default_value='my_robot',
            choices=['my_robot', 'turtlebot3'],
            description='Type of robot'
        ),
        OpaqueFunction(function=conditional_launch)
    ])
```

## لانچ فائلز میں ایونٹ ہینڈلنگ

ایونٹ ہینڈلرز آپ کو نوڈ لائف سائیکل ایونٹس اور سسٹم تبدیلیوں کا جواب دینے کی اجازت دیتے ہیں۔

### بنیادی ایونٹ ہینڈلنگ

```python
from launch import LaunchDescription, LaunchService
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # Create nodes
    controller_node = Node(
        package='my_robot_package',
        executable='robot_controller',
        name='robot_controller'
    )

    monitor_node = Node(
        package='my_robot_package',
        executable='system_monitor',
        name='system_monitor'
    )

    # Event handler: When controller starts, start monitor
    controller_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_node,
            on_start=[
                monitor_node
            ]
        )
    )

    # Event handler: When controller exits, shutdown everything
    controller_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Controller exited'))
            ]
        )
    )

    return LaunchDescription([
        controller_node,
        controller_start_handler,
        controller_exit_handler
    ])
```

### شرائط کے ساتھ اعلیٰ ایونٹ ہینڈلنگ

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    safety_node = Node(
        package='my_robot_package',
        executable='safety_monitor',
        name='safety_monitor',
        output='both'  # Both screen and log
    )

    # Event handler: Log when safety node outputs specific messages
    safety_output_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=safety_node,
            on_stdout=lambda event: LogInfo(
                msg=f"[SAFETY] {event.text.decode().strip()}"
            ) if 'EMERGENCY_STOP' in event.text.decode() else None,
            on_stderr=lambda event: LogInfo(
                msg=f"[ERROR] {event.text.decode().strip()}"
            )
        )
    )

    # Event handler: Restart robot controller if it crashes
    controller_node = Node(
        package='my_robot_package',
        executable='robot_controller',
        name='robot_controller'
    )

    controller_restart_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_node,
            on_exit=[
                LogInfo(msg='Controller crashed, restarting...'),
                controller_node  # This will restart the node
            ]
        )
    )

    return LaunchDescription([
        safety_node,
        controller_node,
        safety_output_handler,
        controller_restart_handler
    ])
```

## لانچ فائلز کے بہترین طریقے

### ماڈیولر لانچ فائلز

دوبارہ قابل استعمال لانچ فائل کمپونینٹس بنائیں:

```python
# my_robot_package/launch/robot_core.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Include other launch files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('robot_hardware_interface'),
                    'launch',
                    'hardware_interface.launch.py'
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('robot_control'),
                    'launch',
                    'control.launch.py'
                ])
            ])
        )
    ])
```

### پیرامیٹر ویلڈیشن

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def validate_and_launch(context: LaunchContext):
    """Validate parameters before launching nodes"""
    config_file = LaunchConfiguration('config_file').perform(context)

    # Validate config file exists and is valid YAML
    if not os.path.exists(config_file):
        raise RuntimeError(f"Config file does not exist: {config_file}")

    with open(config_file, 'r') as f:
        try:
            config_data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise RuntimeError(f"Invalid YAML in config file: {e}")

    # Validate required parameters exist
    required_params = ['robot_name', 'update_rate']
    for param in required_params:
        if param not in config_data.get('/**/ros__parameters', {}):
            raise RuntimeError(f"Required parameter '{param}' not found in config")

    # Launch nodes if validation passes
    return [Node(
        package='my_robot_package',
        executable='robot_controller',
        parameters=[config_file]
    )]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=validate_and_launch)
    ])
```

## عملی مشق: پیچیدہ سسٹم لانچ

ایک لانچ فائل بنائیں جو دکھاتی ہو:

1. شرطی منطق کے ساتھ متعدد لانچ آرگیومنٹس
2. کارکردگی کے لیے نوڈ کمپوزیشن
3. YAML فائلز سے پیرامیٹرز لوڈ کرنا
4. نوڈ لائف سائیکل مینجمنٹ کے لیے ایونٹ ہینڈلنگ
5. ماڈیولر لانچ فائل ڈھانچہ

یہ مشق آپ کو لانچ فائلز کے ساتھ پیچیدہ روبوٹک سسٹمز کو آرکیسٹریٹ کرنے کا طریقہ سمجھنے میں مدد دے گی۔

## اگلا باب

اگلے باب میں، [باب 1.6: ROS 2 بہترین طریقے اور ڈیبگنگ](./chapter-1-6.md)، آپ ROS 2 کوڈ لکھنے اور پیچیدہ روبوٹک سسٹمز کو ڈیبگ کرنے کے ضروری طریقے سیکھیں گے۔