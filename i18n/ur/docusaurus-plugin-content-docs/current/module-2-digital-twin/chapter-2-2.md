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

# باب 2.2: گزیبو کی بنیادی باتیں

گزیبو سب سے زیادہ استعمال ہونے والا اوپن سورس روبوٹکس سمولیٹر ہے، جو ROS 2 ایپلی کیشنز کی ٹیسٹنگ کے لیے ایک مضبوط پلیٹ فارم فراہم کرتا ہے۔ یہ باب گزیبو کے ساتھ کام کرنے کی بنیادی باتوں پر محیط ہے، سمولیشن ماحول بنانے سے لے کر ROS 2 کے ساتھ انٹیگریشن تک۔

## گزیبو کی آرکیٹیکچر

گزیبو کئی اہم اجزاء پر مشتمل ہے:

- **گزیبو سرور (gzserver)**: فیزکس سمولیشن اور سینسر جنریشن
- **گزیبو کلائنٹ (gzclient)**: ویژولائزیشن اور GUI
- **SDF ماڈلز**: روبوٹ اور ماحول کی تفصیلات
- **پلگ انز**: فنکشنلٹی کو بڑھانا (سینسرز، ایکچویٹرز، کسٹم بیہیویئرز)

### گزیبو کلاسک بمقابلہ گزیبو (ہارمونک)

| فیچر | گزیبو کلاسک | گزیبو (ہارمونک) |
|---------|----------------|-------------------|
| ریلیز | لیگیسی (ROS 1/2) | جدید (ROS 2) |
| فیزکس | ODE, Bullet, DART | DART (ڈیفالٹ) |
| رینڈرنگ | OGRE 1.x | OGRE 2.x, Optix |
| فارمیٹ | SDF 1.6 | SDF 1.8+ |
| ROS 2 انٹیگریشن | gazebo_ros_pkgs | ros_gz |
| کارکردگی | معتدل | بہتر |

**تجویز**: نئے پراجیکٹس کے لیے گزیبو ہارمونک استعمال کریں۔

## SDF (سمولیشن ڈیسکرپشن فارمیٹ)

SDF سمولیشن ماحول کی وضاحت کے لیے XML پر مبنی فارمیٹ ہے۔

### بنیادی ورلڈ فائل کی ساخت

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

### ماڈل کی تفصیل

ماڈلز سمولیشن میں اشیاء کی نمائندگی کرتے ہیں:

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

## فیزکس کنفیگریشن

فیزکس انجنز یہ طے کرتے ہیں کہ سمولیشن میں اشیاء کیسے تعامل کرتی ہیں۔

### فیزکس پیرامیٹرز

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

### عام فیزکس مسائل

**مسئلہ**: روبوٹ گراؤنڈ پلان سے گرتا ہے
- **وجہ**: ٹکراؤ جیومیٹری غائب یا غلط ہے
- **حل**: تصدیق کریں کہ ٹکراؤ عناصر بصری جیومیٹری سے میل کھاتے ہیں

**مسئلہ**: سمولیشن ریئل ٹائم سے سست چلتی ہے
- **وجہ**: فیزکس بہت پیچیدہ ہے یا ٹائم اسٹیپ بہت چھوٹا ہے
- **حل**: ٹکراؤ مشز کو آسان بنائیں، ٹائم اسٹیپ بڑھائیں

**مسئلہ**: غیر مستحکم بیہیویئر (جھٹکے، اچھلنا)
- **وجہ**: سخت پابندیاں، غیر مناسب رابطہ پیرامیٹرز
- **حل**: ڈیمپنگ، فرکشن، رابطہ سختی کو ایڈجسٹ کریں

## گزیبو میں روبوٹز کو سپاون کرنا

### طریقہ 1: ورلڈ فائل میں شامل کریں

```xml
<world name="robot_world">
  <include>
    <uri>model://my_robot</uri>
    <pose>0 0 0.5 0 0 0</pose>
  </include>
</world>
```

### طریقہ 2: ROS 2 سے سپاون کریں

```bash
# Using spawn_entity.py from ros_gz_sim
ros2 run ros_gz_sim spawn_entity.py \
    -name my_robot \
    -file /path/to/model.sdf \
    -x 0 -y 0 -z 0.5
```

### طریقہ 3: لانچ فائل انٹیگریشن

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

## ROS 2 انٹیگریشن کے ساتھ ros_gz_bridge

برج گزیبو اور ROS 2 کے درمیان کمیونیکیشن کو فعال کرتا ہے۔

### ros_gz انسٹال کرنا

```bash
sudo apt update
sudo apt install ros-humble-ros-gz
```

### ٹاپکس کو برج کرنا

```bash
# Bridge a single topic
ros2 run ros_gz_bridge parameter_bridge \
    /model/my_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# Bridge multiple topics (create a YAML config)
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:=bridge_config.yaml
```

### برج کنفیگریشن فائل

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

## گزیبو پلگ انز

پلگ انز گزیبو کی فنکشنلٹی کو سینسرز اور ایکچویٹرز کے لیے بڑھاتے ہیں۔

### عام پلگ ان ٹائپس

- **سینسر پلگ انز**: کیمرا، LiDAR، IMU، GPS
- **ایکچویٹر پلگ انز**: ڈفرینشل ڈرائیو، جوائنٹ کنٹرولرز
- **ورلڈ پلگ انز**: کسٹم فیزکس، ماحولیاتی اثرات
- **ماڈل پلگ انز**: روبوٹ مخصوص بیہیویئرز

### مثال: ڈفرینشل ڈرائیو پلگ ان

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

## عملی مشق: ایک گزیبو ورلڈ بنائیں

ایک کسٹم گزیبو ماحول بنائیں:

1. **ورلڈ فائل بنائیں**: گراؤنڈ، لائٹنگ، اور رکاوٹوں کی وضاحت کریں
2. **فیزکس شامل کریں**: DART فیزکس انجن کو 1ms ٹائم اسٹیپ کے ساتھ کنفیگر کریں
3. **روبوٹ سپاون کریں**: ماڈیول 1 سے اپنا URDF استعمال کریں
4. **ٹاپکس برج کریں**: cmd_vel اور سینسر ڈیٹا کو ROS 2 سے جوڑیں
5. **کنٹرول ٹیسٹ کریں**: ٹیلی آپ کا استعمال کرتے ہوئے روبوٹ کو ڈرائیو کریں

**تصدیق**:
- [ ] ورلڈ بغیر کسی غلطی کے لوڈ ہوتا ہے
- [ ] روبوٹ صحیح پوزیشن پر سپاون ہوتا ہے
- [ ] ٹاپکس کامیابی سے برج ہوتے ہیں (چیک کریں `ros2 topic list`)
- [ ] روبوٹ ویلاسٹی کمانڈز کا جواب دیتا ہے

## ٹربل شوٹنگ گائیڈ

| علامت | ممکنہ وجہ | حل |
|---------|----------------|----------|
| گزیبو لانچ پر کریش ہوتا ہے | ماڈل ڈپینڈنسیز غائب ہیں | `GZ_SIM_RESOURCE_PATH` میں ماڈل پاتھز چیک کریں |
| روبوٹ نظر نہیں آتا | غلط ماڈل URI | ورلڈ فائل میں `<uri>` کی تصدیق کریں |
| کوئی ROS 2 ٹاپکس نہیں | برج نہیں چل رہا | صحیح کنفیگ کے ساتھ ros_gz_bridge لانچ کریں |
| فیزکس غیر مستحکم ہے | ٹائم اسٹیپ بہت بڑا ہے | `<max_step_size>` کو 0.001 تک کم کریں |
| سست سمولیشن | پیچیدہ ٹکراؤ مشز | جیومیٹری کو آسان بنائیں یا پرائمیٹیو شاپس استعمال کریں |

## اگلا باب

[باب 2.3: سینسر سمولیشن](./chapter-2-3.md) میں، آپ کیمرے، LiDAR، اور IMUs کو ریئلسٹک نوائز ماڈلز کے ساتھ سمولیٹ کرنا سیکھیں گے تاکہ پرسیپشن الگورتھم ڈویلپمنٹ کے لیے۔