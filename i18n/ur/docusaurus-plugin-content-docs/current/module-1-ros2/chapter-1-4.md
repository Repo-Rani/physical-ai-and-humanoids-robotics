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

# باب 1.4: URDF: روبوٹ کی ساخت کا بیان

URDF (Unified Robot Description Format) ROS میں روبوٹ ماڈلز کا بیان کرنے کا معیار ہے۔ یہ باب روبوٹ کے بیان بنانے کی بنیادی باتوں پر محیط ہے، بنیادی لنک اور جوائنٹ کی تعریفوں سے لے کر پیچیدہ کینیمیٹک چینز اور ویژولائزیشن تک۔

## URDF ساخت کی سمجھ

URDF ایک XML پر مبنی فارمیٹ ہے جو روبوٹ کے جسمانی اور بصری خصوصیات کا بیان کرتا ہے۔ ایک URDF فائل میں شامل ہوتا ہے:

- **لنکس**: روبوٹ کو بنانے والے سخت اجسام
- **جوائنٹس**: لنکس کے درمیان رابطے جو نسبتاً حرکت کی اجازت دیتے ہیں
- **ویژول**: روبوٹ کی شکل سیمیولیشن اور ویژولائزیشن میں
- **ٹکراؤ**: روبوٹ کی جسمانی دنیا کے ساتھ تعامل
- **انرشل**: فیزکس سیمیولیشن کے لیے ماس کی خصوصیات

### بنیادی URDF ساخت

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

## لنک عناصر

لنکس روبوٹ میں سخت اجسام کی نمائندگی کرتے ہیں۔ ہر لنک کے متعدد چائلڈ عناصر ہو سکتے ہیں جو اس کی خصوصیات کی تعریف کرتے ہیں۔

### بصری خصوصیات

`<visual>` عنصر کی تعریف کرتا ہے کہ لنک ویژولائزیشن اور سیمیولیشن میں کیسے نظر آتا ہے:

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

### ٹکراؤ کی خصوصیات

`<collision>` عنصر کی تعریف کرتا ہے کہ لنک جسمانی دنیا کے ساتھ کیسے تعامل کرتا ہے:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Often simplified compared to visual geometry for performance -->
    <box size="1 2 3"/>
  </geometry>
</collision>
```

### انرشل خصوصیات

`<inertial>` عنصر کی تعریف کرتا ہے کہ ڈائنامکس سیمیولیشن کے لیے جسمانی خصوصیات:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.4"/>
</inertial>
```

ایک ٹھوس کرہ کے لیے: `ixx = iyy = izz = 0.4 * mass * radius²`
ایک ٹھوس سلنڈر (z-axis کے ساتھ): `ixx = iyy = mass * (3*radius² + length²) / 12`, `izz = 0.5 * mass * radius²`

## جوائنٹ عناصر

جوائنٹس لنکس کے درمیان رابطے کی تعریف کرتے ہیں اور اجازت یافتہ حرکت کی وضاحت کرتے ہیں۔

### جوائنٹ کی اقسام

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

## Xacro قابل استعمال روبوٹ ماڈلز کے لیے

Xacro (XML Macros) آپ کو میکروز اور خصوصیات کا استعمال کرتے ہوئے پیرامیٹرائزڈ، قابل استعمال روبوٹ کے بیان بنانے کی اجازت دیتا ہے۔

### بنیادی Xacro ساخت

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

### اعلیٰ Xacro فیچرز

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

## ٹکراؤ بمقابلہ بصری جیومیٹری

کارکردگی اور درستی کے لیے ٹکراؤ اور بصری جیومیٹری کے درمیان فرق کو سمجھنا ضروری ہے:

### بصری جیومیٹری
- کی تعریف کرتا ہے کہ روبوٹ RViz2 اور سیمیولیشن میں کیسے نظر آتا ہے
- حقیقی شکل کے لیے تفصیلی میشز شامل کر سکتا ہے
- رینڈرنگ اور ویژولائزیشن کے لیے استعمال ہوتا ہے
- ٹکراؤ جیومیٹری سے زیادہ پیچیدہ ہو سکتا ہے

### ٹکراؤ جیومیٹری
- کی تعریف کرتا ہے کہ روبوٹ جسمانی دنیا کے ساتھ کیسے تعامل کرتا ہے
- کارکردگی کے لیے سادہ ہونا چاہیے
- ٹکراؤ کی تشخیص اور فیزکس سیمیولیشن کے لیے استعمال ہوتا ہے
- اکثر بنیادی شکلیں (بکسز، سلنڈرز، کرے) استعمال کرتا ہے

## RViz2 میں روبوٹ کی ویژولائزیشن

RViz2 ROS 2 کا 3D ویژولائزیشن ٹول ہے جو روبوٹ ماڈلز دکھا سکتا ہے۔

### روبوٹ ویژولائزیشن کا آغاز

```bash
# Launch robot state publisher to broadcast transforms
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat my_robot.urdf)

# Launch RViz2
ros2 run rviz2 rviz2
```

### روبوٹ ویژولائزیشن کے لیے RViz2 کے ضروری پلگ انز

1. **RobotModel**: URDF کا استعمال کرتے ہوئے روبوٹ ماڈل دکھاتا ہے
2. **TF**: کوآرڈینیٹ فریمز اور ان کے تعلقات دکھاتا ہے
3. **JointState**: جوائنٹ پوزیشنز دکھاتا ہے (joint_state_publisher کی ضرورت ہوتی ہے)

### حرکت پذیری کے لیے جوائنٹ اسٹیٹ پبلشر

روبوٹ کو RViz2 میں حرکت کرتے ہوئے دیکھنے کے لیے، آپ کو جوائنٹ اسٹیٹس پبلش کرنے کی ضرورت ہے:

```bash
# Interactive joint state publisher
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Or non-interactive
ros2 run joint_state_publisher joint_state_publisher
```

## URDF ماڈلز کی توثیق

کئی ٹولز URDF ماڈلز کی توثیق اور ڈیبگنگ میں مدد کرتے ہیں:

### کمانڈ لائن توثیق

```bash
# Check URDF syntax
check_urdf my_robot.urdf

# View robot tree structure
urdf_to_graphiz my_robot.urdf

# Parse and display robot information
ros2 run xacro xacro my_robot.urdf.xacro
```

### URDF کی عام غلطیاں اور حل

1. **غائب پیرنٹ/چائلڈ لنکس**: یقینی بنائیں کہ تمام جوائنٹس موجودہ لنکس کی طرف اشارہ کرتے ہیں
2. **غیر معیاری جیومیٹری**: جیومیٹری پیرامیٹرز چیک کریں (مثال کے طور پر، رداس > 0)
3. **دہرائے گئے نام**: تمام لنکس اور جوائنٹس کے منفرد نام ہونے چاہئیں
4. **منقطع اجزاء**: تمام لنکس کو جوائنٹس کے ذریعے جوڑا جانا چاہیے
5. **غیر معیاری انرشل ویلیوز**: یقینی بنائیں کہ انرشل میٹرکس مثبت معین ہے

## عملی مشق: ایک سادہ روبوٹ ماڈل بنائیں

ایک مکمل روبوٹ ماڈل بنائیں جس میں شامل ہو:

1. بصری اور ٹکراؤ جیومیٹری کے ساتھ ایک بیس لنک
2. مختلف اجزاء کو جوڑنے والے متعدد جوائنٹس
3. دوہرانے سے بچنے کے لیے Xacro میکروز کا استعمال
4. check_urdf کا استعمال کرتے ہوئے ماڈل کی توثیق
5. RViz2 میں روبوٹ کی ویژولائزیشن

یہ مشق آپ کو URDF میں روبوٹ ماڈلز بنانے اور ان کی توثیق کے لیے مکمل ورک فلو کو سمجھنے میں مدد دے گی۔

## اگلا باب

اگلے باب میں، [باب 1.5: لانچ فائلز اور پیرامیٹر مینجمنٹ](./chapter-1-5.md)، آپ لانچ فائلز اور پیرامیٹرز کے ساتھ پیچیدہ روبوٹک سسٹمز کو منیج کرنے کے اعلیٰ تکنیک سیکھیں گے۔