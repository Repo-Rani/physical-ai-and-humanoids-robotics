---
id: module-1-chapter-1
title: "ROS 2 Architecture and Core Concepts"
sidebar_label: "1.1 Architecture & Core Concepts"
sidebar_position: 1
description: "Learn ROS 2 architecture: DDS middleware, computational graph, Quality of Service (QoS) profiles, namespaces, and lifecycle nodes for distributed robotic systems"
keywords: [ros2, dds, middleware, computational-graph, qos, nodes, topics, services]
estimated_time: 75
prerequisites:
  - module-0-chapter-3
  - basic-programming-concepts
learning_outcomes:
  - Understand DDS middleware and its role in ROS 2 communication
  - Identify components of the ROS 2 computational graph
  - Configure Quality of Service (QoS) profiles for different communication needs
  - Implement namespace management for large-scale robotic systems
  - Create and manage lifecycle nodes for robust system operation
hardware_tier: proxy
---

# باب 1.1: ROS 2 کی ساخت اور بنیادی تصورات

ROS 2 (روبوٹ آپریٹنگ سسٹم 2) اصل ROS فریم ورک کا ایک مکمل دوبارہ ڈیزائن ہے، جو جدید روبوٹکس ایپلی کیشنز میں حفاظت، سیکیورٹی، اور اسکیل ایبلٹی کی اہم ضروریات کو پورا کرتا ہے۔ یہ باب ROS 2 سسٹمز کی بنیاد بننے والے بنیادی آرکیٹیکچرل تصورات کا تعارف پیش کرتا ہے۔

## DDS مڈل ویئر فاؤنڈیشن

ROS 2 کی ساخت ڈیٹا ڈسٹری بیوشن سروس (DDS) پر مبنی ہے، جو ریئل ٹائم، اسکیل ایبل، اور قابل اعتماد کمیونیکیشن کے لیے ایک مڈل ویئر معیار ہے۔ اصل ROS جو کسٹم TCP/UDP پروٹوکولز استعمال کرتا تھا، کے برعکس، DDS انڈسٹری معیار کی کمیونیکیشن پیٹرنز فراہم کرتا ہے جس میں مضبوط کوالٹی آف سروس (QoS) کنٹرولز شامل ہیں۔

### DDS کیا ہے؟

DDS ایک آبجیکٹ مینجمنٹ گروپ (OMG) معیار ہے جو ایک پبلش-سبسکرائب کمیونیکیشن ماڈل فراہم کرتا ہے۔ یہ درج ذیل کو ممکن بناتا ہے:

- **غیر مرکزی کمیونیکیشن**: کوئی مرکزی ماسٹر نوڈ درکار نہیں
- **ڈسکووری**: نیٹ ورک میں شرکاء کی خودکار دریافت
- **اعتمادیت**: ڈیٹا ڈیلیوری کی یقین دہانی کے لیے بلٹ ان میکانزم
- **ریئل ٹائم کارکردگی**: قابل پیشین گوئی ٹائمنگ خصوصیات
- **اسکیل ایبلٹی**: بڑے نیٹ ورکس اور بہت سے شرکاء کی حمایت

### DDS بمقابلہ روایتی ROS کمیونیکیشن

| پہلو | روایتی ROS | ROS 2 (DDS پر مبنی) |
|--------|----------------|-------------------|
| آرکیٹیکچر | ماسٹر-سلیو | پیئر ٹو پیئر |
| ڈسکووری | ماسٹر پر مبنی | خودکار نیٹ ورک ڈسکووری |
| کمیونیکیشن | TCP/UDP | DDS مڈل ویئر |
| QoS کنٹرول | محدود | جامع QoS پروفائلز |
| سیکیورٹی | بنیادی | بلٹ ان سیکیورٹی فریم ورک |
| اسکیل ایبلٹی | معتدل | زیادہ (انڈسٹریل اسکیل) |

## ROS 2 کمپیوٹیشنل گراف

کمپیوٹیشنل گراف ROS 2 سسٹم میں تمام نوڈز، ٹاپکس، سروسز، اور ان کے کنکشنز کی نمائندگی کرتا ہے۔ اس گراف کو سمجھنا تقسیم شدہ روبوٹک سسٹمز کے ڈیزائن اور ڈیبگنگ کے لیے اہم ہے۔

### گراف اجزاء

کمپیوٹیشنل گراف کئی اہم عناصر پر مشتمل ہوتا ہے:

- **نوڈز**: انفرادی پروسیز جو کمپیوٹیشن انجام دیتی ہیں
- **ٹاپکس**: غیر متزامن میسج پاسنگ کے لیے نامزد بسز
- **سروسز**: متزامن درخواست-جواب کمیونیکیشن
- **ایکشنز**: فیڈبیک کے ساتھ مقصد پر مبنی کمیونیکیشن
- **پیرامیٹرز**: نوڈز کے درمیان شیئر کی گئی کنفیگریشن ویلیوز

### گراف ویژولائزیشن

ROS 2 کمپیوٹیشنل گراف کو ویژولائز کرنے کے لیے ٹولز فراہم کرتا ہے:

```bash
# Visualize the current graph
ros2 run rqt_graph rqt_graph

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Get information about a specific node
ros2 node info <node_name>
```

## کوالٹی آف سروس (QoS) پروفائلز

QoS پروفائلز ROS 2 کو روایتی ROS سے ممتاز کرنے والی اہم خصوصیات میں سے ایک ہیں۔ یہ ایپلی کیشن کی ضروریات کے مطابق کمیونیکیشن کے رویے کو باریک بین سے ٹیون کرنے کی اجازت دیتے ہیں۔

### QoS پالیسی کی اقسام

ROS 2 کئی QoS پالیسیز کو تعریف کرتا ہے:

#### اعتمادیت پالیسی
- **مستحکم**: تمام میسجز ڈیلیور کیے جاتے ہیں (ضرورت پڑنے پر ری ٹرائز کے ساتھ)
- **بہترین کوشش**: میسجز کھو سکتے ہیں؛ کوئی ری ٹرائی نہیں

**استعمال کے کیسز:**
- مستحکم: اہم کنٹرول کمانڈز، کنفیگریشن ڈیٹا
- بہترین کوشش: سینسر ڈیٹا (LiDAR، کیمرا)، اسٹیٹس اپ ڈیٹس جہاں کچھ نقصان قابل قبول ہے

#### دوام پالیسی
- **عارضی مقامی**: دیر سے شامل ہونے والے سبسکرائبرز کو آخری جانے والے ویلیوز ملتے ہیں
- **متغیر**: دیر سے شامل ہونے والوں کو کوئی تاریخی ڈیٹا فراہم نہیں کیا جاتا

**استعمال کے کیسز:**
- عارضی مقامی: پیرامیٹر سرورز، میپس، سٹٹک کنفیگریشنز
- متغیر: اسٹریمنگ سینسر ڈیٹا، ریئل ٹائم کنٹرول کمانڈز

#### تاریخ پالیسی
- **آخری کو برقرار رکھیں**: صرف حال ہی میں N میسجز برقرار رکھیں
- **سب کو برقرار رکھیں**: تمام میسجز برقرار رکھیں (وسائل کی پابندیوں کے تابع)

#### ڈیڈلائن پالیسی
متواتر میسجز کے درمیان زیادہ سے زیادہ وقت کی وضاحت کرتی ہے۔ ریئل ٹائم سسٹمز کے لیے استعمال ہوتی ہے جہاں ٹائمنگ اہم ہوتی ہے۔

### QoS کنفیگریشن کا مثال

```python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# QoS for critical control commands
critical_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# QoS for sensor data (acceptable to lose some messages)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)
```

## نام سپیسز اور نوڈ آرگنائزیشن

نام سپیسز بڑے پیمانے پر روبوٹک سسٹمز میں نوڈز کو منظم کرنے کا ایک طریقہ فراہم کرتے ہیں، ناموں کے ٹکراؤ کو روکتے ہیں اور سسٹم کی ماڈیولرائزیشن کو ممکن بناتے ہیں۔

### نام سپیس ہائرارکی

نام سپیسز فائل سسٹمز کی طرح ایک ہائرارکیکل ساخت پر عمل کرتے ہیں:

```
/robot1/sensors/camera_front
/robot1/sensors/lidar_top
/robot1/control/arm_left
/robot2/sensors/camera_front
/robot2/sensors/lidar_top
```

### نام سپیسز کا استعمال

```bash
# Launch a node with a namespace
ros2 run package_name node_name --ros-args --remap __ns:=/robot1

# Or specify namespace in launch file
<node pkg="package_name" exec="node_name" namespace="robot1" />
```

## لائف سائیکل نوڈز

لائف سائیکل نوڈز نوڈ مینجمنٹ کے لیے ایک ساختہ بند طریقہ فراہم کرتے ہیں، جو اچھی طرح سے تعریف شدہ اسٹیٹ ٹرانزیشنز کے ذریعے زیادہ مضبوط سسٹم آپریشن کو ممکن بناتے ہیں۔

### لائف سائیکل اسٹیٹس

- **غیر کنفیگرڈ**: نوڈ لوڈ ہوا لیکن آغاز نہیں ہوا
- **غیر فعال**: نوڈ کنفیگر ہوا لیکن فعال نہیں
- **فعال**: نوڈ عام طور پر چل رہا ہے
- **فائنلائزڈ**: نوڈ صاف ستھرا بند ہو گیا

### لائف سائیکل نوڈز کے فوائد

- **مترتب آغاز**: فعال ہونے سے پہلے ڈیپینڈنسیز کی تیاری کی یقین دہانی
- **خوش اسلوبی سے بحالی**: غلطی کی بحالی کے لیے نظامتی طریقہ
- **ریسورس مینجمنٹ**: وسائل کی تقسیم پر بہتر کنٹرول
- **سسٹم مانیٹرنگ**: نوڈ اسٹیٹس کی واضح نظر

### لائف سائیکل نوڈ کا مثال

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring')
        # Initialize resources
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating')
        # Start active operations
        return TransitionCallbackReturn.SUCCESS
```

## عملی مشق: کمپیوٹیشنل گراف تجزیہ

کمپیوٹیشنل گراف کو مشاہدہ کرنے کے لیے ایک سادہ ROS 2 سسٹم بنائیں:

1. ایک پبلشر نوڈ بنائیں جو سینسر ڈیٹا پبلش کرتا ہو
2. ایک سبسکرائبر نوڈ بنائیں جو ڈیٹا کو پروسیس کرتا ہو
3. گراف کو ویژولائز کرنے کے لیے `ros2 run rqt_graph rqt_graph` کا استعمال کریں
4. مختلف QoS پروفائلز کے ساتھ تجربہ کریں اور فرق کو مشاہدہ کریں

یہ مشق آپ کو سمجھنے میں مدد دے گی کہ نوڈز، ٹاپکس، اور QoS پالیسیز کمپیوٹیشنل گراف میں کیسے تعامل کرتے ہیں۔

## اگلا باب

اگلے باب میں، [باب 1.2: نوڈز، ٹاپکس، سروسز، اور ایکشنز](./chapter-1-2.md)، آپ اپنا پہلا ROS 2 نوڈز نافذ کریں گے اور سسٹم میں دستیاب مختلف