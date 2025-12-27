---
id: module-1-chapter-2
title: "Nodes, Topics, Services, and Actions"
sidebar_label: "1.2 Communication Patterns"
sidebar_position: 2
description: "Implement publisher-subscriber patterns for sensor data streaming, request-reply services for robot configuration, and goal-based actions for long-running tasks in distributed robotics systems"
keywords: [ros2, nodes, topics, services, actions, pub-sub, request-reply, robotics]
estimated_time: 75
prerequisites:
  - module-1-chapter-1
  - basic-python-programming
learning_outcomes:
  - Implement publisher-subscriber patterns for sensor data streaming
  - Create services for synchronous robot configuration requests
  - Build action servers for preemptible long-running tasks
  - Choose appropriate communication patterns for different robotics scenarios
  - Debug communication issues using ROS 2 command-line tools
hardware_tier: proxy
---

# باب 1.2: نوڈز، موضوعات، خدمات، اور اقدامات

یہ باب ROS 2 میں چار بنیادی مواصلاتی پیٹرنز کا جائزہ لیتا ہے: موضوعات (پبلش-سبسکرائب)، خدمات (ریکوئسٹ-ریپلائی)، اقدامات (هدف-بنیاد)، اور پیرامیٹرز۔ ہر پیٹرن کا استعمال کب اور کیسے کرنا ہے، اس کی سمجھ موثر روبوٹک سسٹمز ڈیزائن کرنے کے لیے ضروری ہے۔

## نوڈ آرکیٹیکچر اور نفاذ

ROS 2 نوڈ ایک پروسیس ہے جو کمپیوٹیشن انجام دیتی ہے۔ ROS 2 میں تمام نوڈز `rclpy.Node` کلاس (پائیتھون) یا `rclcpp::Node` کلاس (C++) سے وراثت میں آتے ہیں۔

### بنیادی نوڈ ڈھانچہ

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### نوڈ لائف سائیکل مینجمنٹ

نوڈز کو مناسب طریقے سے وسائل کا انتظام کرنا چاہیے اور صفائی کا کام سنبھالنا چاہیے:

```python
def destroy_node(self):
    # Clean up any resources before destroying the node
    if self.timer:
        self.timer.destroy()
    if self.publisher_:
        self.publisher_.destroy()
    if self.subscriber_:
        self.subscriber_.destroy()
    super().destroy_node()
```

## موضوعاتی مواصلات (پبلش-سبسکرائب پیٹرن)

موضوعات غیر متزامن، کثیر سے کثیر مواصلات کو پبلش-سبسکرائب پیٹرن کے ذریعے فعال کرتی ہیں۔ یہ سنسر ریڈنگز یا روبوٹ کی حالتوں جیسے اسٹریمنگ ڈیٹا کے لیے مثالی ہے۔

### پبلشر نفاذ

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)

        # Timer to simulate sensor readings
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        self.sensor_value = 0.0

    def publish_sensor_data(self):
        msg = Float32()
        # Simulate sensor reading (e.g., temperature, distance)
        msg.data = self.sensor_value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing sensor data: {msg.data}')
        self.sensor_value += 0.1  # Simulate changing value
```

### سبسکرائبر نفاذ

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Process the received sensor data
        processed_value = msg.data * 2  # Example processing
        self.get_logger().info(f'Received: {msg.data}, Processed: {processed_value}')
```

### موضوعات کے لیے سروس کی کوالٹی کے اعتبارات

ڈیٹا کی مختلف اقسام کو مختلف QoS پروفائلز کی ضرورت ہوتی ہے:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For critical control data
control_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For sensor data where some loss is acceptable
sensor_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For map data (needs to be available to late joiners)
map_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## سروس مواصلات (ریکوئسٹ-ریپلائی پیٹرن)

سروسز متزامن، ریکوئسٹ-ریپلائی مواصلات فراہم کرتی ہیں۔ یہ کنفیگریشن میں تبدیلیاں، کیلیبریشن، یا کسی بھی آپریشن کے لیے مثالی ہے جس کے لیے گارنٹی شدہ جواب کی ضرورت ہوتی ہے۔

### سروس سرور نفاذ

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ConfigurationServer(Node):
    def __init__(self):
        super().__init__('configuration_server')
        self.srv = self.create_service(
            SetBool,
            'set_robot_mode',
            self.set_robot_mode_callback
        )
        self.robot_mode = 'idle'

    def set_robot_mode_callback(self, request, response):
        if request.data:
            self.robot_mode = 'active'
            response.success = True
            response.message = 'Robot mode set to active'
        else:
            self.robot_mode = 'idle'
            response.success = True
            response.message = 'Robot mode set to idle'

        self.get_logger().info(f'Service called: {response.message}')
        return response
```

### سروس کلائنٹ نفاذ

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(SetBool, 'set_robot_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetBool.Request()

    def send_request(self, mode):
        self.req.data = mode
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## ایکشن مواصلات (هدف-بنیاد پیٹرن)

ایکشنز طویل مدتی کاموں کے لیے ڈیزائن کیے گئے ہیں جو مکمل ہونے میں اہم وقت لے سکتے ہیں۔ وہ نفاذ کے دوران فیڈبیک فراہم کرتے ہیں اور قبل از وقت روکے جا سکتے ہیں۔

### ایکشن سرور نفاذ

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result
```

### ایکشن کلائنٹ نفاذ

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')
```

## مواصلاتی پیٹرن سلیکشن گائیڈ

### کب موضوعات استعمال کریں (پبلش-سبسکرائب)

موضوعات استعمال کریں:
- **اسٹریمنگ ڈیٹا**: سنسر ریڈنگز، روبوٹ کی حالتیں، کیمرا امیجز
- **یک طرفہ سے کثیر مواصلات**: معلومات کو کثیر سبسکرائبرز کو نشر کرنا
- **غیر متزامن آپریشنز**: جب فوری جواب کی ضرورت نہیں ہوتی
- **ہائی فریکوئنسی اپ ڈیٹس**: جب ڈیٹا ریٹ خدمات کے لیے بہت زیادہ ہو

### کب خدمات استعمال کریں (ریکوئسٹ-ریپلائی)

خدمات استعمال کریں:
- **کنفیگریشن میں تبدیلیاں**: پیرامیٹرز، موڈز، یا حالتوں کو سیٹ کرنا
- **متزامن آپریشنز**: جب آپ کو جواب کا انتظار کرنا ہو
- **یک طرفہ مواصلات**: جب صرف ایک سروس فراہم کنندہ موجود ہو
- **سادہ آپریشنز**: آپریشنز جو جلد مکمل ہو جاتے ہیں

### کب اقدامات استعمال کریں (هدف-بنیاد)

ایکشنز استعمال کریں:
- **طویل مدتی کام**: نیویگیشن، مینیپولیشن، کیلیبریشن
- **پروگریس فیڈبیک**: جب آپ کو جاننا ہو کہ کتنا کام مکمل ہوا ہے
- **پریمپشن کی صلاحیت**: جب کاموں کو منسوخ کرنے کی ضرورت ہو
- **مختلف مرحلوں والے مقاصد**: جب مقاصد کے کثیر مراحل یا مرحلے ہوں

## عملی مشق: کثیر پیٹرن مواصلاتی سسٹم

ایک سادہ روبوٹ کنٹرول سسٹم بنائیں جو تینوں مواصلاتی پیٹرنز کا مظاہرہ کرے:

1. **موضوع**: 10Hz پر روبوٹ کی پوزیشن پبلش کریں
2. **سروس**: روبوٹ موڈ سیٹ کریں (آئل، ایکٹیو، میںٹیننس)
3. **ایکشن**: فیڈبیک کے ساتھ ہدف کی پوزیشن پر نیویگیٹ کریں

یہ مشق آپ کو ہر مواصلاتی پیٹرن کے مناسب استعمال کے کیسز اور ان کے ایک روبوٹک سسٹم میں ایک ساتھ کیسے کام کرتے ہیں، سمجھنے میں مدد دے گی۔

## مواصلاتی مسائل کی ڈیبگنگ

ROS 2 مواصلاتی مسائل کی ڈیبگنگ کے لیے کئی ٹولز فراہم کرتا ہے:

```bash
# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic_name std_msgs/msg/String

# List all services
ros2 service list

# Call a service
ros2 service call /service_name example_interfaces/srv/SetBool "{data: true}"

# List all actions
ros2 action list

# Send an action goal
ros2 action send_goal /action_name example_interfaces/action/Fibonacci "{order: 5}"
```

## اگلا باب

اگلے باب میں، [باب 1.3: پائیتھون کے ساتھ ROS 2 پیکیجز بنانا](./chapter-1-3.md)، آپ سیکھیں گے کہ مناسب ڈائریکٹری ڈھانچہ، انحصارات، اور بلڈ کنفیگریشنز کے ساتھ مناسب ROS 2 پیکیجز کیسے بنائے جاتے ہیں۔