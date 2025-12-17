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

# Chapter 1.2: Nodes, Topics, Services, and Actions

This chapter explores the four primary communication patterns in ROS 2: topics (publish-subscribe), services (request-reply), actions (goal-based), and parameters. Understanding when and how to use each pattern is crucial for designing effective robotic systems.

## Node Architecture and Implementation

A ROS 2 node is a process that performs computation. All nodes in ROS 2 inherit from the `rclpy.Node` class (Python) or `rclcpp::Node` class (C++).

### Basic Node Structure

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

### Node Lifecycle Management

Nodes should properly manage resources and handle cleanup:

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

## Topic Communication (Publish-Subscribe Pattern)

Topics enable asynchronous, many-to-many communication through a publish-subscribe pattern. This is ideal for streaming data like sensor readings or robot states.

### Publisher Implementation

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

### Subscriber Implementation

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

### Quality of Service Considerations for Topics

Different types of data require different QoS profiles:

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

## Service Communication (Request-Reply Pattern)

Services provide synchronous, request-reply communication. This is ideal for configuration changes, calibration, or any operation that requires a guaranteed response.

### Service Server Implementation

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

### Service Client Implementation

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

## Action Communication (Goal-Based Pattern)

Actions are designed for long-running tasks that may take significant time to complete. They provide feedback during execution and can be preempted.

### Action Server Implementation

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

### Action Client Implementation

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

## Communication Pattern Selection Guide

### When to Use Topics (Publish-Subscribe)

Use topics for:
- **Streaming data**: Sensor readings, robot states, camera images
- **One-to-many communication**: Broadcasting information to multiple subscribers
- **Asynchronous operations**: When immediate response is not required
- **High-frequency updates**: When data rate is too high for services

### When to Use Services (Request-Reply)

Use services for:
- **Configuration changes**: Setting parameters, modes, or states
- **Synchronous operations**: When you need to wait for a response
- **One-to-one communication**: When only one service provider exists
- **Simple operations**: Operations that complete quickly

### When to Use Actions (Goal-Based)

Use actions for:
- **Long-running tasks**: Navigation, manipulation, calibration
- **Progress feedback**: When you need to know how much is completed
- **Preemption capability**: When tasks might need to be canceled
- **Complex goals**: When goals have multiple steps or phases

## Practical Exercise: Multi-Pattern Communication System

Create a simple robot control system that demonstrates all three communication patterns:

1. **Topic**: Publish robot pose at 10Hz
2. **Service**: Set robot mode (idle, active, maintenance)
3. **Action**: Navigate to a goal position with feedback

This exercise will help you understand the appropriate use cases for each communication pattern and how they work together in a robotic system.

## Debugging Communication Issues

ROS 2 provides several tools for debugging communication issues:

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

## Next Chapter

In the next chapter, [Chapter 1.3: Building ROS 2 Packages with Python](./chapter-1-3.md), you'll learn how to create proper ROS 2 packages with proper directory structure, dependencies, and build configurations.