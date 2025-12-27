---
id: module-1-chapter-6
title: "ROS 2 Best Practices and Debugging"
sidebar_label: "1.6 Best Practices & Debugging"
sidebar_position: 6
description: "Master ROS 2 logging with rclpy, debug with ros2 doctor, optimize performance, handle errors gracefully, and implement safety protocols for robust robotic systems"
keywords: [ros2, logging, debugging, performance, best-practices, error-handling, safety]
estimated_time: 75
prerequisites:
  - module-1-chapter-5
  - basic-ros2-programming
learning_outcomes:
  - Implement comprehensive logging with different severity levels
  - Debug ROS 2 systems using ros2 doctor and diagnostic tools
  - Profile and optimize ROS 2 node performance
  - Handle errors and exceptions gracefully in robotic systems
  - Implement safety protocols and emergency procedures
hardware_tier: proxy
---

# باب 1.6: ROS 2 بہترین طریقے اور ڈیبگنگ

یہ باب ROS 2 کوڈ لکھنے کے لیے ضروری بہترین طریقوں اور پیچیدہ روبوٹک سسٹمز کی ڈیبگنگ پر محیط ہے۔ آپ لاگنگ، کارکردگی کی بہتری، غلطیوں کے انتظام، اور حفاظتی پروٹوکولز کے بارے میں سیکھیں گے۔

## rclpy کے ساتھ جامع لاگنگ

روبوٹک سسٹمز کی ڈیبگنگ اور نگرانی کے لیے موثر لاگنگ ضروری ہے۔ ROS 2 مختلف شدت کے لیولز کے ساتھ ایک جامع لاگنگ سسٹم فراہم کرتا ہے۔

### لاگنگ لیولز اور استعمال

```python
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

    def run_control_loop(self):
        # DEBUG: Detailed information for diagnosing problems
        self.get_logger().debug('Starting control loop iteration')

        try:
            # INFO: General information about program execution
            self.get_logger().info('Processing new command')

            # Perform control calculations
            result = self.perform_control_calculation()

            if result is None:
                # WARN: Indication of potential problems
                self.get_logger().warn('Control calculation returned None, using default')
                result = self.get_default_control_output()

            # INFO: Significant events
            self.get_logger().info(f'Control output: {result}')

        except ValueError as e:
            # ERROR: Error events that don't prevent program execution
            self.get_logger().error(f'Invalid control value: {e}')
        except Exception as e:
            # ERROR: General errors
            self.get_logger().error(f'Unexpected error in control loop: {e}')

    def perform_control_calculation(self):
        # Simulate potential error
        import random
        if random.random() < 0.1:  # 10% chance of error
            raise ValueError("Invalid sensor reading")
        return "control_output"

    def get_default_control_output(self):
        return "default_output"
```

### اعلیٰ لاگنگ تشکیل

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

class AdvancedLoggerNode(Node):
    def __init__(self):
        super().__init__('advanced_logger')

        # Configure logging for different components
        self.setup_component_loggers()

    def setup_component_loggers(self):
        # Create loggers for different components
        self.motion_logger = self.get_logger().get_child('motion_control')
        self.sensor_logger = self.get_logger().get_child('sensor_processing')
        self.safety_logger = self.get_logger().get_child('safety_system')

    def process_sensor_data(self, sensor_msg):
        self.sensor_logger.debug(f'Received sensor data: {sensor_msg}')

        if self.is_sensor_data_valid(sensor_msg):
            self.sensor_logger.info('Sensor data validated successfully')
            return self.process_valid_sensor_data(sensor_msg)
        else:
            self.sensor_logger.warn('Invalid sensor data detected')
            return self.get_safe_sensor_data()

    def is_sensor_data_valid(self, sensor_msg):
        # Implement validation logic
        return True  # Simplified for example

    def process_valid_sensor_data(self, sensor_msg):
        # Process valid data
        return "processed_data"

    def get_safe_sensor_data(self):
        # Return safe default values
        return "safe_data"
```

## ROS 2 تشخیصی ٹولز

ROS 2 سسٹم کے مسائل کی شناخت اور حل کے لیے کئی تشخیصی ٹولز فراہم کرتا ہے۔

### سسٹم ہیلتھ کے لیے ros2 ڈاکٹر کا استعمال

```bash
# Check overall system health
ros2 doctor

# Check specific aspects
ros2 doctor --report-timeout 10  # Set timeout for checks
ros2 doctor --report-level 2     # Detailed report level

# Check specific topics
ros2 topic info /topic_name

# Check specific nodes
ros2 node info node_name
```

### تشخیصی پیغامات اور پبلشرز

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')

        # Create diagnostic publisher
        self.diag_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        # Timer for periodic diagnostic updates
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System status
        system_status = DiagnosticStatus()
        system_status.name = 'System Status'
        system_status.level = DiagnosticStatus.OK
        system_status.message = 'All systems nominal'
        system_status.hardware_id = 'robot_system'

        # Add key-value pairs for additional info
        system_status.values.extend([
            KeyValue(key='cpu_usage', value='25%'),
            KeyValue(key='memory_usage', value='45%'),
            KeyValue(key='disk_usage', value='60%'),
            KeyValue(key='temperature', value='42C')
        ])

        diag_array.status.append(system_status)

        # Sensor status
        sensor_status = DiagnosticStatus()
        sensor_status.name = 'Sensor Status'
        sensor_status.level = DiagnosticStatus.OK
        sensor_status.message = 'All sensors operational'
        sensor_status.hardware_id = 'sensor_array'

        diag_array.status.append(sensor_status)

        self.diag_publisher.publish(diag_array)
```

## کارکردگی کی پروفائلنگ اور بہتری

ریل ٹائم روبوٹک سسٹمز کے لیے کارکردگی کو سمجھنا اور بہتر بنانا ضروری ہے۔

### CPU اور میموری پروفائلنگ

```python
import rclpy
from rclpy.node import Node
import psutil
import time
from std_msgs.msg import Float32

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Publishers for performance metrics
        self.cpu_pub = self.create_publisher(Float32, 'cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, 'memory_usage', 10)

        # Timer for performance monitoring
        self.perf_timer = self.create_timer(0.5, self.monitor_performance)

    def monitor_performance(self):
        # Get CPU usage
        cpu_percent = psutil.cpu_percent()
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_pub.publish(cpu_msg)

        # Get memory usage
        memory_percent = psutil.virtual_memory().percent
        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_pub.publish(memory_msg)

        # Log performance warnings
        if cpu_percent > 80:
            self.get_logger().warn(f'High CPU usage: {cpu_percent}%')
        if memory_percent > 85:
            self.get_logger().warn(f'High memory usage: {memory_percent}%')
```

### پیغامات کی پروسیسنگ کی بہتری

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class OptimizedSensorProcessor(Node):
    def __init__(self):
        super().__init__('optimized_sensor_processor')

        # Use efficient subscription with custom QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        qos_profile = QoSProfile(
            depth=1,  # Keep only latest message
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )

        # Pre-allocate arrays to avoid memory allocation in callback
        self.processed_scan_buffer = None

    def scan_callback(self, msg):
        # Convert to numpy array for efficient processing
        ranges = np.array(msg.ranges)

        # Efficient filtering using numpy
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0.1) & (ranges < 10.0)]

        # Perform calculations using vectorized operations
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            avg_distance = np.mean(valid_ranges)

            # Log performance metrics
            self.get_logger().debug(f'Min: {min_distance:.2f}, Avg: {avg_distance:.2f}')
```

## غلطیوں کا انتظام اور ایکسیپشن مینجمنٹ

محفوظ روبوٹک آپریشن کے لیے مضبوط غلطیوں کا انتظام ضروری ہے۔

### جامع غلطیوں کا انتظام

```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
import traceback

class RobustController(Node):
    def __init__(self):
        super().__init__('robust_controller')

        # Emergency stop publisher
        self.emergency_stop_pub = self.create_publisher(
            Bool, 'emergency_stop', 10
        )

        # Error counter for monitoring
        self.error_count = 0
        self.last_error_time = self.get_clock().now()

    def safe_execute(self, func, *args, **kwargs):
        """Wrapper to safely execute functions with error handling"""
        try:
            return func(*args, **kwargs)
        except ControlError as e:
            self.handle_control_error(e)
        except SensorError as e:
            self.handle_sensor_error(e)
        except Exception as e:
            self.handle_general_error(e)

    def handle_control_error(self, error):
        """Handle control system errors"""
        self.get_logger().error(f'Control error: {error}')
        self.trigger_safety_protocol()

    def handle_sensor_error(self, error):
        """Handle sensor system errors"""
        self.get_logger().error(f'Sensor error: {error}')
        self.fallback_to_safe_mode()

    def handle_general_error(self, error):
        """Handle general errors"""
        self.get_logger().error(f'General error: {error}')
        self.get_logger().error(f'Traceback: {traceback.format_exc()}')

        # Increment error counter
        self.error_count += 1
        self.last_error_time = self.get_clock().now()

        # Check if error rate is too high
        if self.error_count > 10:  # More than 10 errors recently
            self.get_logger().fatal('Too many errors, initiating emergency shutdown')
            self.emergency_shutdown()

    def trigger_safety_protocol(self):
        """Execute safety protocol when errors occur"""
        # Publish emergency stop
        from std_msgs.msg import Bool
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Log safety action
        self.get_logger().warn('Safety protocol triggered - robot stopped')

    def emergency_shutdown(self):
        """Complete system shutdown in emergency"""
        self.get_logger().fatal('EMERGENCY SHUTDOWN INITIATED')
        # Perform emergency shutdown procedures
        self.destroy_node()
        rclpy.shutdown()
```

### ٹائم آؤٹ اور ری ٹرائی میکانزم

```python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci
import asyncio

class RobustActionClient(Node):
    def __init__(self):
        super().__init__('robust_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    async def send_goal_with_retry(self, order, max_retries=3, timeout=10.0):
        """Send action goal with retry and timeout"""
        for attempt in range(max_retries):
            try:
                # Wait for action server with timeout
                if not self._action_client.wait_for_server(timeout_sec=timeout):
                    raise TimeoutError(f"Action server not available after {timeout}s")

                # Create and send goal
                goal_msg = Fibonacci.Goal()
                goal_msg.order = order

                future = self._action_client.send_goal_async(goal_msg)

                # Wait for result with timeout
                result = await asyncio.wait_for(
                    future,
                    timeout=timeout
                )

                if result.accepted:
                    return await result.get_result_async()
                else:
                    raise RuntimeError("Goal was rejected by server")

            except (TimeoutError, RuntimeError) as e:
                self.get_logger().warn(f"Attempt {attempt + 1} failed: {e}")
                if attempt == max_retries - 1:  # Last attempt
                    raise
                # Wait before retry
                await asyncio.sleep(1.0)

        return None
```

## حفاظتی پروٹوکولز اور ایمرجنسی طریقہ کار

روبوٹک سسٹمز میں حفاظت سب سے اہم ہے، خاص طور پر وہ جو انسانوں کے قریب کام کرتے ہیں۔

### حفاظتی اسٹیٹ مشین

```python
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class SafetyState(Enum):
    OPERATIONAL = 1
    WARNING = 2
    EMERGENCY_STOP = 3
    MAINTENANCE = 4

class SafetyManager(Node):
    def __init__(self):
        super().__init__('safety_manager')

        self.current_state = SafetyState.OPERATIONAL
        self.velocity_limit = 0.5  # m/s
        self.emergency_stop_active = False

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, 'safety_status', 10)
        self.obstacle_sub = self.create_subscription(
            Float32, 'obstacle_distance', self.obstacle_callback, 10
        )

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.1, self.safety_check)

    def obstacle_callback(self, msg):
        """Handle obstacle detection"""
        if msg.data.data < 0.5:  # Less than 0.5m from obstacle
            self.get_logger().warn(f'Obstacle detected at {msg.data.data}m')
            if self.current_state == SafetyState.OPERATIONAL:
                self.transition_to_state(SafetyState.WARNING)

    def safety_check(self):
        """Periodic safety checks"""
        # Check for emergency conditions
        if self.emergency_stop_active:
            self.enforce_emergency_stop()
        elif self.current_state == SafetyState.WARNING:
            self.enforce_velocity_limit()

        # Publish current safety status
        status_msg = Bool()
        status_msg.data = self.current_state != SafetyState.EMERGENCY_STOP
        self.safety_pub.publish(status_msg)

    def transition_to_state(self, new_state):
        """Safely transition between safety states"""
        old_state = self.current_state
        self.current_state = new_state

        self.get_logger().info(f'Safety state transition: {old_state} -> {new_state}')

        # Take appropriate actions based on state
        if new_state == SafetyState.EMERGENCY_STOP:
            self.execute_emergency_stop()
        elif new_state == SafetyState.MAINTENANCE:
            self.execute_maintenance_mode()

    def enforce_velocity_limit(self):
        """Limit robot velocity in warning state"""
        self.velocity_limit = 0.1  # Reduce to safe speed

    def enforce_emergency_stop(self):
        """Ensure robot stops in emergency state"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    def execute_emergency_stop(self):
        """Full emergency stop procedure"""
        self.enforce_emergency_stop()
        self.get_logger().error('EMERGENCY STOP: Robot stopped immediately')

    def execute_maintenance_mode(self):
        """Maintenance mode procedures"""
        self.get_logger().info('MAINTENANCE MODE: Robot in safe state')
        self.velocity_limit = 0.05  # Very slow for maintenance
```

## ڈیبگنگ تکنیک اور ٹولز

مستحکم روبوٹک سسٹمز کی ترقی کے لیے موثر ڈیبگنگ ضروری ہے۔

### ROS 2 کے ساتھ انٹریکٹیو ڈیبگنگ

```bash
# Echo messages on a topic to see what's being published
ros2 topic echo /topic_name

# Echo with filtering
ros2 topic echo /scan --field ranges[0]

# Call services to test functionality
ros2 service call /set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'param_name', value: {string_value: 'value'}}]}"

# List and inspect nodes
ros2 node info node_name

# Get detailed information about topics
ros2 topic info /topic_name --verbose
```

### کسٹم ڈیبگنگ ٹولز

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterType

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        # Debug publisher
        self.debug_pub = self.create_publisher(String, 'debug_info', 10)

        # Parameter for enabling/disabling debug output
        self.declare_parameter('debug_enabled', False)
        self.debug_enabled = self.get_parameter('debug_enabled').value

        # Timer for periodic debug output
        self.debug_timer = self.create_timer(1.0, self.publish_debug_info)

    def publish_debug_info(self):
        """Publish debugging information"""
        if not self.debug_enabled:
            return

        # Collect debug information
        debug_info = f"Node: {self.get_name()}"
        debug_info += f", Time: {self.get_clock().now().nanoseconds}"
        debug_info += f", State: Operational"

        # Publish debug message
        debug_msg = String()
        debug_msg.data = debug_info
        self.debug_pub.publish(debug_msg)

    def debug_print(self, message):
        """Conditional debug printing"""
        if self.debug_enabled:
            self.get_logger().info(f"DEBUG: {message}")
```

## عملی مشق: مضبوط نوڈ کی نفاذ

ایک مکمل ROS 2 نوڈ بنائیں جو ظاہر کرے:

1. مختلف شدت کے لیولز کے ساتھ جامع لاگنگ
2. مناسب غلطیوں کا انتظام اور ایکسیپشن مینجمنٹ
3. حفاظتی پروٹوکولز اور اسٹیٹ مینجمنٹ
4. کارکردگی کی نگرانی اور بہتری
5. ڈیبگنگ ٹولز اور تکنیک

یہ مشق آپ کو سکھائے گی کہ کیسے مضبوط، پروڈکشن کے لیے تیار ROS 2 نوڈز کو نافذ کیا جائے۔

## اگلا ماڈیول

یہ ماڈیول 1: ROS 2 بنیادی باتوں کا اختتام ہے۔ اب آپ ROS 2 آرکیٹیکچر، کمیونیکیشن پیٹرنز، پیکج مینجمنٹ، روبوٹ کی تفصیل، سسٹم آرکیسٹریشن، اور بہترین طریقوں کی جامع سمجھ رکھتے ہیں۔ [ماڈیول 2: ڈجیٹل ٹوئن](../module-2-digital-twin/index.md) میں، آپ روبوٹکس کی ترقی کے لیے سیمیولیشن ماحول کے بارے میں سیکھیں گے۔