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

# Chapter 1.1: ROS 2 Architecture and Core Concepts

ROS 2 (Robot Operating System 2) represents a complete redesign of the original ROS framework, addressing critical requirements for safety, security, and scalability in modern robotics applications. This chapter introduces the fundamental architectural concepts that underpin all ROS 2 systems.

## DDS Middleware Foundation

ROS 2's architecture is built on Data Distribution Service (DDS), a middleware standard for real-time, scalable, and reliable communication. Unlike the original ROS which used custom TCP/UDP protocols, DDS provides industry-standard communication patterns with robust Quality of Service (QoS) controls.

### What is DDS?

DDS is an Object Management Group (OMG) standard that provides a publish-subscribe communication model. It enables:

- **Decentralized Communication**: No central master node required
- **Discovery**: Automatic discovery of participants in the network
- **Reliability**: Built-in mechanisms for data delivery assurance
- **Real-time Performance**: Predictable timing characteristics
- **Scalability**: Support for large networks with many participants

### DDS vs. Traditional ROS Communication

| Aspect | Traditional ROS | ROS 2 (DDS-based) |
|--------|----------------|-------------------|
| Architecture | Master-slave | Peer-to-peer |
| Discovery | Master-based | Automatic network discovery |
| Communication | TCP/UDP | DDS middleware |
| QoS Control | Limited | Comprehensive QoS profiles |
| Security | Basic | Built-in security framework |
| Scalability | Moderate | High (industrial scale) |

## The ROS 2 Computational Graph

The computational graph represents all nodes, topics, services, and their connections in a ROS 2 system. Understanding this graph is crucial for designing and debugging distributed robotic systems.

### Graph Components

The computational graph consists of several key elements:

- **Nodes**: Individual processes that perform computation
- **Topics**: Named buses for asynchronous message passing
- **Services**: Synchronous request-response communication
- **Actions**: Goal-oriented communication with feedback
- **Parameters**: Configuration values shared between nodes

### Graph Visualization

ROS 2 provides tools to visualize the computational graph:

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

## Quality of Service (QoS) Profiles

QoS profiles are one of the most important features distinguishing ROS 2 from traditional ROS. They allow fine-tuning of communication behavior to match application requirements.

### QoS Policy Types

ROS 2 defines several QoS policies:

#### Reliability Policy
- **Reliable**: All messages are delivered (with retries if needed)
- **Best Effort**: Messages may be lost; no retries

**Use Cases:**
- Reliable: Critical control commands, configuration data
- Best Effort: Sensor data (LiDAR, camera), status updates where some loss is acceptable

#### Durability Policy
- **Transient Local**: Late-joining subscribers receive last-known values
- **Volatile**: No historical data provided to late joiners

**Use Cases:**
- Transient Local: Parameter servers, maps, static configurations
- Volatile: Streaming sensor data, real-time control commands

#### History Policy
- **Keep Last**: Maintain only the most recent N messages
- **Keep All**: Maintain all messages (limited by resource constraints)

#### Deadline Policy
Specifies the maximum time between consecutive messages. Used for real-time systems where timing is critical.

### Example QoS Configuration

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

## Namespaces and Node Organization

Namespaces provide a way to organize nodes in large-scale robotic systems, preventing naming conflicts and enabling system modularization.

### Namespace Hierarchy

Namespaces follow a hierarchical structure similar to file systems:

```
/robot1/sensors/camera_front
/robot1/sensors/lidar_top
/robot1/control/arm_left
/robot2/sensors/camera_front
/robot2/sensors/lidar_top
```

### Using Namespaces

```bash
# Launch a node with a namespace
ros2 run package_name node_name --ros-args --remap __ns:=/robot1

# Or specify namespace in launch file
<node pkg="package_name" exec="node_name" namespace="robot1" />
```

## Lifecycle Nodes

Lifecycle nodes provide a structured approach to node management, enabling more robust system operation through well-defined state transitions.

### Lifecycle States

- **Unconfigured**: Node loaded but not initialized
- **Inactive**: Node configured but not active
- **Active**: Node running normally
- **Finalized**: Node shut down cleanly

### Benefits of Lifecycle Nodes

- **Coordinated Startup**: Ensures dependencies are ready before activation
- **Graceful Recovery**: Systematic approach to error recovery
- **Resource Management**: Better control over resource allocation
- **System Monitoring**: Clear visibility into node states

### Example Lifecycle Node

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

## Practical Exercise: Computational Graph Analysis

Create a simple ROS 2 system to observe the computational graph:

1. Create a publisher node that publishes sensor data
2. Create a subscriber node that processes the data
3. Use `ros2 run rqt_graph rqt_graph` to visualize the graph
4. Experiment with different QoS profiles and observe the differences

This exercise will help you understand how nodes, topics, and QoS policies interact in the computational graph.

## Next Chapter

In the next chapter, [Chapter 1.2: Nodes, Topics, Services, and Actions](./chapter-1-2.md), you'll implement your first ROS 2 nodes and explore the different communication patterns available in the system.