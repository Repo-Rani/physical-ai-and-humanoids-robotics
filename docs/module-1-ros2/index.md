---
id: module-1-ros2-index
title: "Module 1: The Robotic Nervous System - ROS 2 (Weeks 3-5)"
sidebar_label: "Module 1: ROS 2 Fundamentals"
sidebar_position: 0
description: "Master ROS 2 fundamentals for building distributed robotic systems: architecture, nodes, topics, services, URDF, and best practices"
keywords: [ros2, robotics, middleware, dds, qos, nodes, topics, services, urdf]
estimated_time: 10
prerequisites:
  - module-0-getting-started-index
  - basic-linux-command-line
learning_outcomes:
  - Implement publisher-subscriber patterns for sensor data streaming
  - Create services for synchronous robot configuration requests
  - Build action servers for preemptible long-running tasks
  - Design robot structure using URDF and Xacro macros
  - Apply ROS 2 best practices for debugging and performance optimization
hardware_tier: proxy
---

# Module 1: The Robotic Nervous System - ROS 2 (Weeks 3-5)

This module introduces you to ROS 2 (Robot Operating System 2), the middleware that serves as the nervous system for distributed robotic systems. You'll learn the core concepts of ROS 2 architecture, communication patterns, and how to build robust robotic applications.

## Module Overview

ROS 2 is the foundation for almost all modern robotics development. This module covers everything from basic architecture concepts to advanced package building and debugging techniques. By the end of this module, you'll be able to create multi-node ROS 2 applications with proper communication patterns and debugging practices.

## Hardware Requirements

This module can be completed with the **Proxy** hardware tier (simulation only):
- 16GB RAM, Quad-core CPU
- Ubuntu 22.04 (or equivalent Linux distribution)
- ROS 2 Humble Hawksbill

Cloud alternative: AWS t3.xlarge instance ($0.17/hr) with Ubuntu 22.04 and ROS 2 Humble pre-installed.

## Prerequisites

Before starting this module, you should have:
- Completed Module 0: Getting Started
- Basic familiarity with Linux command line
- Python programming experience (or willingness to learn)

## Module Structure

This module consists of 6 chapters covering essential ROS 2 concepts:

- **Chapter 1.1**: ROS 2 Architecture and Core Concepts (Week 3)
- **Chapter 1.2**: Nodes, Topics, Services, and Actions (Week 3)
- **Chapter 1.3**: Building ROS 2 Packages with Python (Week 4)
- **Chapter 1.4**: URDF: Describing Robot Structure (Week 4)
- **Chapter 1.5**: Launch Files and Parameter Management (Week 5)
- **Chapter 1.6**: ROS 2 Best Practices and Debugging (Week 5)

## Learning Path

This module establishes the foundation for all subsequent modules. You'll use ROS 2 concepts throughout the curriculum when implementing perception, navigation, and control systems.

## Next Steps

Ready to dive into ROS 2? Start with [Chapter 1.1: ROS 2 Architecture and Core Concepts](./chapter-1-1.md) to understand the DDS middleware and computational graph that underpin all ROS 2 systems.