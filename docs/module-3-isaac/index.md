---
id: module-3-isaac-index
title: "Module 3: The AI-Robot Brain - NVIDIA Isaac (Weeks 8-10)"
sidebar_label: "Module 3: NVIDIA Isaac"
sidebar_position: 0
description: "Implement GPU-accelerated perception, navigation, and learning using NVIDIA Isaac platform: Isaac Sim, Isaac ROS, VSLAM, Nav2, and reinforcement learning"
keywords: [nvidia-isaac, isaac-sim, isaac-ros, vslam, nav2, reinforcement-learning, gpu-acceleration]
estimated_time: 10
prerequisites:
  - module-2-digital-twin-index
  - basic-machine-learning
learning_outcomes:
  - Launch and configure Isaac Sim for photorealistic simulation
  - Implement GPU-accelerated VSLAM using NVIDIA NVSLAM pipeline
  - Configure Nav2 for path planning with custom planners for bipedal movement
  - Train robot control policies using Isaac Gym reinforcement learning
  - Apply domain randomization techniques for sim-to-real transfer
hardware_tier: miniature
---

# Module 3: The AI-Robot Brain - NVIDIA Isaac (Weeks 8-10)

This module introduces you to NVIDIA Isaac, the platform for implementing GPU-accelerated perception, navigation, and learning for robotic systems. You'll explore Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception, and techniques for transferring learning from simulation to reality.

## Module Overview

NVIDIA Isaac represents the cutting edge of AI-robotics integration, leveraging GPU acceleration for computationally intensive tasks like visual SLAM, path planning, and reinforcement learning. This module covers the entire Isaac ecosystem and demonstrates how to apply it to complex robotics problems.

## Hardware Requirements

This module requires the **Miniature** hardware tier:
- RTX 3060+ GPU (12GB VRAM minimum)
- 32GB RAM
- Ubuntu 22.04
- CUDA 12.1+
- NVIDIA Isaac Sim 2023.1.1+

Cloud alternative: AWS g5.2xlarge instance ($3.00/hr) with GPU support and Isaac Sim pre-installed.

## Prerequisites

Before starting this module, you should have:
- Completed Module 2: Digital Twin
- Basic understanding of machine learning concepts
- Familiarity with ROS 2 and simulation environments

## Module Structure

This module consists of 6 chapters covering Isaac platform concepts:

- **Chapter 3.1**: NVIDIA Isaac Platform Overview (Week 8)
- **Chapter 3.2**: Isaac Sim: Photorealistic Simulation (Week 8)
- **Chapter 3.3**: Isaac ROS: Hardware-Accelerated VSLAM (Week 9)
- **Chapter 3.4**: Nav2: Path Planning for Bipedal Movement (Week 9)
- **Chapter 3.5**: Reinforcement Learning for Robot Control (Week 10)
- **Chapter 3.6**: Sim-to-Real Transfer Techniques (Week 10)

## Learning Path

The perception and control techniques from this module will be essential for the advanced robotics applications in Modules 4 and 5, where you'll integrate AI capabilities with physical systems.

## Next Steps

Ready to accelerate your robotics with AI? Start with [Chapter 3.1: NVIDIA Isaac Platform Overview](./chapter-3-1.md) to understand the Isaac ecosystem and how it integrates with ROS 2.