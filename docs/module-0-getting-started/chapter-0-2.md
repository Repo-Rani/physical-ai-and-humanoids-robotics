---
id: module-0-chapter-2
title: "Development Environment Setup"
sidebar_label: "0.2 Environment Setup"
sidebar_position: 2
description: "Set up your development environment for Physical AI & Humanoid Robotics: local, cloud, or hybrid approaches with Ubuntu 22.04, ROS 2 Humble, and Docker"
keywords: [development-environment, ubuntu, ros2, docker, setup, configuration]
estimated_time: 60
prerequisites:
  - module-0-chapter-1
  - basic-linux-knowledge
learning_outcomes:
  - Choose appropriate development environment setup for your hardware and budget
  - Install Ubuntu 22.04 (dual boot, VM, or WSL2) with ROS 2 Humble
  - Configure Docker for reproducible development environments
  - Validate environment setup with basic ROS 2 tests
  - Access cloud GPU resources for simulation-intensive tasks
hardware_tier: none
---

# Chapter 0.2: Development Environment Setup

Setting up your development environment is a critical first step in your Physical AI journey. This chapter provides comprehensive guidance for configuring your system with the necessary tools, libraries, and frameworks for robotics development.

## Development Environment Options

The curriculum supports three approaches to environment setup, each suited to different hardware capabilities and budget constraints:

### Option 1: Local Development (Recommended for most users)

Local development provides the best performance for iterative development and testing. This approach requires specific hardware specifications but offers the most responsive development experience.

**Minimum Requirements:**
- CPU: Quad-core processor (Intel i5 or AMD Ryzen 5 equivalent)
- RAM: 16GB (32GB recommended for simulation work)
- Storage: 50GB free space
- OS: Ubuntu 22.04 LTS (recommended) or WSL2 on Windows

**Recommended Specifications:**
- CPU: 8+ cores (Intel i7/i9 or AMD Ryzen 7/9)
- RAM: 32GB+
- GPU: NVIDIA RTX 3060+ (for simulation and Isaac work)
- Storage: 100GB+ SSD

### Option 2: Cloud Development (Recommended for simulation-intensive work)

Cloud development provides access to high-end hardware without requiring expensive local equipment. This approach is ideal for simulation-heavy tasks that require powerful GPUs.

**Recommended Cloud Instances:**
- AWS: g5.xlarge (4 vCPU, 16GB RAM, 1x NVIDIA A10G GPU)
- GCP: A2 series (various GPU options)
- Azure: NCas_T4_v3 series

### Option 3: Hybrid Development (Best of both worlds)

A hybrid approach uses local development for coding and basic testing, with cloud resources for intensive simulation and training tasks.

## Installing Ubuntu 22.04 LTS

Ubuntu 22.04 LTS is the recommended operating system for this curriculum. You have several options for running Ubuntu:

### Option A: Native Installation (Dual Boot)

Dual booting provides the best performance but requires partitioning your hard drive.

1. Download Ubuntu 22.04 LTS from ubuntu.com
2. Create a bootable USB drive using Rufus (Windows) or Etcher (macOS/Linux)
3. Boot from the USB drive and follow installation prompts
4. When partitioning, ensure you have at least 50GB for Ubuntu
5. Complete the installation and configure your user account

### Option B: Virtual Machine

A virtual machine allows Ubuntu to run alongside your existing OS without partitioning.

1. Install VirtualBox or VMware Workstation
2. Create a new VM with Ubuntu 22.04 ISO
3. Allocate resources: minimum 8GB RAM, 4 CPU cores, 50GB storage
4. Install Ubuntu in the VM following standard procedures

### Option C: Windows Subsystem for Linux 2 (WSL2)

WSL2 provides a Linux environment on Windows with good integration.

1. Enable WSL2 in Windows Features
2. Install Ubuntu 22.04 from Microsoft Store
3. Update packages: `sudo apt update && sudo apt upgrade`
4. Configure development tools as needed

## Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the LTS (Long Term Support) version used throughout this curriculum.

### Setting up the ROS 2 Repository

```bash
# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
sudo apt update
```

### Installing ROS 2 Packages

```bash
# Install the ROS 2 desktop package (includes Gazebo)
sudo apt install -y ros-humble-desktop ros-humble-ros-base

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Setting up the ROS 2 Environment

Add the following to your `~/.bashrc` file:

```bash
source /opt/ros/humble/setup.bash
```

Then reload your environment:

```bash
source ~/.bashrc
```

## Installing Docker

Docker enables reproducible development environments and containerized applications.

```bash
# Install Docker from the official repository
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the Docker repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add your user to the docker group (to run Docker without sudo)
sudo usermod -aG docker $USER
```

After adding yourself to the docker group, log out and log back in for the changes to take effect.

## Validation: Testing Your Setup

Verify that your environment is properly configured:

```bash
# Test ROS 2 installation
ros2 --version

# Test that ROS 2 commands are available
ros2 topic list

# Test Docker installation
docker --version
docker run hello-world
```

You should see version information for ROS 2 and Docker, and the hello-world container should run successfully.

## Next Chapter

In the next chapter, [Chapter 0.3: Introduction to Physical AI](./chapter-0-3.md), you'll explore the theoretical foundations of Physical AI, including the embodiment hypothesis and sensorimotor grounding concepts that distinguish it from traditional Digital AI.