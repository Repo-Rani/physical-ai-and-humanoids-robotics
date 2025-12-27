---
id: hardware-guide
title: "Hardware Setup Guide"
sidebar_label: "Hardware Setup Guide"
sidebar_position: 10
description: "Complete hardware setup guide with tabs for Local Workstation, Cloud GPU, and Budget Alternative configurations for Physical AI & Humanoid Robotics curriculum"
keywords: [hardware-setup, local-workstation, cloud-gpu, budget-alternative, development-environment]
estimated_time: 120
prerequisites: []
learning_outcomes:
  - Configure Digital Twin Workstation with RTX 3060+ GPU and Ubuntu 22.04
  - Set up Cloud GPU environment on AWS g5.xlarge instance for Isaac Sim
  - Implement Budget Alternative with simulation-only approach and software rendering
  - Validate hardware setup with CUDA, ROS 2, Gazebo, and Docker installations
  - Troubleshoot common hardware and software configuration issues
hardware_tier: none
---

# ہارڈویئر سیٹ اپ گائیڈ

یہ گائیڈ آپ کے ترقیاتی ماحول کو Physical AI & Humanoid Robotics نصاب کے لیے سیٹ اپ کرنے کے لیے تفصیلی ہدایات فراہم کرتا ہے۔ ہم مختلف بجٹ اور سیکھنے کے مقاصد کو مدنظر رکھتے ہوئے تین ہارڈویئر ٹائرز پیش کرتے ہیں۔

## ہارڈویئر ٹائرز کا جائزہ

### پراکسی (صرف سیمیولیشن) - $0
- **ہدف**: ماڈیولز 0-2
- **ضروریات**: 16GB RAM، Quad-core CPU
- **استعمال کا کیس**: ROS 2 اور بنیادی سیمیولیشن کا تعارف

### مینی ایچر ($1.5k-2k) - ڈجیٹل ٹوئن ورک اسٹیشن
- **ہدف**: ماڈیولز 2-3 (Gazebo & Isaac)
- **ضروریات**: RTX 3060+ GPU (12GB VRAM)، 32GB RAM، Ubuntu 22.04
- **استعمال کا کیس**: فیزکس-درست سیمیولیشن اور GPU-تیز AI

### پریمیئم ($90k+) - Physical AI ایج کٹ
- **ہدف**: ماڈیول 4-5 (VLA ماڈلز & Capstone)
- **ضروریات**: Jetson Orin Nano + RealSense D435i + USB مائیکروفون
- **استعمال کا کیس**: حقیقی دنیا میں تعیناتی اور انسان-روبوٹ تعامل

## مقامی ورک اسٹیشن سیٹ اپ

### ڈجیٹل ٹوئن ورک اسٹیشن (ماڈیولز 2-3 کے لیے تجویز کردہ)

#### کم از کم خصوصیات
- **CPU**: Intel i7-10700K / AMD Ryzen 7 3700X یا بہتر
- **GPU**: NVIDIA RTX 3060 (12GB) / RTX 3070 یا بہتر
- **RAM**: 32GB DDR4-3200MHz یا بہتر
- **سٹوریج**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS

#### تجویز کردہ خصوصیات
- **CPU**: Intel i9-12900K / AMD Ryzen 9 5900X
- **GPU**: NVIDIA RTX 4070 Ti / RTX 4080 یا RTX A4000/A5000 پیشہ ورانہ استعمال کے لیے
- **RAM**: 64GB DDR4-3600MHz
- **سٹوریج**: 2TB+ NVMe SSD
- **OS**: Ubuntu 22.04 LTS

#### تنصیب کے اقدامات

1. **Ubuntu 22.04 LTS انسٹال کریں**
   - [ubuntu.com](https://ubuntu.com/download/desktop) سے ڈاؤن لوڈ کریں
   - Rufus (Windows) یا Etcher (macOS/Linux) کا استعمال کرتے ہوئے بوٹ ایبل USB بنائیں
   - USB سے بوٹ کریں اور تنصیب کے احکامات پر عمل کریں
   - Ubuntu پارٹیشن کے لیے کم از کم 100GB مختص کریں

2. **NVIDIA ڈرائیورز انسٹال کریں**
   ```bash
   # Add graphics drivers PPA
   sudo add-apt-repository ppa:graphics-drivers/ppa
   sudo apt update

   # Install recommended driver
   sudo ubuntu-drivers autoinstall

   # Or install specific driver
   sudo apt install nvidia-driver-535
   ```

3. **Install CUDA 12.1**
   ```bash
   # Download CUDA 12.1 from NVIDIA
   wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda_12.1.0_530.30.02_linux.run

   # Make executable and run
   chmod +x cuda_12.1.0_530.30.02_linux.run
   sudo sh cuda_12.1.0_530.30.02_linux.run
   ```

4. **Install ROS 2 Humble Hawksbill**
   ```bash
   # Set up ROS 2 apt repository
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS 2 packages
   sudo apt update
   sudo apt install -y ros-humble-desktop ros-humble-ros-base
   sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
   ```

5. **Install Gazebo Garden/Harmonic**
   ```bash
   # Add Gazebo repository
   sudo curl -sSL http://get.gazebosim.org | sh

   # Install Gazebo Garden
   sudo apt install gz-garden
   ```

6. **Install Docker**
   ```bash
   # Install Docker
   sudo apt update
   sudo apt install ca-certificates curl gnupg lsb-release

   sudo mkdir -p /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

   sudo apt update
   sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

   # Add user to docker group
   sudo usermod -aG docker $USER
   ```

7. **Install NVIDIA Isaac Sim**
   - Download Isaac Sim from [NVIDIA Developer Portal](https://developer.nvidia.com/isaac-sim)
   - Follow installation instructions for Linux
   - Ensure your GPU supports Isaac Sim requirements

## Cloud GPU Setup

### AWS g5.xlarge Instance Setup

#### Instance Configuration
- **Instance Type**: g5.xlarge (4 vCPU, 16GB RAM, 1x NVIDIA A10G GPU)
- **AMI**: Ubuntu 22.04 LTS
- **Storage**: 100GB+ GP3 SSD
- **Security Group**: Allow SSH (port 22) and VNC (port 5900-5910) if using GUI

#### Setup Commands

1. **Connect to Instance**
   ```bash
   # SSH to your instance
   ssh -i your-key.pem ubuntu@your-instance-ip
   ```

2. **Install Dependencies**
   ```bash
   # Update system
   sudo apt update && sudo apt upgrade -y

   # Install basic tools
   sudo apt install -y build-essential curl git vim htop
   ```

3. **Install NVIDIA Drivers**
   ```bash
   # Install NVIDIA drivers
   sudo apt install -y nvidia-driver-535 nvidia-utils-535
   sudo reboot
   ```

4. **Install Docker and NVIDIA Container Toolkit**
   ```bash
   # Install Docker
   sudo apt install -y docker.io docker-compose-plugin
   sudo usermod -aG docker ubuntu

   # Install NVIDIA Container Toolkit
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

   sudo apt update
   sudo apt install -y nvidia-container-toolkit
   sudo systemctl restart docker
   ```

5. **Install ROS 2 in Docker**
   ```bash
   # Create a ROS 2 workspace
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws

   # Use ROS 2 Humble Docker image
   docker run -it --gpus all --rm osrf/ros:humble-desktop-full bash
   ```

6. **Set up VNC for GUI Applications**
   ```bash
   # Install VNC server
   sudo apt install -y ubuntu-desktop-minimal tigervnc-standalone-server

   # Configure VNC
   vncserver :1
   vncpasswd
   ```

### Alternative Cloud Options

#### Google Cloud Platform (GCP)
- **Instance Type**: A2 series with A100 GPUs
- **Setup**: Similar to AWS but using gcloud CLI

#### Microsoft Azure
- **Instance Type**: NCas_T4_v3 series with T4 GPUs
- **Setup**: Similar to AWS but using Azure CLI

## Budget Alternative (Simulation Only)

### Software Rendering Setup

For users without dedicated GPUs, this approach uses CPU-based rendering:

#### Installation Steps

1. **Install Ubuntu 22.04 LTS**
   - Same as Local Workstation setup

2. **Install Software Rendering Support**
   ```bash
   # Install Mesa for software rendering
   sudo apt install mesa-utils libgl1-mesa-glx libgl1-mesa-dri

   # Set environment for software rendering
   export LIBGL_ALWAYS_SOFTWARE=1
   ```

3. **Install ROS 2 (No GPU acceleration)**
   - Follow same steps as Local Workstation but without CUDA

4. **Install Lightweight Simulation**
   ```bash
   # Install Ignition Gazebo (lighter than Garden)
   sudo apt install ignition-citadel
   ```

#### Performance Considerations

- **Limitations**: No Isaac Sim support, slower physics simulation
- **Optimizations**: Reduce physics update rate, simplify collision models
- **Alternatives**: Use Webots or lightweight simulators

## Validation Scripts

### Hardware Validation Script

Create a script to validate your setup:

```bash
#!/bin/bash
# validate_hardware.sh

echo "=== Hardware Validation Script ==="

# Check CUDA
if command -v nvidia-smi &> /dev/null; then
    echo "✓ CUDA detected:"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader,nounits
else
    echo "✗ CUDA not detected"
fi

# Check ROS 2
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 Humble detected: $(ros2 --version)"
else
    echo "✗ ROS 2 not detected"
fi

# Check Gazebo
if command -v gz &> /dev/null; then
    echo "✓ Gazebo detected: $(gz --version)"
elif command -v gazebo &> /dev/null; then
    echo "✓ Gazebo classic detected: $(gazebo --version)"
else
    echo "✗ Gazebo not detected"
fi

# Check Docker
if command -v docker &> /dev/null; then
    echo "✓ Docker detected: $(docker --version)"
    if docker run --rm hello-world > /dev/null 2>&1; then
        echo "✓ Docker working correctly"
    else
        echo "✗ Docker not working properly"
    fi
else
    echo "✗ Docker not detected"
fi

echo "=== Validation Complete ==="
```

### Running Validation

```bash
# Make script executable
chmod +x validate_hardware.sh

# Run validation
./validate_hardware.sh
```

## Troubleshooting

### Common Issues and Solutions

#### CUDA Installation Issues
- **Problem**: CUDA not working after installation
- **Solution**: Check GPU compatibility, reinstall drivers, verify PATH and LD_LIBRARY_PATH

#### ROS 2 Installation Issues
- **Problem**: ROS 2 commands not found
- **Solution**: Source setup.bash in ~/.bashrc: `source /opt/ros/humble/setup.bash`

#### Gazebo کارکردگی کے مسائل
- **مسئلہ**: سست سیمیولیشن یا کرشز
- **حل**: فیزکس اپ ڈیٹ ریٹ کو کم کریں، GPU ڈرائیورز چیک کریں، RAM الاؤنس کو بڑھائیں

#### Isaac Sim تنصیب کے مسائل
- **مسئلہ**: Isaac Sim شروع نہیں ہو رہا
- **حل**: GPU مطابقت کی تصدیق کریں، NVIDIA ڈرائیورز چیک کریں، ضروری لائبریریز انسٹال کریں

### سپورٹ وسائل

- **ROS Answers**: [answers.ros.org](https://answers.ros.org)
- **Gazebo کمیونٹی**: [community.gazebosim.org](https://community.gazebosim.org)
- **NVIDIA Isaac Forum**: [forums.developer.nvidia.com](https://forums.developer.nvidia.com)
- **کورس Discord**: کورس مواد میں فراہم کردہ لنک

## اگلے اقدامات

ایک بار جب آپ کا ہارڈویئر سیٹ اپ ہو جائے اور تصدیق ہو جائے، تو آپ آگے بڑھ سکتے ہیں:

- [Module 2: Digital Twin](./module-2-digital-twin/index.md) اگر آپ کے پاس مینی ایچر ٹائر ہارڈویئر ہے
- [Module 1: ROS 2 Fundamentals](./module-1-ros2/index.md) پراکسی ٹائر صارفین کے لیے
- [Module 3: NVIDIA Isaac](./module-3-isaac/index.md) اگر آپ کے پاس مینی ایچر یا پریمیئم ٹائر ہارڈویئر ہے

اپنے سیٹ اپ کو باقاعدگی سے تصدیق کرنا یاد رکھیں اور بہترین کارکردگی کے لیے اپنے نظام کو اپ ڈیٹ رکھیں۔