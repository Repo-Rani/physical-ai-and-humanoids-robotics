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

# باب 0.2: ترقیاتی ماحول کی تنصیب

اپنے ترقیاتی ماحول کی تنصیب آپ کی فزیکل AI کی سفر میں پہلا اہم قدم ہے۔ یہ باب آپ کے نظام کو روبوٹکس کی ترقی کے لیے ضروری ٹولز، لائبریریوں، اور فریم ورکس کے ساتھ ترتیب دینے کے لیے جامع رہنمائی فراہم کرتا ہے۔

## ترقیاتی ماحول کے اختیارات

کورس تین مختلف ترقیاتی ماحول کی تنصیب کے طریقے پیش کرتا ہے، ہر ایک مختلف ہارڈویئر کی صلاحیتوں اور بجٹ کی پابندیوں کے لیے موزوں ہے:

### اختیارات 1: مقامی ترقی (زیادہ تر صارفین کے لیے تجویز کیا گیا)

مقامی ترقی تکراری ترقی اور ٹیسٹنگ کے لیے بہترین کارکردگی فراہم کرتا ہے۔ یہ طریقہ مخصوص ہارڈویئر کی ضروریات کا تقاضا کرتا ہے لیکن سب سے زیادہ ردعملی ترقیاتی تجربہ فراہم کرتا ہے۔

**کم از کم ضروریات:**
- CPU: کوآر-کور پروسیسر (انٹیل i5 یا AMD Ryzen 5 کے برابر)
- RAM: 16GB (32GB سیمیولیشن کے کام کے لیے تجویز کیا گیا)
- اسٹوریج: 50GB خالی جگہ
- OS: Ubuntu 22.04 LTS (تجویز کیا گیا) یا Windows پر WSL2

**تجویز کردہ ضروریات:**
- CPU: 8+ کورز (انٹیل i7/i9 یا AMD Ryzen 7/9)
- RAM: 32GB+
- GPU: NVIDIA RTX 3060+ (سیمیولیشن اور Isaac کے کام کے لیے)
- اسٹوریج: 100GB+ SSD

### اختیارات 2: کلاؤڈ ترقی (سیمیولیشن پر مبنی کام کے لیے تجویز کیا گیا)

کلاؤڈ ترقی مہنگے مقامی آلات کے بغیر اعلیٰ درجے کے ہارڈویئر تک رسائی فراہم کرتا ہے۔ یہ طریقہ طاقتور GPUs کی ضرورت والے سیمیولیشن پر مبنی کام کے لیے مثالی ہے۔

**تجویز کردہ کلاؤڈ انسٹینس:**
- AWS: g5.xlarge (4 vCPU, 16GB RAM, 1x NVIDIA A10G GPU)
- GCP: A2 سیریز (مختلف GPU اختیارات)
- Azure: NCas_T4_v3 سیریز

### اختیارات 3: ہائبرڈ ترقی (دونوں دنیاوں کا بہترین)

ہائبرڈ طریقہ کوڈنگ اور بنیادی ٹیسٹنگ کے لیے مقامی ترقی کا استعمال کرتا ہے، جبکہ سیمیولیشن اور تربیت کے لیے کلاؤڈ وسائل کا استعمال کرتا ہے۔

## Ubuntu 22.04 LTS کی تنصیب

Ubuntu 22.04 LTS اس کورس کے لیے تجویز کردہ آپریٹنگ سسٹم ہے۔ آپ کے پاس Ubuntu چلانے کے لیے کئی اختیارات ہیں:

### اختیارات A: نیٹو انسٹالیشن (ڈوئل بوٹ)

ڈوئل بوٹنگ بہترین کارکردگی فراہم کرتا ہے لیکن آپ کے ہارڈ ڈرائیو کو پارٹیشن کرنے کی ضرورت ہوتی ہے۔

1. ubuntu.com سے Ubuntu 22.04 LTS ڈاؤن لوڈ کریں
2. Rufus (Windows) یا Etcher (macOS/Linux) کا استعمال کرتے ہوئے بوٹ ایبل USB ڈرائیو بنائیں
3. USB ڈرائیو سے بوٹ کریں اور انسٹالیشن کے احکامات پر عمل کریں
4. پارٹیشننگ کے دوران، یقینی بنائیں کہ آپ کے پاس Ubuntu کے لیے کم از کم 50GB جگہ ہے
5. انسٹالیشن مکمل کریں اور اپنے صارف اکاؤنٹ کو ترتیب دیں

### اختیارات B: ورچوئل مشین

ورچوئل مشین آپ کو اپنے موجودہ OS کے ساتھ Ubuntu چلانے کی اجازت دیتا ہے، پارٹیشننگ کی ضرورت نہیں ہوتی۔

1. VirtualBox یا VMware Workstation انسٹال کریں
2. Ubuntu 22.04 ISO کے ساتھ ایک نیا VM بنائیں
3. وسائل کو الاٹ کریں: کم از کم 8GB RAM، 4 CPU کورز، 50GB اسٹوریج
4. VM میں Ubuntu انسٹال کریں، معیاری طریقہ کار پر عمل کریں

### اختیارات C: Windows Subsystem for Linux 2 (WSL2)

WSL2 Windows پر ایک Linux ماحول فراہم کرتا ہے، اچھا انٹیگریشن کے ساتھ۔

1. Windows Features میں WSL2 کو فعال کریں
2. Microsoft Store سے Ubuntu 22.04 انسٹال کریں
3. پیکیجز کو اپ ڈیٹ کریں: `sudo apt update && sudo apt upgrade`
4. ضرورت کے مطابق ترقیاتی ٹولز کو ترتیب دیں

## ROS 2 Humble Hawksbill کی تنصیب

ROS 2 Humble Hawksbill وہ LTS (Long Term Support) ورژن ہے جو اس کورس میں استعمال ہوتا ہے۔

### ROS 2 ریپوزٹیری کی ترتیب

```bash
# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
sudo apt update
```

### ROS 2 پیکیجز کی تنصیب

```bash
# Install the ROS 2 desktop package (includes Gazebo)
sudo apt install -y ros-humble-desktop ros-humble-ros-base

# Install development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

### ROS 2 ماحول کی ترتیب

اپنے `~/.bashrc` فائل میں درج ذیل کو شامل کریں:

```bash
source /opt/ros/humble/setup.bash
```

پھر اپنے ماحول کو دوبارہ لوڈ کریں:

```bash
source ~/.bashrc
```

## Docker کی تنصیب

Docker قابل اعادہ ترقیاتی ماحول اور کنٹینرائزڈ ایپلی کیشنز کو فعال بناتا ہے۔

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

Docker گروپ میں خود کو شامل کرنے کے بعد، تبدیلیوں کو لاگو کرنے کے لیے لاگ آؤٹ کریں اور دوبارہ لاگ ان کریں۔

## تصدیق: اپنی ترتیب کا ٹیسٹ کرنا

یقینی بنائیں کہ آپ کا ماحول صحیح طریقے سے ترتیب دیا گیا ہے:

```bash
# Test ROS 2 installation
ros2 --version

# Test that ROS 2 commands are available
ros2 topic list

# Test Docker installation
docker --version
docker run hello-world
```

آپ کو ROS 2 اور Docker کے لیے ورژن کی معلومات دیکھنی چاہیے، اور hello-world کنٹینر کامیابی سے چلنا چاہیے۔

## اگلا باب

اگلے باب میں، [باب 0.3: فزیکل AI کا تعارف](./chapter-0-3.md)، آپ فزیکل AI کے نظریاتی بنیادوں کا جائزہ لیں گے، جس میں جسمانی فرضیہ اور سینسریموٹر گراؤنڈنگ کے تصورات شامل ہیں جو اسے روایتی ڈیجیٹل AI سے ممتاز کرتے ہیں۔