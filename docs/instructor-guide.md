---
id: instructor-guide
title: "Instructor Guide"
sidebar_label: "Instructor Guide"
sidebar_position: 12
description: "Comprehensive instructor resources: lecture slides, solution keys, grading rubrics, suggested lab configurations, and equipment lists for Physical AI & Humanoid Robotics curriculum"
keywords: [instructor-guide, lecture-slides, solution-keys, grading-rubrics, lab-configurations, equipment-lists]
estimated_time: 60
prerequisites: []
learning_outcomes:
  - Access lecture slides and presentation materials for each module
  - Find solution keys for exercises and practical activities
  - Apply grading rubrics for assessments and capstone projects
  - Configure suggested lab environments for institutional setups
  - Identify required equipment for 20-student robotics laboratory
hardware_tier: none
---

# Instructor Guide

This guide provides comprehensive resources for instructors teaching the Physical AI & Humanoid Robotics curriculum. It includes lecture materials, solution keys, grading rubrics, and laboratory setup recommendations.

## Course Overview

### Course Structure
- **Duration**: 13 weeks (full semester)
- **Format**: Combination of lectures, hands-on labs, and project work
- **Prerequisites**: Basic programming knowledge, introductory robotics concepts
- **Target Audience**: Undergraduate/graduate students, professional development

### Learning Objectives
By the end of this course, students will be able to:
- Design and implement ROS 2-based robotic systems
- Integrate perception, planning, and control components
- Simulate robotic systems using Gazebo and Unity
- Apply AI techniques to robotic problems
- Build autonomous humanoid robot systems

## Lecture Materials

### Module 0: Getting Started
- **Lecture Slides**: [Module 0 Slides](#) - Course orientation, Physical AI concepts, development environment setup
- **Video Content**: [Introduction to Physical AI](#) - 45-minute lecture on embodiment hypothesis and sensorimotor grounding
- **Supplementary Reading**: [Recommended papers on Physical AI](#)

### Module 1: ROS 2 Fundamentals
- **Lecture Slides**: [Module 1 Slides](#) - ROS 2 architecture, communication patterns, package management
- **Lab Exercises**: [ROS 2 Basics Lab](#) - Creating nodes, topics, services, and actions
- **Code Examples**: [Complete ROS 2 Package Examples](#)

### Module 2: Digital Twin
- **Lecture Slides**: [Module 2 Slides](#) - Gazebo simulation, physics modeling, sensor simulation
- **Lab Exercises**: [Gazebo Simulation Lab](#) - Creating robot models, simulation environments
- **Code Examples**: [Gazebo Integration Examples](#)

### Module 3: NVIDIA Isaac
- **Lecture Slides**: [Module 3 Slides](#) - Isaac Sim, Isaac ROS, VSLAM, Nav2 integration
- **Lab Exercises**: [Isaac Integration Lab](#) - GPU-accelerated perception and navigation
- **Code Examples**: [Isaac ROS Examples](#)

### Module 4: Vision-Language-Action Models
- **Lecture Slides**: [Module 4 Slides](#) - VLA models, LLM integration, multimodal interaction
- **Lab Exercises**: [VLA Integration Lab](#) - Natural language control and multimodal systems
- **Code Examples**: [VLA Model Examples](#)

### Module 5: Capstone Project
- **Lecture Slides**: [Module 5 Slides](#) - System integration, capstone project requirements
- **Project Guidelines**: [Capstone Project Specifications](#)
- **Assessment Criteria**: [Capstone Rubric](#)

## Solution Keys

### Module 0 Exercises
- **Exercise 0.1**: Development environment validation
  - **Solution**: Students should verify ROS 2 installation with `ros2 --version` and run basic publisher/subscriber example
  - **Common Issues**: PATH configuration, workspace sourcing, permission issues with Docker

- **Exercise 0.2**: Physical AI concepts
  - **Solution**: Students should articulate the difference between Physical AI and Digital AI, provide examples of embodied cognition
  - **Assessment**: Look for understanding of embodiment hypothesis and sensorimotor grounding

### Module 1 Exercises
- **Exercise 1.1**: Node creation and communication
  - **Solution**: Complete publisher-subscriber implementation with proper error handling and logging
  - **Code Review**: Check for proper QoS configuration, resource management, and documentation

- **Exercise 1.2**: URDF robot model
  - **Solution**: Create valid URDF with proper kinematic chain, visual/collision geometry
  - **Validation**: Use `check_urdf` and visualize in RViz2

### Module 2 Exercises
- **Exercise 2.1**: Gazebo simulation setup
  - **Solution**: Create simulation world with robot model, verify physics parameters
  - **Testing**: Robot should spawn correctly, physics should behave as expected

- **Exercise 2.2**: Sensor integration
  - **Solution**: Add LiDAR, camera, and IMU sensors to robot model with proper noise parameters
  - **Validation**: Sensor data should be published with realistic values and noise

### Module 3 Exercises
- **Exercise 3.1**: Isaac Sim integration
  - **Solution**: Launch robot in Isaac Sim, verify photorealistic rendering
  - **Testing**: Scene should render with proper lighting and materials

- **Exercise 3.2**: VSLAM implementation
  - **Solution**: Configure Isaac ROS VSLAM pipeline, verify pose estimation
  - **Validation**: Robot should localize and map environment successfully

### Module 4 Exercises
- **Exercise 4.1**: Voice command processing
  - **Solution**: Integrate OpenAI Whisper for ASR, implement command validation
  - **Testing**: Voice commands should be recognized and converted to robot actions

- **Exercise 4.2**: LLM integration
  - **Solution**: Connect GPT model for task planning, implement safety filtering
  - **Validation**: LLM should generate appropriate action sequences for given tasks

### Module 5 Exercises
- **Exercise 5.1**: System integration
  - **Solution**: Integrate all components into cohesive autonomous system
  - **Testing**: All subsystems should work together without conflicts

- **Exercise 5.2**: Capstone demonstration
  - **Solution**: Complete autonomous task using all learned concepts
  - **Assessment**: System should operate autonomously with minimal intervention

## Grading Rubrics

### Module Assessments (Each worth 10% of final grade)
- **Technical Implementation** (40%): Code quality, functionality, error handling
- **Documentation** (20%): Comments, README, parameter descriptions
- **Testing** (20%): Unit tests, integration tests, validation scripts
- **Presentation** (20%): Code clarity, adherence to standards, modularity

### Capstone Project (20% of final grade)
- **System Architecture** (20%): Clear separation of components, design patterns
- **Voice Command Processing** (15%): ASR accuracy, natural language understanding, safety validation
- **Navigation** (15%): Path planning, obstacle avoidance, goal reaching
- **Perception** (15%): Object detection, pose estimation, scene understanding
- **Manipulation** (15%): Grasping success, motion planning, force control
- **Integration & Robustness** (10%): Multi-node coordination, error handling, recovery
- **Documentation** (10%): Code comments, system diagrams, demonstration video

### Participation and Labs (10% of final grade)
- **Attendance**: Regular attendance at lectures and labs
- **Participation**: Active engagement in discussions and activities
- **Lab Completion**: Successful completion of hands-on exercises

## Suggested Lab Configurations

### 20-Student Lab Setup

#### Hardware Requirements
- **Workstations**: 20 units with specifications:
  - CPU: Intel i9-12900K or AMD Ryzen 9 5900X
  - GPU: NVIDIA RTX 4070 Ti (12GB) or better
  - RAM: 64GB DDR4-3600MHz
  - Storage: 2TB NVMe SSD
  - Network: Gigabit Ethernet, WiFi 6 support

- **Robot Platforms**: 10 Physical AI Edge Kits
  - Jetson Orin Nano development kits
  - RealSense D435i depth cameras
  - USB microphones for voice input
  - Basic manipulator arms

- **Networking Infrastructure**:
  - Managed Gigabit switches
  - WiFi 6 access points for wireless robot communication
  - Network-attached storage for shared resources

#### Software Configuration
- **Operating System**: Ubuntu 22.04 LTS on all workstations
- **ROS 2**: Humble Hawksbill installation
- **Simulation Software**: Gazebo Garden, Isaac Sim 2023.1.1
- **Development Tools**: VS Code with ROS extensions, Git, Docker
- **Version Control**: GitLab or GitHub Enterprise instance

#### Safety Considerations
- **Physical Safety**: Safety barriers around robot operation areas
- **Electrical Safety**: Proper grounding, GFCI outlets
- **Emergency Procedures**: Emergency stop buttons, evacuation procedures
- **Training**: Safety training for all students before robot operation

## Equipment Lists

### Digital Twin Workstation (Per Student)
- **Computer**: 32GB RAM, RTX 3060 (12GB) or better
- **Monitor**: 24" 1080p or 4K display
- **Input Devices**: Mechanical keyboard, precision mouse
- **Networking**: Gigabit Ethernet, WiFi adapter

### Physical AI Edge Kit (Shared, 1 per 2 students)
- **Compute**: Jetson Orin Nano Developer Kit
- **Sensors**:
  - Intel RealSense D435i depth camera
  - 9-axis IMU
  - Microphone array
- **Actuators**: Basic manipulator arm with gripper
- **Chassis**: Mobile base platform
- **Power**: Battery pack with charging station

### Laboratory Equipment
- **Safety Equipment**: Safety glasses, first aid kit, fire extinguisher
- **Tools**: Multimeters, screwdrivers, cable testers
- **Consumables**: Cables, adapters, replacement parts
- **Storage**: Lockable cabinets for equipment storage

## Course Administration

### LMS Integration
- **Content Delivery**: Module materials, assignments, resources
- **Assessment Tools**: Quiz platforms, assignment submission
- **Communication**: Discussion forums, announcements
- **Grading**: Gradebook integration, rubric tools

### Student Support
- **Office Hours**: Weekly scheduled time for student questions
- **TA Support**: Graduate student assistants for lab supervision
- **Online Resources**: Discussion forums, video tutorials, documentation
- **Peer Support**: Student mentorship program for advanced students

## Assessment Strategies

### Formative Assessments
- **Weekly Quizzes**: 10-15 minute online quizzes on module content
- **Code Reviews**: Peer review of implementation projects
- **Lab Checkpoints**: In-lab demonstrations of concepts learned

### Summative Assessments
- **Module Projects**: End-of-module comprehensive projects
- **Midterm Exam**: Written exam covering first half of curriculum
- **Final Project**: Capstone autonomous robot demonstration
- **Final Exam**: Comprehensive written exam

## Accessibility Considerations

### Universal Design
- **Visual Impairment**: Audio descriptions, screen reader compatibility
- **Motor Impairment**: Alternative input methods, adjustable workstations
- **Hearing Impairment**: Captions on videos, visual alerts
- **Cognitive**: Clear instructions, multiple representation formats

### Accommodation Resources
- **Disability Services**: Campus disability support office
- **Assistive Technology**: Screen readers, voice recognition software
- **Alternative Formats**: Large print, Braille, audio materials

## Course Evaluation

### Student Feedback
- **Mid-semester Survey**: Anonymous feedback on course progress
- **End-of-semester Evaluation**: Comprehensive course evaluation
- **Focus Groups**: Small group discussions on specific topics

### Continuous Improvement
- **Curriculum Review**: Annual review of content and delivery
- **Technology Updates**: Regular updates for new ROS 2 features
- **Industry Input**: Advisory board with industry professionals
- **Student Outcomes**: Tracking of job placement and career success

## Additional Resources

### Recommended Textbooks
- **Primary**: "Programming Robots with ROS" by Morgan Quigley
- **Supplementary**: "Robotics, Vision and Control" by Peter Corke
- **Advanced**: "Principles of Robot Motion" by Howie Choset

### Online Resources
- **ROS Documentation**: [docs.ros.org](https://docs.ros.org)
- **Gazebo Tutorials**: [gazebosim.org/tutorials](https://gazebosim.org/tutorials)
- **NVIDIA Isaac**: [developer.nvidia.com/isaac](https://developer.nvidia.com/isaac)

### Professional Development
- **Workshops**: Annual ROSCon conference
- **Certification**: ROS Industrial training programs
- **Networking**: IEEE Robotics and Automation Society chapters

---

**Instructor Resources Updated**: This guide is maintained by the curriculum development team. For updates or questions, contact [curriculum-team@institution.edu].