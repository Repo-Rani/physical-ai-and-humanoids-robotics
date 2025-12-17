---
id: glossary
title: "Glossary of Terms"
sidebar_label: "Glossary"
sidebar_position: 11
description: "Comprehensive glossary of technical terms, acronyms, and definitions for Physical AI & Humanoid Robotics curriculum with cross-references to chapters"
keywords: [glossary, terms, definitions, robotics, ai, acronym, dictionary]
estimated_time: 30
prerequisites: []
learning_outcomes:
  - Define key technical terms used throughout the curriculum
  - Understand acronyms and their context in robotics and AI
  - Locate chapters where terms are introduced or explained in depth
  - Use glossary as reference for technical communication
  - Navigate curriculum using cross-referenced terms
hardware_tier: none
---

# Glossary of Terms

This glossary provides definitions for technical terms, acronyms, and concepts used throughout the Physical AI & Humanoid Robotics curriculum. Terms are organized alphabetically with cross-references to relevant chapters.

## A

### Action
A communication pattern in ROS 2 designed for long-running tasks that provide feedback during execution and can be preempted. Actions are ideal for navigation, manipulation, and other goal-oriented tasks. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

### Action Server
A ROS 2 node that implements the action pattern, accepting goals, providing feedback, and returning results. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

### Algorithmic Differentiation (AD)
A technique for computing derivatives of functions specified by computer programs, used in robotics for trajectory optimization and control. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

### Apache License 2.0
An open-source software license that allows users to freely use, modify, and distribute software while preserving copyright notices and disclaimers. Used by ROS 2 and many robotics frameworks. ([Chapter 1.3](./module-1-ros2/chapter-1-3.md))

### Application Programming Interface (API)
A set of rules and protocols for building and interacting with software applications. In robotics, APIs allow communication between different software components. ([Chapter 4.4](./module-4-vla/chapter-4-4.md))

### Artificial Intelligence (AI)
The simulation of human intelligence processes by machines, especially computer systems. In robotics, AI enables perception, decision-making, and learning capabilities. ([Chapter 0.3](./module-0-getting-started/chapter-0-3.md))

### Asynchronous Programming
A programming paradigm that allows multiple operations to run concurrently without blocking the main execution thread. Essential for responsive robotic systems. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

## B

### Behavior Tree
A hierarchical structure used in robotics for organizing and executing complex behaviors. Behavior trees provide a modular approach to robot control and decision-making. ([Chapter 3.6](./module-3-isaac/chapter-3-6.md))

### Best Effort (QoS Policy)
A Quality of Service policy in ROS 2 where messages may be lost with no retries. Suitable for sensor data where some loss is acceptable. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Bidirectional Encoder Representations from Transformers (BERT)
A transformer-based machine learning technique for natural language processing pre-training. Used in some robotics applications for language understanding. ([Chapter 4.3](./module-4-vla/chapter-4-3.md))

### Bridge
A component that connects two different communication systems or protocols. In robotics, bridges often connect ROS 2 with other simulation or hardware platforms. ([Chapter 2.5](./module-2-digital-twin/chapter-2-5.md))

### Build System
Software tools that automate the process of compiling source code into executable programs. In ROS 2, Colcon is the primary build system. ([Chapter 1.3](./module-1-ros2/chapter-1-3.md))

## C

### Cartesian Space
The three-dimensional space defined by X, Y, and Z coordinates, used to describe positions and movements in robotics. ([Chapter 3.6](./module-3-isaac/chapter-3-6.md))

### Chain of Thought (CoT)
A reasoning approach where AI models generate intermediate reasoning steps to solve complex problems. Used in robotics for task planning and decision-making. ([Chapter 4.3](./module-4-vla/chapter-4-3.md))

### Collision Detection
The computational problem of detecting whether two or more objects are intersecting. Essential for safe robot navigation and manipulation. ([Chapter 2.3](./module-2-digital-twin/chapter-2-3.md))

### Command Line Interface (CLI)
A text-based interface for interacting with computer programs or operating systems. Essential for robotics development and system administration. ([Chapter 0.2](./module-0-getting-started/chapter-0-2.md))

### Colcon
The build system used in ROS 2 for building and installing packages. Colcon supports multiple build systems and package types. ([Chapter 1.3](./module-1-ros2/chapter-1-3.md))

### Computational Graph
The network of nodes and topics that represents all processes and communication channels in a ROS system. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Computer Vision
A field of artificial intelligence that trains computers to interpret and understand the visual world. Used extensively in robotics for perception and navigation. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Continuous Joint
A type of joint in URDF that allows unlimited rotation around a single axis, like a wheel or continuous rotation servo. ([Chapter 1.4](./module-1-ros2/chapter-1-4.md))

### Control Theory
An interdisciplinary branch of engineering and mathematics that deals with the behavior of dynamical systems with inputs, and how their behavior is modified by feedback. Fundamental to robotics. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Convolutional Neural Network (CNN)
A class of deep neural networks commonly applied to analyzing visual imagery. Widely used in robotics for object detection and scene understanding. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

## D

### Data Distribution Service (DDS)
A middleware standard for real-time, scalable, and reliable communication that forms the foundation of ROS 2. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Deep Learning
A subset of machine learning based on artificial neural networks with representation learning. Used in robotics for perception, control, and decision-making. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

### Deep Neural Network (DNN)
An artificial neural network with multiple layers between the input and output layers. Used in robotics for complex pattern recognition and decision-making. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Denavit-Hartenberg (DH) Parameters
A convention for defining coordinate frames on robotic manipulator links, used for kinematic analysis. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Digital Twin
A virtual representation of a physical system that can be used for simulation, testing, and optimization before deployment to the real system. ([Chapter 2.1](./module-2-digital-twin/chapter-2-1.md))

### Docker
A containerization platform that enables applications to run reliably when moved from one computing environment to another. Used in robotics for reproducible environments. ([Chapter 0.2](./module-0-getting-started/chapter-0-2.md))

### Domain Randomization
A technique for training machine learning models by randomizing the simulation environment to improve sim-to-real transfer. ([Chapter 3.2](./module-3-isaac/chapter-3-2.md))

### Dynamic Movement Primitives (DMP)
A method for learning and reproducing movement patterns in robotics, particularly useful for manipulation tasks. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

## E

### Embodiment Hypothesis
The theory that intelligence emerges from the interaction between an agent and its environment, with the physical body contributing to computation. ([Chapter 0.3](./module-0-getting-started/chapter-0-3.md))

### Emergency Stop (E-Stop)
A safety mechanism that immediately stops robot operation in emergency situations. Critical for safe human-robot interaction. ([Chapter 5.2](./module-5-capstone/chapter-5-2.md))

### End Effector
The device at the end of a robotic arm designed to interact with the environment, such as a gripper or tool. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Event Handler
A callback routine that operates asynchronously in response to events, used in ROS 2 launch files for managing node lifecycles. ([Chapter 1.5](./module-1-ros2/chapter-1-5.md))

### Extended Kalman Filter (EKF)
A nonlinear version of the Kalman filter that linearizes around the current estimate, used in robotics for state estimation. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

## F

### Forward Kinematics
The process of determining the position and orientation of the end effector given the joint angles of a robotic manipulator. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Framework
A reusable software platform that provides a foundation for developing applications. ROS 2 is a framework for robotics development. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Fused Multiply-Add (FMA)
A hardware instruction that performs a multiplication and addition in a single operation, important for GPU computing in robotics. ([Chapter 3.1](./module-3-isaac/chapter-3-1.md))

## G

### Gaussian Process
A non-parametric method for regression and classification that provides uncertainty estimates, useful in robotics for modeling uncertain environments. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

### Generalized Coordinates
A set of parameters that define the configuration of a mechanical system, used in robotics for describing robot states. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Geographic Information System (GIS)
A system designed to capture, store, manipulate, analyze, manage, and present spatial or geographic data. Used in outdoor robotics navigation. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

### Gazebo
A 3D simulation environment for robotics that provides accurate physics simulation and sensor models. ([Chapter 2.2](./module-2-digital-twin/chapter-2-2.md))

### Generative Adversarial Network (GAN)
A class of machine learning frameworks where two neural networks contest with each other, used in robotics for generating synthetic training data. ([Chapter 3.2](./module-3-isaac/chapter-3-2.md))

### Geometry_msgs
A ROS 2 package containing messages for common geometric primitives such as points, vectors, and poses. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

### GPU Computing
The use of graphics processing units for general-purpose computing, essential for deep learning and computer vision in robotics. ([Chapter 3.1](./module-3-isaac/chapter-3-1.md))

## H

### Hardware-in-the-Loop (HIL)
A testing technique that involves physical components in a simulation environment, used for validating robotic systems. ([Chapter 5.2](./module-5-capstone/chapter-5-2.md))

### Haptic Feedback
The use of tactile sensations to communicate information to the user, important for teleoperation and human-robot interaction. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Head-Mounted Display (HMD)
A display device worn on the head that provides visual output directly to the user, used in virtual reality applications for robotics. ([Chapter 2.6](./module-2-digital-twin/chapter-2-6.md))

### Human-Robot Interaction (HRI)
The study of interactions between humans and robots, focusing on design, development, and evaluation of robotic systems for human use. ([Chapter 4.5](./module-4-vla/chapter-4-5.md))

### Hyperparameter
A parameter whose value is set before the learning process begins, used in machine learning and robotics algorithm configuration. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

## I

### Inertial Measurement Unit (IMU)
A device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body. ([Chapter 2.4](./module-2-digital-twin/chapter-2-4.md))

### Inverse Kinematics
The mathematical process of determining the joint angles required to achieve a desired end-effector position and orientation. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Isaac Sim
NVIDIA's robotics simulation application based on NVIDIA Omniverse, providing photorealistic simulation for robotics development. ([Chapter 3.2](./module-3-isaac/chapter-3-2.md))

### Isaac ROS
NVIDIA's collection of hardware-accelerated perception and AI packages for ROS 2, designed to accelerate robotics applications on NVIDIA platforms. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Iterative Closest Point (ICP)
An algorithm used to minimize the distance between points of two point clouds, commonly used in robotics for localization and mapping. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

## J

### Joint
In URDF, a connection between two links that specifies how they can move relative to each other. ([Chapter 1.4](./module-1-ros2/chapter-1-4.md))

### Joint Space
The space defined by the joint angles of a robotic manipulator, used for motion planning and control. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Jacobian Matrix
A matrix of all first-order partial derivatives of a vector-valued function, used in robotics for relating joint velocities to end-effector velocities. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

## K

### Kalman Filter
An algorithm that uses a series of measurements observed over time to estimate unknown variables, widely used in robotics for state estimation. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Kinematics
The branch of mechanics that deals with motion without reference to force or mass, essential for understanding robot movement. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Kinesthetic Teaching
A method of teaching robot motions by physically guiding the robot through desired movements, used for programming manipulation tasks. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

## L

### Language Model Integration
The process of incorporating large language models into robotic systems for natural language understanding and task planning. ([Chapter 4.3](./module-4-vla/chapter-4-3.md))

### Launch File
A ROS 2 file that specifies which nodes to run and how to configure them, enabling complex system orchestration. ([Chapter 1.5](./module-1-ros2/chapter-1-5.md))

### Lidar (Light Detection and Ranging)
A remote sensing method that uses light in the form of a pulsed laser to measure distances, commonly used in robotics for navigation and mapping. ([Chapter 2.4](./module-2-digital-twin/chapter-2-4.md))

### Linux Operating System
An open-source Unix-like operating system that serves as the foundation for most robotics development platforms. ([Chapter 0.2](./module-0-getting-started/chapter-0-2.md))

### Localization
The process of determining the position and orientation of a robot within its environment, essential for autonomous navigation. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Long Short-Term Memory (LSTM)
A type of recurrent neural network capable of learning order dependence in sequence prediction problems, used in robotics for temporal pattern recognition. ([Chapter 4.1](./module-4-vla/chapter-4-1.md))

## M

### Machine Learning (ML)
A subset of artificial intelligence that enables systems to learn and improve from experience without being explicitly programmed. Widely used in robotics for perception and control. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

### Manipulation
The process of grasping, moving, and controlling objects using a robotic end effector, a key capability for humanoid robots. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Manipulator
A robotic device used to manipulate objects, typically consisting of multiple joints and links. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Mapping
The process of creating a representation of the environment, typically used in conjunction with localization for navigation. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Markov Decision Process (MDP)
A mathematical framework for modeling decision-making in situations where outcomes are partly random and partly under the control of a decision maker. Used in robotics for planning. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

### Mermaid Diagram
A diagramming and charting tool that uses markdown-inspired text definitions to create and modify diagrams dynamically. ([Chapter 4.2](./module-4-vla/chapter-4-2.md))

### Middleware
Software that provides common services and capabilities to applications beyond what's offered by the operating system, such as DDS in ROS 2. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### MoveIt
A motion planning framework for ROS that provides tools for path planning, inverse kinematics, and manipulation. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

## N

### Navigation
The process of planning and executing paths for a robot to move from one location to another, often involving mapping and localization. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

### Navigation2 (Nav2)
The current navigation stack for ROS 2, providing path planning, obstacle avoidance, and robot control for autonomous navigation. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

### Neural Radiance Fields (NeRF)
A method for synthesizing novel views of complex 3D scenes based on a collection of 2D images, used in robotics for 3D scene understanding. ([Chapter 3.2](./module-3-isaac/chapter-3-2.md))

### Node
In ROS, a process that performs computation. Nodes are the fundamental building blocks of ROS applications. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Node Composition
A ROS 2 feature that allows multiple nodes to run in the same process, reducing communication overhead. ([Chapter 1.5](./module-1-ros2/chapter-1-5.md))

### Non-Holonomic Constraint
A constraint that cannot be integrated to form a constraint on only the coordinates, common in wheeled robot kinematics. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

## O

### Object Detection
The computer technology related to searching for and locating objects in images or videos, essential for robotic perception. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Omnidirectional Robot
A robot that can move in any direction regardless of the orientation of its body, providing greater mobility than conventional wheeled robots. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

### OpenAI API
An application programming interface that provides access to OpenAI's language models, used in robotics for natural language processing. ([Chapter 4.4](./module-4-vla/chapter-4-4.md))

### OpenCV
An open-source computer vision and machine learning software library that provides tools for image processing and computer vision. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Operational Space Control
A control method that operates in the task space rather than the joint space, commonly used for robotic manipulation. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Optimization-Based Control
A control approach that formulates control as an optimization problem, used in advanced robotics applications. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

## P

### Parameter Server
A component in ROS that provides a shared configuration system for storing and retrieving parameters used by nodes. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

### Path Planning
The computational problem of finding a sequence of valid configurations that moves an object from a starting configuration to a goal configuration. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

### Perception
The ability of a robot to interpret and understand its environment through sensors, essential for autonomous operation. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Physical AI
A paradigm of artificial intelligence that emphasizes the importance of physical embodiment and interaction with the real world for developing intelligent systems. ([Chapter 0.3](./module-0-getting-started/chapter-0-3.md))

### Point Cloud
A set of data points in space, typically produced by 3D scanners or depth cameras, used in robotics for environment mapping. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Pose
The position and orientation of a robot or object in space, typically represented by a position vector and rotation matrix or quaternion. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Prismatic Joint
A type of joint in URDF that allows linear motion along a single axis, like a sliding mechanism. ([Chapter 1.4](./module-1-ros2/chapter-1-4.md))

### Publisher-Subscriber Pattern
A messaging pattern in ROS where publishers send messages to topics and subscribers receive messages from topics without direct connection. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

## Q

### Quality of Service (QoS)
A set of policies in ROS 2 that define how messages are delivered, including reliability, durability, and history characteristics. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Quaternion
A mathematical concept used to represent rotations in 3D space, preferred over Euler angles to avoid gimbal lock in robotics applications. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Q-Learning
A model-free reinforcement learning algorithm that learns a policy telling an agent what action to take under what circumstances. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

## R

### Reinforcement Learning (RL)
A type of machine learning where an agent learns to make decisions by performing actions and receiving rewards or penalties. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

### Robot Operating System (ROS)
A flexible framework for writing robot software that provides services designed for a heterogeneous computer cluster. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Robot Operating System 2 (ROS 2)
The second generation of ROS with improved architecture for real-time systems, multi-robot systems, and commercial applications. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### Robot State Publisher
A ROS package that helps publish transformations for non-fixed joints in a robot, used for visualization and navigation. ([Chapter 1.4](./module-1-ros2/chapter-1-4.md))

### Robotic Process Automation (RPA)
The use of software robots to automate routine tasks, distinct from physical robotics but related to AI automation. ([Chapter 4.1](./module-4-vla/chapter-4-1.md))

### ROS 2 Humble Hawksbill
The Long-Term Support (LTS) distribution of ROS 2 used throughout this curriculum, released in May 2022. ([Chapter 0.2](./module-0-getting-started/chapter-0-2.md))

### ROS 2 Middleware
The communication layer in ROS 2 based on DDS (Data Distribution Service) that enables message passing between nodes. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

### RRT (Rapidly-exploring Random Tree)
A path planning algorithm that incrementally builds a tree of possible paths, widely used in robotics for motion planning. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

## S

### Sensor Fusion
The process of combining data from multiple sensors to improve the accuracy and reliability of information, crucial for robust robotic perception. ([Chapter 2.4](./module-2-digital-twin/chapter-2-4.md))

### Service
A synchronous request-reply communication pattern in ROS where a client sends a request to a server and waits for a response. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

### Sim-to-Real Transfer
The process of transferring knowledge or skills learned in simulation to real-world robotic systems, a major challenge in robotics. ([Chapter 3.6](./module-3-isaac/chapter-3-6.md))

### Simulation
The use of computer models to represent the behavior of a real system, essential for testing robotics algorithms safely and efficiently. ([Chapter 2.1](./module-2-digital-twin/chapter-2-1.md))

### SLAM (Simultaneous Localization and Mapping)
A computational problem where a robot builds a map of an unknown environment while simultaneously keeping track of its location within that map. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### State Machine
A computational model used to design algorithms that can be in exactly one of a finite number of states at any given time, common in robotics control. ([Chapter 5.2](./module-5-capstone/chapter-5-2.md))

### Subscriber
A ROS node that receives messages from topics in the publisher-subscriber communication pattern. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

### Synchronous Communication
A communication pattern where the sender waits for a response before continuing, used in ROS services. ([Chapter 1.2](./module-1-ros2/chapter-1-2.md))

## T

### Teleoperation
The remote operation of a robot by a human operator, often used for tasks too dangerous or difficult for autonomous operation. ([Chapter 5.2](./module-5-capstone/chapter-5-2.md))

### Transformer Architecture
A deep learning model architecture that uses self-attention mechanisms, foundational for modern language models used in robotics. ([Chapter 4.1](./module-4-vla/chapter-4-1.md))

### Trajectory Optimization
The process of finding the optimal path or trajectory for a robot to follow, considering constraints and objectives. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

### Trust Region Policy Optimization (TRPO)
A reinforcement learning algorithm that constrains the policy updates to ensure stable learning, used in robotics applications. ([Chapter 3.5](./module-3-isaac/chapter-3-5.md))

## U

### Unified Robot Description Format (URDF)
An XML format for representing robot models in ROS, describing links, joints, and their properties. ([Chapter 1.4](./module-1-ros2/chapter-1-4.md))

### Ubuntu
A Linux distribution that serves as the primary operating system for ROS development and robotics applications. ([Chapter 0.2](./module-0-getting-started/chapter-0-2.md))

### Universal Scene Description (USD)
A 3D scene description and file format developed by Pixar, used in Isaac Sim for scene representation. ([Chapter 3.2](./module-3-isaac/chapter-3-2.md))

### Unity Robotics
A collection of tools and packages that enable robotics simulation and development within the Unity game engine. ([Chapter 2.6](./module-2-digital-twin/chapter-2-6.md))

### User Datagram Protocol (UDP)
A communications protocol that provides low-latency communication between applications, sometimes used in robotics for sensor data transmission. ([Chapter 1.1](./module-1-ros2/chapter-1-1.md))

## V

### Vision-Language-Action (VLA) Models
Advanced AI models that integrate visual perception, language understanding, and action execution for robotic manipulation and interaction. ([Chapter 4.1](./module-4-vla/chapter-4-1.md))

### Visual Servoing
A control strategy that uses visual feedback to control the motion of a robot, commonly used in robotic manipulation. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Visual Simultaneous Localization and Mapping (VSLAM)
A technique for building a map of an unknown environment using visual sensors while simultaneously keeping track of the camera's location within that map. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Virtual Reality (VR)
A computer-generated simulation of a three-dimensional environment that can be interacted with in a seemingly real way, used in robotics for training and simulation. ([Chapter 2.6](./module-2-digital-twin/chapter-2-6.md))

## W

### Waypoint Navigation
A navigation method where a robot moves to a series of predetermined positions in space, commonly used in autonomous mobile robots. ([Chapter 3.4](./module-3-isaac/chapter-3-4.md))

### Webots
A robot simulation software that provides a complete development environment for robotics research and education. ([Chapter 2.2](./module-2-digital-twin/chapter-2-2.md))

### Wheel Odometry
The use of sensors to measure the rotation of wheels to estimate the distance traveled and position of a robot. ([Chapter 3.3](./module-3-isaac/chapter-3-3.md))

### Workspace
The volume of space that a robot manipulator can reach, important for task planning and robot placement. ([Chapter 5.6](./module-5-capstone/chapter-5-6.md))

## X

### Xacro
An XML macro language for generating URDF files, allowing parameterization and reuse of robot descriptions. ([Chapter 1.4](./module-1-ros2/chapter-1-4.md))

## Y

### YAML (YAML Ain't Markup Language)
A human-readable data serialization language commonly used in ROS for configuration files and parameters. ([Chapter 1.5](./module-1-ros2/chapter-1-5.md))

## Z

### Zero-Shot Learning
A machine learning paradigm where a model can recognize objects or perform tasks it has never seen before, relevant for robotics in novel environments. ([Chapter 4.1](./module-4-vla/chapter-4-1.md))

### Z-Buffer
A component in computer graphics that stores depth information for 3D rendering, used in simulation environments for realistic rendering. ([Chapter 2.6](./module-2-digital-twin/chapter-2-6.md))

---

**Glossary Complete**: This glossary contains technical terms from all modules of the Physical AI & Humanoid Robotics curriculum. For terms not listed here, please refer to the specific chapter where they are introduced.