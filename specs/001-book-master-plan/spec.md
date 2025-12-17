# Feature Specification: Book Master Plan

**Feature Branch**: `001-book-master-plan`
**Created**: 2025-12-09
**Status**: Draft
**Input**: 13-week Physical AI & Humanoid Robotics textbook with Docusaurus platform

---

## Clarifications

### Session 2025-12-09

- Q: How should assessment checkpoints (quizzes) be implemented in Docusaurus? → A: External quiz platform (e.g., Quizlet embeds, Google Forms) - embed third-party quiz tools via iframes; students take quizzes on external platform
- Q: Which search solution should be used (Algolia vs. local search plugin)? → A: Algolia DocSearch (free for open-source projects) - apply for Algolia DocSearch program; automatic crawling and indexing with ML-powered relevance
- Q: What content should the homepage (intro.md) contain? → A: Course overview with module cards - brief intro, 5 module cards (title, duration, icon), hardware requirements summary, "Get Started" CTA to Module 0

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Navigate Curriculum Structure (Priority: P1) 🎯 MVP

**Scenario**: As a student, I need to understand the course structure and navigate between chapters so that I can follow the learning path from fundamentals to capstone project.

**Why this priority**: Without clear navigation, students cannot effectively progress through the curriculum. This is the foundational user experience that enables all other learning activities.

**Independent Test**: Can be fully tested by navigating the Docusaurus site with sidebar navigation, breadcrumbs, and next/previous chapter links. Delivers immediate value by making content discoverable.

**Acceptance Scenarios**:

1. **Given** I am a new student viewing the textbook homepage, **When** I look at the sidebar, **Then** I see all 5 modules organized hierarchically with chapters grouped by week
2. **Given** I am reading Chapter 1.2 (ROS 2 Architecture), **When** I reach the bottom of the page, **Then** I see "Next: Chapter 1.3" and "Previous: Chapter 1.1" navigation links
3. **Given** I am on any chapter page, **When** I look at the breadcrumb trail, **Then** I see "Home > Module 1: ROS 2 > Chapter 1.2: Architecture" with clickable links
4. **Given** I am viewing Module 3 (NVIDIA Isaac), **When** I click on the module title in the sidebar, **Then** the chapter list expands/collapses to show all chapters in that module
5. **Given** I complete Module 2, **When** I look at the sidebar, **Then** Module 3 is highlighted as the next recommended section

---

### User Story 2 - Access Prerequisites and Learning Outcomes (Priority: P1) 🎯 MVP

**Scenario**: As a student, I need to verify I have the required prerequisites before starting a chapter and understand what I'll learn so that I can prepare appropriately and set learning goals.

**Why this priority**: Students need to self-assess readiness and understand learning objectives before investing time. This prevents frustration from missing foundational knowledge and enables goal-directed learning.

**Independent Test**: Can be tested by viewing any chapter frontmatter displaying prerequisites and learning outcomes. Delivers value by enabling informed learning decisions without requiring the full curriculum to be complete.

**Acceptance Scenarios**:

1. **Given** I am viewing Chapter 3.3 (Isaac ROS VSLAM), **When** I read the chapter header, **Then** I see a "Prerequisites" section listing "Module 1 (ROS 2 basics), Module 2 (Gazebo fundamentals), CUDA/GPU basics"
2. **Given** I lack ROS 2 knowledge, **When** I see the prerequisites section, **Then** each prerequisite is a hyperlink to the relevant chapter or external resource
3. **Given** I am starting Chapter 2.1 (Introduction to Robot Simulation), **When** I read the learning outcomes, **Then** I see 3-5 bullet points describing specific skills I'll gain (e.g., "Compare physics engines", "Identify sim-to-real gaps")
4. **Given** I completed a chapter, **When** I review the learning outcomes, **Then** I can self-assess whether I achieved each outcome with the practical exercises
5. **Given** I am planning my learning schedule, **When** I view a chapter, **Then** I see an "Estimated Time: 60-90 minutes" indicator

---

### User Story 3 - Run Code Examples Locally (Priority: P1) 🎯 MVP

**Scenario**: As a student, I need to download and execute code examples from the textbook so that I can practice ROS 2, Gazebo, and Isaac implementations hands-on.

**Why this priority**: Physical AI requires hands-on practice with real code. Students must be able to reproduce examples to build practical skills, not just read theory.

**Independent Test**: Can be tested by downloading a code example from static/code/, following the README, and successfully running it. Delivers immediate value even if only one chapter's code is available.

**Acceptance Scenarios**:

1. **Given** I am reading Chapter 1.4 (Building ROS 2 Packages), **When** I see a code block for a publisher node, **Then** I see a "Download: m1-c4-publisher.py" link below the code
2. **Given** I downloaded `m1-c4-publisher.py`, **When** I open the file, **Then** I see a docstring explaining the purpose, requirements.txt for dependencies, and inline comments for each section
3. **Given** I want to run the example, **When** I check the file header, **Then** I see installation instructions: "Requires: Ubuntu 22.04, ROS 2 Humble. Run: `ros2 run <package> <node>`"
4. **Given** I run the code example, **When** the script executes, **Then** it produces the expected output documented in the chapter (e.g., "Publishing IMU data at 100 Hz")
5. **Given** I encounter an error, **When** I check the code comments, **Then** I see troubleshooting tips for common issues (e.g., "If topic doesn't show, check QoS settings")

---

### User Story 4 - View Interactive Diagrams (Priority: P2)

**Scenario**: As a student, I need to see visual diagrams of ROS 2 graphs, robot architectures, and data flows so that I can understand system interactions beyond text descriptions.

**Why this priority**: Complex robotics systems are difficult to understand from text alone. Diagrams improve comprehension and retention, especially for visual learners.

**Independent Test**: Can be tested by viewing a chapter with an embedded Mermaid diagram that renders correctly in the browser. Delivers value by improving understanding of individual concepts without requiring all diagrams to be complete.

**Acceptance Scenarios**:

1. **Given** I am reading Chapter 1.2 (ROS 2 Architecture), **When** I scroll to the "Computational Graph" section, **Then** I see a Mermaid diagram showing nodes, topics, and message flow
2. **Given** I view the diagram, **When** I hover over a node, **Then** I see a tooltip with additional context (e.g., "Publisher Node: sensor_fusion")
3. **Given** I need to reference the diagram later, **When** I right-click the image, **Then** I can download the diagram as SVG or PNG
4. **Given** I am viewing on mobile, **When** I see a complex architecture diagram, **Then** the diagram scales responsively and remains readable
5. **Given** I am reading Chapter 5.2 (System Integration), **When** I view the capstone architecture diagram, **Then** I see color-coded components matching the module they were introduced in

---

### User Story 5 - Complete Practical Exercises with Validation (Priority: P2)

**Scenario**: As a student, I need step-by-step practical exercises with validation scripts so that I can practice skills and confirm I completed tasks correctly.

**Why this priority**: Exercises transform passive reading into active learning. Validation scripts provide immediate feedback, enabling students to self-assess without instructor intervention.

**Independent Test**: Can be tested by completing one exercise, running the validation script, and receiving pass/fail feedback. Delivers value incrementally as exercises are added per chapter.

**Acceptance Scenarios**:

1. **Given** I completed reading Chapter 1.5 (URDF), **When** I reach the "Practical Exercise" section, **Then** I see numbered steps: "1. Create humanoid_torso.urdf, 2. Define joints, 3. Validate with check_urdf"
2. **Given** I follow the exercise steps, **When** I finish, **Then** I see a "Validation" checklist with items like "[ ] URDF passes check_urdf", "[ ] Robot visualizes in RViz2"
3. **Given** I run the validation command `python validate_exercise_1-5.py`, **When** the script executes, **Then** I see output: "✓ URDF syntax valid, ✓ 3 joints detected, ✓ Kinematic chain complete - PASSED"
4. **Given** I made a mistake in my URDF, **When** I run validation, **Then** I see specific error messages: "✗ Joint 'shoulder_elbow' missing parent link 'shoulder'"
5. **Given** I completed the exercise, **When** I review the "Troubleshooting" section, **Then** I see solutions for common errors I might have encountered

---

### User Story 6 - Access Hardware Setup Guides (Priority: P2)

**Scenario**: As a student, I need detailed hardware setup instructions for Digital Twin Workstation, Physical AI Edge Kit, or cloud alternatives so that I can prepare my development environment.

**Why this priority**: Hardware setup is a major barrier to entry. Clear guides reduce frustration and enable students to start hands-on work quickly.

**Independent Test**: Can be tested by following one hardware setup guide (e.g., cloud GPU setup) and successfully running a test simulation. Delivers value independently of other content.

**Acceptance Scenarios**:

1. **Given** I am starting Module 2 (Gazebo), **When** I check the prerequisites, **Then** I see a link to "Hardware Setup Guide: Digital Twin Workstation"
2. **Given** I open the hardware guide, **When** I view the page, **Then** I see three tabs: "Local Workstation (RTX 3060+)", "Cloud GPU (AWS g5.xlarge)", "Budget Alternative (Simulation Only)"
3. **Given** I select "Cloud GPU", **When** I follow the steps, **Then** I see AWS-specific commands: "Launch g5.xlarge instance, Install CUDA 12.1, Setup ROS 2 Docker container"
4. **Given** I completed cloud setup, **When** I run the validation script, **Then** I see: "✓ CUDA detected (12.1), ✓ ROS 2 Humble installed, ✓ Gazebo Fortress running - READY"
5. **Given** I am on a budget, **When** I view the "Budget Alternative" tab, **Then** I see instructions for using Gazebo without GPU (software rendering) and performance expectations

---

### User Story 7 - Search and Discover Content (Priority: P3)

**Scenario**: As a student, I need to search for specific topics (e.g., "VSLAM", "URDF", "Isaac Sim") so that I can quickly find relevant chapters without manually browsing.

**Why this priority**: As the textbook grows, search becomes essential for retrieving information. This enhances the textbook's utility as a reference resource beyond linear reading.

**Independent Test**: Can be tested by entering a search query and verifying relevant chapters appear in results. Delivers value once search is configured, independent of content completeness.

**Acceptance Scenarios**:

1. **Given** I need to review VSLAM concepts, **When** I type "VSLAM" in the search bar, **Then** I see Chapter 3.3 (Isaac ROS VSLAM) as the top result
2. **Given** I search for "safety protocols", **When** results appear, **Then** I see chapters mentioning safety (e.g., Chapter 5.6: Manipulation with safety warnings)
3. **Given** I search for an acronym "QoS", **When** results display, **Then** the glossary entry for "Quality of Service" appears first, followed by chapters discussing QoS
4. **Given** I am viewing search results, **When** I click a result, **Then** the page scrolls to the section containing the search term highlighted
5. **Given** no results match my query, **When** I search for "nonexistent topic", **Then** I see "No results found" with suggestions for related terms from the glossary

---

### User Story 8 - Take Assessments and Review Answers (Priority: P3)

**Scenario**: As a student, I need to complete checkpoint quizzes and review explanations for correct/incorrect answers so that I can assess my understanding and identify knowledge gaps.

**Why this priority**: Assessments reinforce learning and provide feedback. While important, they are lower priority than core navigation and content access features.

**Independent Test**: Can be tested by completing one quiz, submitting answers, and viewing feedback. Delivers value per quiz independently of other assessments.

**Acceptance Scenarios**:

1. **Given** I finish reading Chapter 1.2 (ROS 2 Architecture), **When** I reach the "Assessment Checkpoint" section, **Then** I see an embedded quiz (Google Form or Quizlet) with 5 multiple-choice questions about DDS and QoS
2. **Given** I answer the questions, **When** I click "Submit" on the external quiz, **Then** I see my score (e.g., "4/5 correct - 80%") and which questions I missed (handled by quiz platform)
3. **Given** I answered incorrectly, **When** I review question 3, **Then** I see the correct answer with an explanation provided by the quiz platform: "QoS Reliability 'Best Effort' allows message loss for low-latency topics like sensor data"
4. **Given** I want to retake the quiz, **When** I click "Retry" on the quiz platform, **Then** the quiz resets (quiz platform handles shuffling if configured)
5. **Given** I completed all module quizzes, **When** I log into my quiz platform account (Google/Quizlet), **Then** I see a progress dashboard across all quizzes

---

### Edge Cases

**What happens when a student skips prerequisites?**
- Chapter pages display a warning banner: "⚠ Prerequisites not met. Review Module 1 before continuing."
- Validation scripts may fail with messages directing to prerequisite content

**How does the system handle broken external links (e.g., ROS 2 docs)?**
- Automated link checker runs weekly, reports broken links in GitHub Issues
- Backup links to archived documentation (Wayback Machine) provided in footnotes

**What happens if a student's hardware doesn't meet minimum specs?**
- Hardware guide includes a "Check Your System" script that outputs compatibility report
- Guide suggests cloud alternatives or budget-conscious simulation-only paths

**How does the system handle version mismatches (e.g., ROS 2 Iron instead of Humble)?**
- Each chapter specifies exact versions: "Tested on ROS 2 Humble (Ubuntu 22.04)"
- Troubleshooting sections include common version-related errors

**What happens if Docusaurus build fails due to malformed frontmatter?**
- Pre-commit hooks validate YAML frontmatter syntax before allowing commits
- CI/CD pipeline blocks PRs if build fails, with specific error messages

---

## Requirements *(mandatory)*

### Functional Requirements

**Navigation & Structure**

- **FR-001**: System MUST organize content into 5 modules with hierarchical chapter structure visible in sidebar navigation
- **FR-002**: System MUST display breadcrumb navigation on every page showing: Home > Module > Chapter
- **FR-003**: System MUST provide "Next Chapter" and "Previous Chapter" links at the bottom of every chapter page
- **FR-004**: System MUST support collapsible sidebar sections for each module to reduce visual clutter
- **FR-005**: System MUST highlight the currently active chapter in the sidebar with distinct styling
- **FR-006-NEW**: Homepage (intro.md) MUST display: Brief course introduction, 5 module cards showing (title, week range, hardware tier icon), hardware requirements summary, "Get Started" CTA button linking to Module 0 Chapter 1

**Content Delivery**

- **FR-007**: Every chapter MUST include frontmatter with: title, sidebar_label, sidebar_position, description, keywords, estimated_time
- **FR-008**: Every chapter MUST begin with a "Learning Objectives" section (3-5 bullet points) and "Prerequisites" section (with hyperlinks)
- **FR-009**: System MUST support Mermaid diagram rendering for flowcharts, sequence diagrams, and architecture visuals
- **FR-010**: Code blocks MUST specify language for syntax highlighting and include download links to complete files in `static/code/`
- **FR-011**: System MUST render LaTeX math equations for kinematics, dynamics, and control theory formulas

**Hardware & Setup**

- **FR-012**: System MUST provide a dedicated "Hardware Setup Guide" page with tabs for: Local Workstation, Cloud GPU, Budget Alternative
- **FR-013**: Hardware guide MUST include validation scripts that check CUDA, ROS 2, Gazebo, and Docker installations
- **FR-014**: Each module MUST specify required hardware tier: Proxy (simulation-only), Miniature ($1.5k-2k), or Premium ($90k+)
- **FR-015**: System MUST link each practical exercise to the appropriate hardware setup guide

**Code Examples & Exercises**

- **FR-016**: System MUST provide downloadable code examples organized by module and chapter in `static/code/module-X/`
- **FR-017**: Every code file MUST include: Docstring with purpose, inline comments, type hints, requirements.txt for dependencies
- **FR-018**: Practical exercises MUST include step-by-step instructions, validation checklists, and troubleshooting tips
- **FR-019**: System MUST provide validation scripts (Python) that students run to verify exercise completion

**Search & Discovery**

- **FR-020**: System MUST integrate Algolia DocSearch (free for open-source educational projects) with automatic crawling and ML-powered relevance ranking based on frontmatter keywords
- **FR-021**: Search results MUST prioritize glossary entries for acronyms, then chapters by relevance (configured in Algolia ranking rules)
- **FR-022**: System MUST provide a dedicated Glossary page with definitions for all technical terms (ROS 2, VSLAM, URDF, etc.)
- **FR-023**: Glossary terms MUST link back to chapters where they are introduced or used

**Assessments**

- **FR-024**: System MUST support assessment checkpoints embedded in chapters using external quiz platform integration (Quizlet, Google Forms, or similar) via iframes
- **FR-025**: Quizzes MUST provide immediate feedback with explanations for correct/incorrect answers (handled by external quiz platform)
- **FR-026**: Student progress tracking is handled by external quiz platform (Google account, Quizlet account, etc.)

**Instructor Resources**

- **FR-027**: System MUST provide an "Instructor Guide" section with: Lecture slides, solution keys for exercises, grading rubrics
- **FR-028**: Instructor guide MUST include suggested lab configurations and equipment lists for institutional setups

**Versioning & Updates**

- **FR-029**: System MUST use Docusaurus versioning to maintain multiple curriculum versions (e.g., v1.0, v1.1)
- **FR-030**: System MUST display a version dropdown in the navbar allowing students to view older content versions
- **FR-031**: System MUST include a changelog page documenting updates, new chapters, and corrected errors

**Accessibility & Internationalization**

- **FR-032**: All images MUST have descriptive alt text for screen readers (WCAG 2.1 Level AA compliance)
- **FR-033**: System MUST support Urdu translations toggle using Docusaurus i18n framework (optional per chapter)
- **FR-034**: System MUST ensure color contrast ratios meet 4.5:1 minimum for readability

---

### Key Entities *(data model)*

**Module**
- Represents a major curriculum unit (e.g., Module 1: ROS 2 Fundamentals)
- Attributes: module_id, title, week_range (e.g., "Weeks 3-5"), description, hardware_tier_required, chapters[]
- Relationships: Contains multiple Chapters

**Chapter**
- Represents a single lesson within a module (e.g., Chapter 1.2: ROS 2 Architecture)
- Attributes: chapter_id, title, sidebar_label, sidebar_position, estimated_time_minutes, prerequisites[], learning_outcomes[], keywords[], content_markdown
- Relationships: Belongs to Module, references Prerequisites (other Chapters), contains Parts

**Part**
- Represents a sub-section within a chapter (e.g., "DDS Middleware Concepts")
- Attributes: part_id, heading_level, title, content_markdown
- Relationships: Belongs to Chapter

**Hardware Configuration**
- Represents a development environment setup option
- Attributes: config_id, name (e.g., "Digital Twin Workstation"), tier (Proxy/Miniature/Premium), components[] (GPU, CPU, RAM, sensors), cost_usd, setup_guide_link
- Relationships: Referenced by Modules requiring specific hardware

**Code Example**
- Represents a downloadable code file
- Attributes: example_id, filename, language (Python/C++/YAML), module_id, chapter_id, description, file_path (static/code/...), requirements[]
- Relationships: Belongs to Chapter, requires Hardware Configuration

**Practical Exercise**
- Represents a hands-on activity for students
- Attributes: exercise_id, title, chapter_id, instructions_markdown, validation_script_path, estimated_time_minutes, difficulty (Beginner/Intermediate/Advanced)
- Relationships: Belongs to Chapter, validated by Validation Script

**Assessment Checkpoint**
- Represents a quiz or knowledge check (hosted on external platform)
- Attributes: assessment_id, chapter_id, platform_type (Quizlet/Google Forms/other), embed_url, passing_score_percentage
- Relationships: Belongs to Chapter
- Implementation: Embedded via iframe in chapter markdown

**Glossary Term**
- Represents a technical term definition
- Attributes: term_id, term, acronym, definition, context, related_terms[], urdu_translation
- Relationships: Referenced by Chapters

**Reference**
- Represents a citation to external documentation or research
- Attributes: reference_id, title, url, type (Official Docs/Research Paper/Tutorial), access_date
- Relationships: Referenced by Chapters

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Navigation & Usability**

- **SC-001**: 90% of students can locate a specific chapter (e.g., "Isaac ROS VSLAM") within 30 seconds using sidebar navigation or search
- **SC-002**: Breadcrumb navigation is present on 100% of chapter pages with functional links to parent pages
- **SC-003**: Zero broken internal links detected by automated link checker in CI/CD pipeline

**Content Quality**

- **SC-004**: 100% of chapters include frontmatter with all required fields (title, description, keywords, estimated_time, prerequisites, learning_outcomes)
- **SC-005**: 100% of code examples execute successfully on Ubuntu 22.04 + ROS 2 Humble without errors (verified by CI/CD tests)
- **SC-006**: All diagrams render correctly on desktop (1920x1080) and mobile (375x667) viewports

**Learning Effectiveness**

- **SC-007**: 80% of students achieve 70%+ on module assessments after completing corresponding chapters
- **SC-008**: Practical exercise validation scripts provide pass/fail feedback with specific error messages for 100% of exercises
- **SC-009**: Students report 4.0+ out of 5.0 average satisfaction with "clarity of learning objectives" in post-module surveys

**Hardware Accessibility**

- **SC-010**: Hardware setup guides exist for 3 tiers (Local/Cloud/Budget) with validation scripts that check system readiness
- **SC-011**: 100% of practical exercises include simulation fallback instructions for students without physical hardware
- **SC-012**: Cloud setup guide enables students to launch and validate environment within 30 minutes on AWS/GCP/Azure

**Search & Discovery**

- **SC-013**: Search functionality returns relevant results within top 3 results for 90% of common queries (e.g., "QoS", "URDF", "VSLAM")
- **SC-014**: Glossary includes definitions for 100% of acronyms and technical terms used across all modules
- **SC-015**: Glossary terms link to at least one chapter where the concept is explained in depth

**Build & Deployment**

- **SC-016**: Docusaurus build completes without warnings or errors in CI/CD pipeline for 100% of commits to main branch
- **SC-017**: GitHub Pages deployment completes within 5 minutes of merging PRs to main branch
- **SC-018**: Accessibility audit (axe DevTools) reports zero critical violations for WCAG 2.1 Level AA compliance

**Instructor Support**

- **SC-019**: Instructor guide includes solution keys for 100% of practical exercises with detailed explanations
- **SC-020**: Grading rubrics provided for all assessments with clear point allocations and criteria

---

## Module Hierarchy & Week Mapping

### Module 0: Getting Started (Weeks 1-2)

**Purpose**: Orient students, set up development environments, introduce Physical AI concepts

**Chapters**:
- **Chapter 0.1**: Welcome & Course Overview (Week 1)
  - What is Physical AI vs. Digital AI?
  - Course structure and learning path
  - How to use this textbook effectively
  - Hardware options and budget considerations

- **Chapter 0.2**: Development Environment Setup (Week 1)
  - Choosing your setup: Local, Cloud, or Hybrid
  - Installing Ubuntu 22.04 (dual boot, VM, or WSL2)
  - Installing ROS 2 Humble
  - Docker basics for reproducible environments
  - **Practical Exercise**: Validate environment setup

- **Chapter 0.3**: Introduction to Physical AI (Week 2)
  - The embodiment hypothesis
  - Sensorimotor grounding and world models
  - Industry landscape: Boston Dynamics, Tesla Optimus, Figure AI
  - Applications: Manufacturing, healthcare, space exploration
  - **Assessment Checkpoint**: Physical AI concepts quiz

**Hardware Requirements**: None (introductory content)

---

### Module 1: The Robotic Nervous System - ROS 2 (Weeks 3-5)

**Purpose**: Master ROS 2 fundamentals for building distributed robotic systems

**Chapters**:
- **Chapter 1.1**: ROS 2 Architecture and Core Concepts (Week 3)
  - DDS middleware and computational graph
  - Quality of Service (QoS) profiles
  - Namespaces and remapping
  - Lifecycle nodes
  - **Practical Exercise**: Visualize ROS graph with rqt_graph

- **Chapter 1.2**: Nodes, Topics, Services, and Actions (Week 3)
  - Publisher-Subscriber pattern
  - Request-Reply services
  - Goal-based actions
  - Parameter servers
  - **Practical Exercise**: Implement IMU publisher/subscriber

- **Chapter 1.3**: Building ROS 2 Packages with Python (Week 4)
  - Package structure (package.xml, setup.py)
  - Colcon build system
  - Workspace overlays
  - Dependency management with rosdep
  - **Practical Exercise**: Create multi-node sensor fusion package

- **Chapter 1.4**: URDF: Describing Robot Structure (Week 4)
  - URDF syntax: links, joints, kinematic chains
  - Xacro macros for reusable components
  - Collision vs. visual geometry
  - **Practical Exercise**: Create humanoid torso URDF

- **Chapter 1.5**: Launch Files and Parameter Management (Week 5)
  - Python-based launch files
  - Node composition
  - YAML parameter files
  - Conditional logic and event handlers
  - **Practical Exercise**: Launch multi-node perception system

- **Chapter 1.6**: ROS 2 Best Practices and Debugging (Week 5)
  - Logging with rclpy
  - Debugging with ros2 doctor
  - Performance profiling
  - **Assessment Checkpoint**: ROS 2 package development project

**Hardware Requirements**: Digital Twin Workstation (minimum: 16GB RAM, quad-core CPU)

---

### Module 2: The Digital Twin - Gazebo & Unity (Weeks 6-7)

**Purpose**: Create physics-accurate simulations for rapid prototyping and testing

**Chapters**:
- **Chapter 2.1**: Introduction to Robot Simulation (Week 6)
  - Why simulate? Benefits and limitations
  - Physics engines: ODE, Bullet, DART comparison
  - Sim-to-real gap and domain randomization
  - **Practical Exercise**: Compare simulation fidelity

- **Chapter 2.2**: Gazebo Environment Setup (Week 6)
  - Installing Gazebo Fortress/Harmonic
  - World files and model databases
  - Plugin architecture
  - ROS 2 - Gazebo bridge (ros_gz_bridge)
  - **Practical Exercise**: Load humanoid model into Gazebo

- **Chapter 2.3**: Physics Simulation: Gravity, Collisions, Forces (Week 6)
  - Contact dynamics and friction models
  - Joint controllers (position, velocity, effort)
  - Real-time factor tuning
  - **Practical Exercise**: Tune PID gains for bipedal standing

- **Chapter 2.4**: Sensor Simulation: LiDAR, Cameras, IMUs (Week 7)
  - Ray-based LiDAR plugins
  - RGB-D camera simulation
  - IMU bias and drift modeling
  - **Practical Exercise**: Fuse simulated IMU and odometry

- **Chapter 2.5**: URDF and SDF Robot Descriptions (Week 7)
  - SDF vs. URDF differences
  - Converting URDF to SDF
  - Gazebo-specific tags
  - **Practical Exercise**: Add Gazebo sensors to URDF

- **Chapter 2.6**: Unity for High-Fidelity Visualization (Week 7)
  - Unity Robotics Hub setup
  - ROS-TCP connector
  - Photorealistic rendering for perception training
  - **Practical Exercise**: Stream robot state to Unity
  - **Assessment Checkpoint**: Gazebo simulation implementation

**Hardware Requirements**: Digital Twin Workstation with GPU (minimum: RTX 3060, 12GB VRAM)

---

### Module 3: The AI-Robot Brain - NVIDIA Isaac (Weeks 8-10)

**Purpose**: Implement GPU-accelerated perception, navigation, and learning

**Chapters**:
- **Chapter 3.1**: NVIDIA Isaac Platform Overview (Week 8)
  - Isaac Sim, Isaac ROS, Isaac Gym ecosystem
  - Omniverse USD workflow
  - RTX-accelerated ray tracing
  - **Practical Exercise**: Launch Isaac Sim Hello World

- **Chapter 3.2**: Isaac Sim: Photorealistic Simulation (Week 8)
  - USD asset creation
  - MaterialX and RTX shaders
  - Domain randomization techniques
  - Synthetic data generation
  - **Practical Exercise**: Randomize lighting for data generation

- **Chapter 3.3**: Isaac ROS: Hardware-Accelerated VSLAM (Week 9)
  - NVIDIA NVSLAM pipeline
  - cuGraph for SLAM optimization
  - ROS 2 integration via Isaac ROS GEMs
  - **Practical Exercise**: Run NVSLAM on RealSense data

- **Chapter 3.4**: Nav2: Path Planning for Bipedal Movement (Week 9)
  - Nav2 stack architecture
  - Costmap generation
  - DWB and TEB local planners
  - Recovery behaviors
  - **Practical Exercise**: Configure Nav2 for humanoid

- **Chapter 3.5**: Reinforcement Learning for Robot Control (Week 10)
  - Isaac Gym massively parallel RL
  - PPO and SAC algorithms
  - Reward shaping for locomotion
  - **Practical Exercise**: Train quadruped walking policy

- **Chapter 3.6**: Sim-to-Real Transfer Techniques (Week 10)
  - Domain randomization strategies
  - System identification
  - Residual learning and online adaptation
  - **Practical Exercise**: Deploy sim-trained policy on Jetson
  - **Assessment Checkpoint**: Isaac perception pipeline project

**Hardware Requirements**: Digital Twin Workstation with RTX GPU (minimum: RTX 3060) OR Cloud GPU (AWS g5.xlarge)

---

### Module 4: Vision-Language-Action Models (Weeks 11-12)

**Purpose**: Enable natural language control and multimodal interaction

**Chapters**:
- **Chapter 4.1**: Convergence of LLMs and Robotics (Week 11)
  - From text LLMs to embodied VLA models
  - PaLM-E, RT-2, OpenVLA architectures
  - Challenges: Inference latency, safety, generalization
  - **Practical Exercise**: Run RT-2 inference on robot images

- **Chapter 4.2**: Voice-to-Action with OpenAI Whisper (Week 11)
  - Automatic Speech Recognition (ASR) with Whisper
  - Streaming vs. batch transcription
  - ROS 2 integration
  - **Practical Exercise**: Build voice command interface

- **Chapter 4.3**: Cognitive Planning with Language Models (Week 11)
  - Chain-of-thought prompting
  - LLM-based task planners (SayCan, Code as Policies)
  - Grounding LLM outputs in affordances
  - **Practical Exercise**: Use GPT-4 to generate ROS 2 actions

- **Chapter 4.4**: Integrating GPT Models for Conversational Robotics (Week 12)
  - OpenAI API integration with ROS 2
  - Conversation context management
  - Safety filtering and command validation
  - **Practical Exercise**: Build conversational robot interface

- **Chapter 4.5**: Multi-Modal Interaction: Speech, Gesture, Vision (Week 12)
  - Sensor fusion for multimodal understanding
  - Gesture recognition with MediaPipe
  - Attention mechanisms for HRI
  - **Practical Exercise**: Fuse speech + gesture commands
  - **Assessment Checkpoint**: VLA integration demo

**Hardware Requirements**: Physical AI Edge Kit (Jetson Orin Nano + RealSense + microphone) OR Cloud GPU

---

### Module 5: Capstone Project - Autonomous Humanoid (Weeks 13-14)

**Purpose**: Integrate all modules into a functioning autonomous system

**Chapters**:
- **Chapter 5.1**: Capstone Overview: The Autonomous Humanoid (Week 13)
  - System requirements and architecture
  - Perception → Planning → Control pipeline
  - Integration strategy
  - **Practical Exercise**: Define capstone milestones

- **Chapter 5.2**: System Integration and Testing (Week 13)
  - Master launch files for full stack
  - Debugging multi-node systems
  - Performance profiling
  - **Practical Exercise**: Profile end-to-end latency

- **Chapter 5.3**: Voice Command Processing Pipeline (Week 13)
  - ASR → NLU → Task Planning → Execution flow
  - Error handling and user feedback
  - **Practical Exercise**: Implement voice pipeline

- **Chapter 5.4**: Navigation and Obstacle Avoidance (Week 14)
  - Nav2 configuration for target environment
  - Dynamic obstacle detection
  - Real-time replanning
  - **Practical Exercise**: Navigate obstacle course

- **Chapter 5.5**: Computer Vision for Object Identification (Week 14)
  - YOLO or Isaac ROS DNN Inference
  - 6D pose estimation
  - Scene understanding
  - **Practical Exercise**: Identify target objects

- **Chapter 5.6**: Manipulation and Grasping (Week 14)
  - MoveIt 2 motion planning
  - Grasp pose generation
  - Force/torque feedback
  - **Practical Exercise**: Execute pick-and-place
  - **Final Assessment**: Complete autonomous humanoid demo

**Hardware Requirements**: Full Physical AI Edge Kit + Proxy Robot Lab (simulation) OR access to physical humanoid platform

---

## Docusaurus Project Structure

### Directory Layout

```
ai-and-humanoid-robotics-hackathon/
├── docs/                              # All content pages
│   ├── intro.md                       # Landing page
│   ├── module-0-getting-started/
│   │   ├── _category_.json            # Sidebar config for Module 0
│   │   ├── chapter-0-1.md             # Welcome & Course Overview
│   │   ├── chapter-0-2.md             # Environment Setup
│   │   └── chapter-0-3.md             # Introduction to Physical AI
│   ├── module-1-ros2/
│   │   ├── _category_.json
│   │   ├── chapter-1-1.md             # ROS 2 Architecture
│   │   ├── chapter-1-2.md             # Nodes, Topics, Services
│   │   ├── chapter-1-3.md             # Building Packages
│   │   ├── chapter-1-4.md             # URDF
│   │   ├── chapter-1-5.md             # Launch Files
│   │   └── chapter-1-6.md             # Best Practices
│   ├── module-2-digital-twin/
│   ├── module-3-isaac/
│   ├── module-4-vla/
│   ├── module-5-capstone/
│   ├── hardware-guide.md              # Hardware setup instructions
│   ├── glossary.md                    # Technical terms dictionary
│   ├── instructor-guide.md            # Teaching resources
│   └── changelog.md                   # Version history
│
├── static/                            # Static assets
│   ├── img/                           # Images and diagrams
│   │   ├── module-0/
│   │   ├── module-1/
│   │   ├── module-2/
│   │   ├── module-3/
│   │   ├── module-4/
│   │   └── module-5/
│   ├── code/                          # Downloadable code examples
│   │   ├── module-1/
│   │   │   ├── m1-c2-publisher.py
│   │   │   ├── m1-c3-package/
│   │   │   └── m1-c4-humanoid.urdf.xacro
│   │   ├── module-2/
│   │   ├── module-3/
│   │   ├── module-4/
│   │   └── module-5/
│   └── validation-scripts/            # Exercise validation scripts
│       ├── validate-1-4-urdf.py
│       ├── validate-2-2-gazebo.py
│       └── ...
│
├── src/                               # Custom React components
│   ├── components/
│   │   ├── InteractiveDiagram.js     # Mermaid wrapper
│   │   ├── CodeDownloadButton.js     # Download code files
│   │   └── HardwareRequirement.js    # Hardware tier badge
│   └── css/
│       └── custom.css                 # Theme customization
│
├── sidebars.js                        # Sidebar navigation config
├── docusaurus.config.js               # Main Docusaurus config
├── package.json                       # npm dependencies
├── .github/
│   └── workflows/
│       └── deploy.yml                 # GitHub Actions CI/CD
│
├── .specify/                          # Spec-Kit Plus templates
│   ├── memory/
│   │   └── constitution.md            # Project constitution
│   └── templates/
│       ├── spec-template.md
│       ├── plan-template.md
│       └── tasks-template.md
│
└── specs/                             # Feature specifications
    └── 001-book-master-plan/
        ├── spec.md                    # This document
        └── checklists/
            └── requirements.md
```

---

### Sidebar Configuration (`sidebars.js`)

```javascript
module.exports = {
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '🏠 Home',
    },
    {
      type: 'category',
      label: 'Module 0: Getting Started (Weeks 1-2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-0-getting-started/chapter-0-1',
        'module-0-getting-started/chapter-0-2',
        'module-0-getting-started/chapter-0-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals (Weeks 3-5)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-1-ros2/chapter-1-1',
        'module-1-ros2/chapter-1-2',
        'module-1-ros2/chapter-1-3',
        'module-1-ros2/chapter-1-4',
        'module-1-ros2/chapter-1-5',
        'module-1-ros2/chapter-1-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Weeks 6-7)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-2-digital-twin/chapter-2-1',
        'module-2-digital-twin/chapter-2-2',
        'module-2-digital-twin/chapter-2-3',
        'module-2-digital-twin/chapter-2-4',
        'module-2-digital-twin/chapter-2-5',
        'module-2-digital-twin/chapter-2-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac (Weeks 8-10)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-3-isaac/chapter-3-1',
        'module-3-isaac/chapter-3-2',
        'module-3-isaac/chapter-3-3',
        'module-3-isaac/chapter-3-4',
        'module-3-isaac/chapter-3-5',
        'module-3-isaac/chapter-3-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Models (Weeks 11-12)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-4-vla/chapter-4-1',
        'module-4-vla/chapter-4-2',
        'module-4-vla/chapter-4-3',
        'module-4-vla/chapter-4-4',
        'module-4-vla/chapter-4-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Project (Weeks 13-14)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-5-capstone/chapter-5-1',
        'module-5-capstone/chapter-5-2',
        'module-5-capstone/chapter-5-3',
        'module-5-capstone/chapter-5-4',
        'module-5-capstone/chapter-5-5',
        'module-5-capstone/chapter-5-6',
      ],
    },
    {
      type: 'category',
      label: '📚 Resources',
      collapsible: true,
      collapsed: true,
      items: [
        'hardware-guide',
        'glossary',
        'instructor-guide',
        'changelog',
      ],
    },
  ],
};
```

---

### Chapter Frontmatter Template

Every chapter MUST include this frontmatter structure:

```yaml
---
id: module-1-chapter-2
title: "Nodes, Topics, Services, and Actions"
sidebar_label: "1.2 Communication Patterns"
sidebar_position: 2
description: "Learn ROS 2 communication patterns: pub-sub topics, request-reply services, and goal-based actions for distributed robotics systems"
keywords: [ros2, topics, services, actions, pub-sub, request-reply, robotics]
estimated_time: 75
prerequisites:
  - module-1-chapter-1
  - basic-python
learning_outcomes:
  - Implement publisher-subscriber patterns for sensor data streaming
  - Create services for synchronous robot configuration requests
  - Build action servers for preemptible long-running tasks
  - Choose appropriate communication patterns for different robotics scenarios
hardware_tier: proxy
---
```

**Field Definitions**:
- `id`: Unique identifier (module-X-chapter-Y format)
- `title`: Full chapter title (displayed at top of page)
- `sidebar_label`: Shortened label for sidebar (format: "X.Y Short Title")
- `sidebar_position`: Numeric position within module (1, 2, 3...)
- `description`: SEO-optimized summary (150-160 characters)
- `keywords`: Array of searchable terms
- `estimated_time`: Minutes to complete chapter (reading + exercises)
- `prerequisites`: Array of prerequisite chapter IDs or concepts
- `learning_outcomes`: Array of 3-5 measurable learning objectives
- `hardware_tier`: Required setup (proxy/miniature/premium)

---

### Docusaurus Configuration (`docusaurus.config.js`)

Key settings for the textbook:

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  url: 'https://your-github-username.github.io',
  baseUrl: '/ai-and-humanoid-robotics-hackathon/',
  onBrokenLinks: 'throw',  // Fail build on broken links
  onBrokenMarkdownLinks: 'throw',

  themeConfig: {
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Textbook',
        },
        {
          to: '/hardware-guide',
          label: 'Hardware Setup',
          position: 'left',
        },
        {
          to: '/glossary',
          label: 'Glossary',
          position: 'left',
        },
        {
          type: 'docsVersionDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/your-repo',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Resources',
          items: [
            {label: 'Hardware Guide', to: '/hardware-guide'},
            {label: 'Instructor Guide', to: '/instructor-guide'},
            {label: 'Glossary', to: '/glossary'},
          ],
        },
        {
          title: 'Community',
          items: [
            {label: 'Discord', href: 'https://discord.gg/your-server'},
            {label: 'GitHub Discussions', href: 'https://github.com/your-repo/discussions'},
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Curriculum Team. Built with Docusaurus.`,
    },

    prism: {
      theme: require('prism-react-renderer/themes/github'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp'],
    },

    algolia: {  // Search integration (credentials provided after DocSearch application approval)
      appId: 'YOUR_APP_ID',  // Provided by Algolia DocSearch team
      apiKey: 'YOUR_SEARCH_API_KEY',  // Provided by Algolia DocSearch team
      indexName: 'physical-ai-textbook',  // Custom index name for this project
      contextualSearch: true,  // Enable contextual search for versioned docs
      searchParameters: {
        facetFilters: ['language:en', 'version:current'],  // Filter by language and version
      },
    },
  },

  markdown: {
    mermaid: true,  // Enable Mermaid diagrams
  },

  themes: ['@docusaurus/theme-mermaid'],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        sidebarPath: require.resolve('./sidebars.js'),
        editUrl: 'https://github.com/your-repo/edit/main/',
        showLastUpdateTime: true,
      },
    ],
  ],
};
```

---

## Hardware-to-Module Mapping

| Module | Hardware Tier | Components Required | Alternative Options |
|--------|---------------|---------------------|---------------------|
| **Module 0: Getting Started** | None | Standard laptop/desktop | N/A |
| **Module 1: ROS 2** | Proxy (Simulation) | 16GB RAM, Quad-core CPU | Cloud VM (AWS t3.xlarge - $0.17/hr) |
| **Module 2: Gazebo & Unity** | Digital Twin Workstation | RTX 3060 (12GB VRAM), 32GB RAM | Cloud GPU (AWS g5.xlarge - $1.50/hr) |
| **Module 3: NVIDIA Isaac** | Digital Twin Workstation | RTX 3060+, 32GB RAM, Ubuntu 22.04 | Cloud GPU (AWS g5.2xlarge - $3/hr) |
| **Module 4: VLA Models** | Physical AI Edge Kit | Jetson Orin Nano, RealSense D435i, USB mic | Cloud GPU (inference only - $1.50/hr) |
| **Module 5: Capstone** | Physical AI Edge Kit + Proxy Lab | Full edge kit + simulation env | Simulation-only (no physical robot) |

**Cost Summary**:
- **Budget Path** (Simulation Only): $0 hardware + $200-400/month cloud (optional)
- **Standard Path** (Local Workstation): $1,500-2,000 one-time + $931 edge kit
- **Premium Path** (Physical Robot): $90,000+ institutional lab setup

---

## AI/Spec-Driven Artifacts Per Chapter

Every chapter follows this spec-driven workflow:

### Phase 1: Chapter Specification (`specs/module-X-chapter-Y/spec.md`)
- User stories for learning activities
- Functional requirements (what students can do after reading)
- Success criteria (measurable learning outcomes)
- Prerequisites and dependencies

### Phase 2: Content Outline (`specs/module-X-chapter-Y/outline.md`)
- Section headings (H2, H3 structure)
- Estimated word count per section
- Placement of diagrams, code blocks, exercises

### Phase 3: Diagram Specifications (`specs/module-X-chapter-Y/diagrams/`)
- Mermaid source files for each diagram
- Annotations for tooltips and legends
- Alternate text descriptions for accessibility

### Phase 4: Code Examples (`static/code/module-X/`)
- Fully runnable Python/C++ files with inline comments
- requirements.txt or package.xml dependencies
- README with installation and execution instructions
- Validation scripts to verify student implementations

### Phase 5: Glossary Terms (`specs/module-X-chapter-Y/glossary-entries.md`)
- New terms introduced in this chapter
- Definitions, context, Urdu translations
- Links to related terms and chapters

### Phase 6: References (`specs/module-X-chapter-Y/references.md`)
- Citations to ROS 2 docs, research papers, tutorials
- Link validation and archive.org backups for external links

### Phase 7: Assessment Design (`specs/module-X-chapter-Y/assessment.md`)
- Quiz questions with answer explanations
- Practical exercise instructions with validation criteria
- Grading rubric for capstone project components

---

## Assessment & Capstone Task Mapping

### Formative Assessments (Throughout Course)

**Checkpoint Quizzes** (35% of grade):
- Module 0: Physical AI Concepts Quiz (5%)
- Module 1: ROS 2 Package Development (10%)
- Module 2: Gazebo Simulation Implementation (10%)
- Module 3: Isaac Perception Pipeline (10%)

**Practical Exercises** (15% of grade):
- 30+ hands-on exercises across all modules
- Validated by automated scripts (pass/fail)
- Graded on: Functionality (60%), Code Quality (20%), Documentation (20%)

### Summative Assessments

**Module Projects** (30% of grade):
- Module 1 Project: Multi-node ROS 2 perception system (10%)
- Module 2 Project: Custom Gazebo simulation with sensors (10%)
- Module 3 Project: GPU-accelerated VSLAM + object detection pipeline (10%)

**Final Capstone Project** (20% of grade):
- Week 13-14: Autonomous humanoid robot demonstration
- **Required Capabilities**:
  1. Voice command understanding (Whisper ASR + GPT-4 planning)
  2. Navigation with obstacle avoidance (Nav2 + Isaac ROS)
  3. Object detection and 6D pose estimation (YOLO/Isaac DNN)
  4. Manipulation and grasping (MoveIt 2)
  5. Full system integration with error recovery

**Capstone Grading Rubric**:
- System Architecture (20%): Clear separation of perception, planning, control
- Voice Command Processing (15%): Accurate ASR, natural language understanding, safe command validation
- Navigation (15%): Autonomous path planning, dynamic obstacle avoidance, goal reaching
- Perception (15%): Object detection accuracy, pose estimation, scene understanding
- Manipulation (15%): Successful grasping, motion planning, force control
- Integration & Robustness (10%): Multi-node coordination, error handling, recovery behaviors
- Documentation (10%): Code comments, system diagrams, demonstration video

---

## Scope Boundaries

### In Scope ✅

**Content**:
- 5 modules covering ROS 2, Gazebo, Unity, Isaac, VLA models
- 30+ chapters with learning objectives, prerequisites, exercises
- 100+ code examples (Python, C++, YAML, URDF)
- 50+ interactive diagrams (Mermaid, Draw.io)
- Comprehensive glossary (200+ terms)
- Hardware setup guides (3 tiers: Local, Cloud, Budget)
- Instructor resources (solution keys, grading rubrics, lecture outlines)

**Technical Platform**:
- Docusaurus 3.0+ for documentation site
- GitHub Pages deployment with CI/CD
- Mermaid for diagrams, Algolia for search
- Custom React components for interactive elements
- Version control with semantic versioning
- Accessibility compliance (WCAG 2.1 Level AA)

**Assessments**:
- Checkpoint quizzes (auto-graded where possible)
- Practical exercises with validation scripts
- Module projects (peer or instructor graded)
- Capstone project with detailed rubric

**Multilingual Support**:
- English (primary language)
- Urdu translations (toggle-enabled per chapter)
- Glossary terms with transliterations

---

### Out of Scope ❌

**Advanced Topics NOT Covered**:
- Low-level embedded programming (firmware, microcontrollers)
- Custom hardware design (PCB design, 3D printing robot parts)
- Advanced control theory (nonlinear control, optimal control)
- Cloud robotics platforms (AWS RoboMaker, Azure Kinect SDK)
- Multi-robot coordination and swarm robotics
- Safety certification processes (ISO 13482 compliance)

**Platform Limitations**:
- No live coding environment (students must set up locally or use cloud)
- No integrated video conferencing for live instruction
- No automated grading for capstone projects (requires human review)
- No student authentication/progress tracking (unless added as future feature)

**Hardware NOT Included**:
- Physical humanoid robots (students use simulation or bring their own)
- Specialized sensors beyond RealSense D435i and standard IMU
- High-end compute clusters for large-scale RL training

**Course Administration**:
- No learning management system (LMS) integration (Canvas, Moodle)
- No certificate issuance or accreditation
- No instructor-led live lectures (textbook is self-paced)

---

## Dependencies & External Integrations

**Required Software**:
- Ubuntu 22.04 LTS (host or VM)
- ROS 2 Humble Hawksbill
- Python 3.10+
- Gazebo Fortress/Harmonic
- Unity 2022.3 LTS + Unity Robotics Hub
- NVIDIA Isaac Sim 2023.1.1+
- Docker 24.0+
- Git 2.34+

**Optional Cloud Platforms**:
- AWS EC2 (g5 instance family for GPU workloads)
- Google Cloud Compute (T4, A100 GPUs)
- Azure NC-series VMs

**Third-Party APIs**:
- OpenAI API (Whisper, GPT-4) for VLA module
- Algolia DocSearch (free for open-source) - applied for via https://docsearch.algolia.com/apply/

**Docusaurus Plugins**:
- `@docusaurus/theme-mermaid` for diagrams
- `@docusaurus/plugin-content-docs` for versioning
- `docusaurus-plugin-sass` for custom styling
- `@docusaurus/plugin-google-gtag` for analytics (optional)

---

## Design & Layout Specifications

### Color Theme

**Light Mode**:
- Primary: #0066CC (blue - for links and accents)
- Secondary: #2E7D32 (green - for success states, checkmarks)
- Background: #FFFFFF (white)
- Sidebar: #F5F5F5 (light gray)
- Code Blocks: #F6F8FA (GitHub-style light gray)
- Text: #24292E (dark gray - high contrast)

**Dark Mode**:
- Primary: #58A6FF (lighter blue)
- Secondary: #56D364 (bright green)
- Background: #0D1117 (dark charcoal)
- Sidebar: #161B22 (slightly lighter charcoal)
- Code Blocks: #161B22 (matching sidebar)
- Text: #C9D1D9 (light gray)

### Typography

**Headings**:
- Font Family: "Inter", system-ui, sans-serif
- H1 (Chapter Title): 2.5rem (40px), font-weight: 700
- H2 (Major Section): 2rem (32px), font-weight: 600
- H3 (Subsection): 1.5rem (24px), font-weight: 600
- H4 (Minor Heading): 1.25rem (20px), font-weight: 500

**Body Text**:
- Font Family: "Inter", system-ui, sans-serif
- Size: 1rem (16px)
- Line Height: 1.6
- Paragraph Spacing: 1.5rem (24px)

**Code**:
- Font Family: "Fira Code", "Consolas", monospace
- Inline Code: 0.875rem (14px), background: #F6F8FA (light), #161B22 (dark)
- Code Blocks: 0.875rem (14px), line-height: 1.5

### Page Layout

**Desktop (>1024px)**:
- Sidebar Width: 300px (fixed)
- Content Max Width: 900px
- Margin: Auto-centered
- Table of Contents (right): 250px (sticky)

**Tablet (768px - 1024px)**:
- Sidebar: Collapsible hamburger menu
- Content Max Width: 100% (with padding)
- Table of Contents: Hidden (access via top button)

**Mobile (<768px)**:
- Sidebar: Slide-out menu
- Content: Full-width with 16px padding
- Font Size: Scale to 0.9rem for better fit

### Component Styling

**Code Block Features**:
- Line numbers (enabled for blocks >10 lines)
- Syntax highlighting (Prism.js with dracula theme for dark mode)
- Copy-to-clipboard button (top-right corner)
- Language label badge (top-left corner)
- Download button for full files (below block)

**Diagram Placement**:
- Centered within content column
- Max Width: 100% of content area
- Shadow: 0 2px 8px rgba(0,0,0,0.1)
- Margin: 2rem top and bottom
- Alt text always provided

**Callout Boxes** (Admonitions):
- **Info** (blue): For additional context or tips
- **Warning** (orange): For hardware safety alerts
- **Danger** (red): For critical safety warnings
- **Success** (green): For completion checkpoints
- Style: Left border (4px), icon, colored background

**Navigation Buttons** (Next/Previous):
- Position: Bottom of page, side-by-side
- Style: Outlined buttons with arrow icons
- Hover: Filled with primary color

---

## Next Steps After Specification Approval

1. **Create Docusaurus Project**: Initialize repo with `npx create-docusaurus@latest`
2. **Configure Sidebar**: Implement `sidebars.js` with 5 module hierarchy
3. **Design Custom Components**: Build React components for code downloads and hardware badges
4. **Apply for Algolia DocSearch**: Submit application at https://docsearch.algolia.com/apply/ (requires public GitHub repo and open-source license)
5. **Write Chapter 0.1**: Start with "Welcome & Course Overview" as first content
6. **Set Up CI/CD**: Configure GitHub Actions for automated build and deployment
7. **Create Hardware Guide**: Document 3 hardware setup tiers with validation scripts
8. **Build Glossary**: Compile initial 50 terms from constitution and Module 0
9. **Configure Algolia**: Once approved, add credentials to docusaurus.config.js and configure ranking rules for glossary priority
10. **Pilot Test**: Recruit 3-5 beta students to test navigation and Module 0 content

---

**Specification Complete** ✅

This document defines the complete structure, requirements, and success criteria for the Physical AI & Humanoid Robotics textbook. Ready for `/sp.plan` to design implementation strategy.
