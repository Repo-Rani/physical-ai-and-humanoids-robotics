<!--
Sync Impact Report:
- Version change: 2.0.0 → 3.0.0
- Modified principles: Complete overhaul - all 6 principles fully specified with concrete requirements
- Added sections: Detailed governance procedures, validation requirements, Docusaurus-specific guidelines
- Removed sections: Placeholder/template content, generic software development sections
- Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated (Constitution Check section aligned)
  - .specify/templates/spec-template.md ✅ updated (Requirements section references constitution)
  - .specify/templates/tasks-template.md ✅ updated (Verification checkpoints reference constitution)
- Follow-up TODOs: None - all placeholders resolved
-->

---
title: "Physical AI & Humanoid Robotics: From Simulation to Reality - Project Constitution"
author: "Physical AI Curriculum Team"
description: "Governance framework for an AI-native, spec-driven robotics textbook built with Docusaurus, covering ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action models"
version: "1.0.0"
target_audience: "Students with intermediate Python and AI knowledge"
prerequisites: "Python programming, basic AI/ML concepts, Linux familiarity"
course_duration: "13 weeks"
constitution_version: "3.0.0"
ratification_date: "2025-12-09"
last_amended: "2025-12-09"
---

# Physical AI & Humanoid Robotics Textbook - Project Constitution

## Document Metadata

**Project Name**: Physical AI & Humanoid Robotics: From Simulation to Reality
**Constitution Version**: 3.0.0
**Ratified**: 2025-12-09
**Last Amended**: 2025-12-09
**Methodology**: Spec-Kit Plus + Claude Code (AI-Native Spec-Driven Development)
**Output Platform**: Docusaurus + GitHub Pages
**Content Type**: Technical textbook with interactive diagrams, runnable code, simulations
**Target Audience**: Beginner to advanced robotics/AI students
**Prerequisites**: Python 3.10+, basic AI/ML concepts, Linux familiarity

---

## Project Overview

This constitution governs the development of a comprehensive AI-driven textbook covering Physical AI, embodied intelligence, and humanoid robotics systems. The textbook integrates theoretical foundations with practical implementation using industry-standard tools: ROS 2, Gazebo, Unity, NVIDIA Isaac platform, and Vision-Language-Action (VLA) models.

**Core Modules**:
1. The Robotic Nervous System (ROS 2) - Weeks 3-5
2. The Digital Twin (Gazebo & Unity) - Weeks 6-7
3. The AI-Robot Brain (NVIDIA Isaac) - Weeks 8-10
4. Vision-Language-Action (VLA) - Weeks 11-13
5. Capstone Project: Autonomous Humanoid - Weeks 13-14

**Key Themes**:
- Sim-to-real transfer methodology
- Hardware-accelerated perception and navigation
- Safety protocols for physical robot deployment
- High-performance computing for robotics AI
- Multilingual support (English/Urdu) for accessibility

---

## Core Principles (Non-Negotiable)

### Principle 1: Content Accuracy & Technical Rigor

**Rule**: All technical explanations, mathematics, code examples, simulations, and hardware specifications MUST be correct, verified against official documentation, peer-reviewed sources, and tested on specified target platforms.

**Implementation Requirements**:

1. **Code Verification**:
   - All ROS 2 code examples MUST be tested on Ubuntu 22.04 LTS + ROS 2 Humble
   - All Python code MUST run on Python 3.10+ and follow PEP 8 + ROS 2 Python style guide
   - All Docker containers MUST build successfully and reproduce documented environments
   - Code snippets MUST be complete and runnable, not fragments requiring guesswork

2. **Simulation Accuracy**:
   - Gazebo simulations MUST specify version (Gazebo Fortress/Harmonic) and physics engine
   - Unity simulations MUST document Unity version and ROS-TCP connector version
   - Isaac Sim examples MUST specify Isaac Sim version and GPU/RTX requirements
   - Physics parameters (timestep, gravity, friction) MUST be explicitly documented

3. **Hardware Specifications**:
   - All hardware requirements MUST include: Model numbers, minimum specs, estimated costs
   - GPU requirements MUST specify VRAM, CUDA compute capability, driver versions
   - Safety warnings MUST precede all instructions involving physical hardware, power, or motion
   - Cloud alternatives MUST be provided for expensive hardware (AWS/GCP/Azure instance types)

4. **Mathematical Rigor**:
   - Kinematics and dynamics equations MUST use consistent notation defined in glossary
   - Matrix operations MUST specify dimensions and coordinate frames
   - Control theory concepts (PID, state estimation) MUST include tuning guidance
   - URDF/SDF coordinate frames MUST follow ROS REP-103 conventions

5. **Citations & References**:
   - Technical claims MUST link to official documentation or peer-reviewed papers
   - Use IEEE citation format for academic papers
   - Link to specific documentation versions (e.g., ROS 2 Humble docs, not generic ROS 2 docs)
   - Industry standards (URDF, SDF, USD) MUST reference official specifications

6. **Safety Constraints**:
   - Physical robot deployment MUST include emergency stop procedures
   - Electrical safety warnings MUST precede hardware assembly instructions
   - Jetson power/thermal limits MUST be documented
   - Workspace boundaries MUST be defined for manipulation tasks

**Rationale**: Incorrect technical content is not just misleading—it can damage expensive hardware, cause physical injury, or fundamentally misrepresent robotics principles. Educational materials require higher accuracy standards than general documentation because students lack the experience to identify errors.

**Validation Process**:
- Automated CI/CD pipeline runs all code examples and simulations on every commit
- Manual peer review by subject matter experts (SMEs) with robotics expertise
- Student pilot testing with error reporting mechanisms
- Hardware compatibility testing on minimum spec devices (RTX 3060, Jetson Orin Nano)

---

### Principle 2: Educational Clarity & Accessibility

**Rule**: Content MUST progress from foundational concepts to advanced topics with explicit learning objectives, prerequisite checks, and multiple explanation modalities to accommodate diverse learning styles and backgrounds.

**Implementation Requirements**:

1. **Learning Structure**:
   - Each chapter MUST begin with: Learning objectives (3-5 bullet points), prerequisites (prior chapters/knowledge), estimated completion time
   - Concepts MUST be introduced before application (theory → example → exercise)
   - Cross-references MUST use relative links with context (e.g., "See Chapter 3.2: VSLAM for depth camera usage")
   - "Key Takeaways" sections MUST distill 2-3 main insights per chapter

2. **Multi-Modal Explanations**:
   - Complex concepts MUST include at least 3 modalities:
     - **Text**: Plain language explanation with technical terminology defined
     - **Visual**: Diagram, flowchart, or architecture diagram (Mermaid/Draw.io)
     - **Code**: Runnable example with inline comments
     - **Exercise**: Hands-on practice with validation criteria
   - Diagrams MUST have alt-text descriptions for accessibility (WCAG 2.1 Level AA)
   - Code examples MUST include: Purpose comment, input/output description, expected behavior

3. **Adaptive Content**:
   - `[PERSONALIZATION_POINT]` markers enable adaptive content paths:
     - Hardware-focused: Additional hardware debugging, oscilloscope usage, signal analysis
     - Software-focused: Algorithm deep-dives, optimization techniques, profiling
     - Industry professionals: Real-world deployment case studies, production considerations
   - `[URDU_TRANSLATION_TOGGLE]` enables Urdu translations for inclusive education
     - Technical terms provide: Urdu transliteration + (English term in parentheses)
     - Code comments remain in English (industry standard)

4. **Troubleshooting Support**:
   - Each chapter MUST include "Troubleshooting" section with common student errors:
     - **Issue**: Specific error message or symptom
     - **Cause**: Why this happens (conceptual explanation)
     - **Solution**: Step-by-step fix with verification command
   - Known simulator quirks MUST be documented (e.g., Gazebo contact instability at small timesteps)

5. **Assessment Integration**:
   - `[ASSESSMENT_CHECKPOINT]` markers indicate quiz/exercise insertion points
   - Checkpoints MUST be spaced every 15-20 minutes of reading (cognitive load management)
   - Questions MUST test understanding, not rote memorization (Bloom's taxonomy: Analyze/Apply level)

**Rationale**: Students arrive with diverse backgrounds—some strong in software but weak in hardware, others vice versa. Multiple explanation modalities ensure concepts are accessible regardless of learning style. Adaptive content prevents advanced students from being bored and beginners from being overwhelmed.

**Validation Process**:
- Pilot testing with 10+ students representing beginner/intermediate/advanced levels
- Feedback surveys after each module measuring: Clarity (1-5), Difficulty (1-5), Modality Preference
- Heatmap analysis of Docusaurus page analytics to identify drop-off points
- Accessibility audit using WAVE and axe DevTools

---

### Principle 3: Consistency & Standards

**Rule**: Terminology, notation, code style, file naming, and formatting MUST be uniform across all modules to reduce cognitive load and enable maintainability.

**Implementation Requirements**:

1. **Terminology Standards**:
   - Maintain project glossary at `docs/glossary.md` with canonical definitions
   - Use consistent technical terms:
     - **ROS 2** (not ROS2, ROS 2.0, or ros2)
     - **VSLAM** (not vSLAM, V-SLAM, or visual slam)
     - **Isaac Sim** (not IsaacSim or ISAAC Sim)
     - **Gazebo Fortress/Harmonic** (not Gazebo Ignition or Ignition Gazebo)
   - Acronyms MUST be defined on first use per chapter: "Visual-Language-Action (VLA)"
   - Avoid synonyms—choose one term and use consistently (e.g., "depth camera" not "RGB-D sensor" randomly)

2. **Mathematical Notation**:
   - Vectors: Bold lowercase (e.g., **v**, **ω**)
   - Matrices: Bold uppercase (e.g., **R**, **T**)
   - Coordinate frames: Subscript notation (e.g., p_world, p_camera)
   - Transforms: Superscript notation (e.g., T^world_camera)
   - Notation guide in glossary and Chapter 1.5 (URDF kinematics)

3. **Code Style Standards**:
   - **Python**: PEP 8 + ROS 2 Python Style Guide
     - 4 spaces (no tabs), max line length 100 characters
     - Type hints for function signatures: `def callback(msg: Imu) -> None:`
     - Docstrings in Google style format
   - **Launch Files**: Python-based launch files (not XML) for ROS 2
   - **URDF/SDF**: 2-space indentation, joint names: `{parent}_{child}_joint`
   - All code files MUST pass automated linting (ruff for Python, markdownlint for Markdown)

4. **File Naming Conventions**:
   - Chapters: `module-X-chapter-Y.md` (e.g., `module-1-chapter-2.md`)
   - Code examples: `{module}-{chapter}-{description}.py` (e.g., `m1-c2-publisher.py`)
   - Diagrams: `{module}-{chapter}-{description}.svg` (e.g., `m1-c2-ros-graph.svg`)
   - URDF files: `{robot_name}.urdf.xacro` (e.g., `humanoid_torso.urdf.xacro`)

5. **Markdown Formatting**:
   - Follow CommonMark specification
   - Headings: Sentence case (not Title Case)
   - Code blocks: Always specify language for syntax highlighting
   - Links: Descriptive text (not "click here")
   - Lists: Consistent punctuation (either all items end with periods or none do)

6. **Version Consistency**:
   - Document all version dependencies in chapter prerequisites:
     - Ubuntu 22.04 LTS (Jammy Jellyfish)
     - ROS 2 Humble Hawksbill
     - Python 3.10+
     - CUDA 12.1+ / cuDNN 8.9+
   - Lock file versions in Docker containers and requirements.txt

**Rationale**: Inconsistency creates unnecessary cognitive load—students waste mental energy deciphering whether "ROS 2" and "ROS2" refer to the same thing. Uniform standards enable:
- Automated tooling (linters, formatters, dead link checkers)
- Easier contributions from multiple authors
- Reduced confusion when cross-referencing chapters

**Validation Process**:
- Pre-commit hooks run markdownlint, ruff, and custom terminology checker
- CI/CD pipeline enforces style on every PR
- Quarterly consistency audits scan for terminology drift
- Glossary updated whenever new terms introduced

---

### Principle 4: Lab & Hardware Guidelines

**Rule**: Hardware requirements, cloud alternatives, budget tiers, and safety protocols MUST be explicitly documented for every hands-on activity. Simulation fallbacks MUST exist for all physical labs to ensure accessibility.

**Implementation Requirements**:

1. **Hardware Documentation**:
   - Each practical exercise MUST specify:
     - **Hardware Option**: Exact models with purchase links (e.g., "Intel RealSense D435i - $279")
     - **Cloud Alternative**: Specific instance types (e.g., "AWS EC2 g5.xlarge - ~$1.50/hr")
     - **Simulation Fallback**: How to complete exercise in Gazebo/Isaac Sim if no hardware
   - Budget tiers provided in constitution: Proxy ($0) / Miniature ($1,500-2,000) / Premium ($90,000+)

2. **Workstation Requirements**:
   - **Minimum Spec** (verified to work):
     - NVIDIA RTX 3060 (12GB VRAM), 32GB RAM, 500GB NVMe SSD
     - Ubuntu 22.04 LTS, CUDA 12.1+, Docker 24.0+
   - **Recommended Spec** (smooth experience):
     - NVIDIA RTX 4070 Ti (16GB VRAM), 64GB RAM, 1TB NVMe SSD
   - **Cloud Alternative** (cost-effective):
     - AWS EC2 g5.xlarge (1x A10G 24GB), $200-400/month if used 8 hours/day

3. **Physical AI Edge Kit** (for real hardware deployment):
   - NVIDIA Jetson Orin Nano (8GB) - $499
   - Intel RealSense D435i - $279
   - BNO085 9-DOF IMU - $29
   - USB array microphone (4-mic) - $49
   - 5V/4A USB-C PD power supply - $25
   - Total: $931 (optional, not required for course completion)

4. **Safety Protocols**:
   - All physical robot operations MUST include:
     - **Emergency Stop Procedure**: How to immediately halt motion (software + hardware kill switch)
     - **Workspace Boundaries**: Defined safe operating area with physical barriers
     - **Power Safety**: Voltage limits, short circuit protection, fuse ratings
     - **Motion Limits**: Joint angle constraints, velocity caps, collision detection
   - Hardware assembly instructions MUST precede electrical connections with:
     - ESD warnings for electronics handling
     - Polarity verification steps for power connections
     - Smoke test procedures (power on without load first)

5. **Jetson Deployment Constraints**:
   - Thermal throttling: Document max ambient temperature (35°C for passive cooling)
   - Power budgets: 15W mode vs 25W mode trade-offs
   - Model quantization requirements: FP16/INT8 for real-time inference
   - Storage limits: microSD card vs NVMe SSD performance implications

6. **Simulation-First Workflow**:
   - Every physical lab MUST be validated in simulation first:
     - Test URDF/SDF in Gazebo before hardware
     - Run perception pipelines on pre-recorded rosbags before live cameras
     - Validate navigation in simulation before deploying to physical robot
   - Simulation configs MUST match hardware specs (camera resolution, FOV, IMU update rate)

**Rationale**: Hardware access is the biggest barrier to robotics education. By providing cloud alternatives and simulation fallbacks, we ensure no student is excluded due to lack of equipment. Safety protocols are non-negotiable—robotics injuries are preventable with proper procedures.

**Validation Process**:
- All labs tested on minimum spec hardware configuration
- All cloud alternatives tested on specified instance types
- All simulation fallbacks verified to meet learning objectives
- Safety protocols reviewed by professional roboticists
- Hardware assembly instructions pilot-tested by beginners

---

### Principle 5: Docusaurus Structure & Quality

**Rule**: Content MUST be navigable, searchable, and maintainable with proper metadata, cross-references, and interactive elements. Build pipeline MUST be automated and error-free.

**Implementation Requirements**:

1. **File Structure**:
   ```
   docs/
   ├── intro.md                        # Landing page
   ├── module-1-ros2/
   │   ├── _category_.json             # Sidebar config
   │   ├── module-1-chapter-1.md
   │   ├── module-1-chapter-2.md
   │   └── ...
   ├── module-2-gazebo/
   ├── module-3-isaac/
   ├── module-4-vla/
   ├── module-5-capstone/
   ├── glossary.md
   └── hardware-guide.md

   static/
   ├── img/
   │   ├── module-1/                   # Diagrams organized by module
   │   └── module-2/
   └── code/
       ├── module-1/                   # Downloadable code examples
       └── module-2/
   ```

2. **Frontmatter Requirements**:
   - Every chapter MUST include:
     ```yaml
     ---
     id: module-1-chapter-2
     title: "ROS 2 Architecture and Core Concepts"
     sidebar_label: "1.2 ROS 2 Architecture"
     sidebar_position: 2
     description: "Learn ROS 2 DDS middleware, QoS profiles, and computational graph"
     keywords: [ros2, dds, qos, middleware, robotics]
     ---
     ```
   - `keywords` enable better search ranking
   - `sidebar_label` provides concise navigation text

3. **Cross-References**:
   - Use relative links with descriptive context:
     - ✅ Good: `[Learn about VSLAM in Chapter 3.3](../module-3-isaac/module-3-chapter-3.md)`
     - ❌ Bad: `[Chapter 3.3](../module-3-isaac/module-3-chapter-3.md)` (no context)
     - ❌ Bad: `Click [here](link)` (non-descriptive)
   - Internal links MUST be checked for broken references on every build

4. **Code Block Standards**:
   - Always specify language for syntax highlighting:
     ````markdown
     ```python
     import rclpy
     from rclpy.node import Node
     ```
     ````
   - Include download links for complete code examples in `static/code/`
   - Long code blocks (>50 lines) MUST be split with explanatory text between sections

5. **Interactive Diagrams**:
   - Use Mermaid for flowcharts, sequence diagrams, state machines:
     ````markdown
     ```mermaid
     graph LR
       A[Sensor] --> B[ROS 2 Topic]
       B --> C[Processing Node]
       C --> D[Actuator]
     ```
     ````
   - Use Draw.io embeds for complex architecture diagrams (export as SVG)
   - Use PlantUML for UML diagrams (class, component)
   - 3D visualizations: Link to Gazebo/Isaac Sim worlds or Unity WebGL builds

6. **Search Optimization**:
   - Keywords in frontmatter boost search relevance
   - Headings MUST contain searchable terms (not vague like "Overview")
   - First paragraph of chapter MUST summarize content (used in search previews)
   - Docusaurus Algolia integration configured for external search

7. **Versioned Documentation**:
   - Each textbook release tagged with semantic version (e.g., v1.0.0, v1.1.0)
   - Docusaurus versions feature enables accessing older content versions
   - Version dropdown in navbar for students to view curriculum updates

8. **Accessibility (WCAG 2.1 Level AA)**:
   - All images MUST have descriptive alt text
   - Color contrast ratios MUST meet 4.5:1 minimum (text on background)
   - Keyboard navigation MUST work for all interactive elements
   - Video captions MUST be provided for tutorial videos

**Rationale**: Docusaurus provides excellent documentation infrastructure, but only if conventions are followed. Proper metadata enables search, cross-references prevent broken links as content grows, and accessibility ensures compliance with educational standards.

**Validation Process**:
- `docusaurus build` MUST pass with zero warnings
- Dead link checker runs on every PR (broken-link-checker npm package)
- Accessibility audit with axe DevTools and WAVE
- Search relevance tested with common student queries
- Mobile responsiveness tested on 375px, 768px, 1024px viewports

---

### Principle 6: Code & Simulation Quality

**Rule**: All code examples and simulations MUST be reproducible, include error handling, follow ROS 2 best practices, and provide safety warnings for real deployments.

**Implementation Requirements**:

1. **Code Completeness**:
   - All code examples MUST include:
     - **Inline comments**: Explain non-obvious logic
     - **Docstrings**: Google-style format for classes/functions
     - **Type hints**: Python 3.10+ type annotations for all function signatures
     - **Entry point**: `if __name__ == '__main__':` guard for scripts
   - No code fragments—every example should be copy-paste runnable

2. **ROS 2 Package Standards**:
   - All ROS 2 packages MUST include:
     - `package.xml`: With correct dependencies and maintainer info
     - `setup.py`: With proper entry points for executables
     - `README.md`: Installation, usage, troubleshooting
     - `launch/`: Python-based launch files (not XML)
     - `config/`: YAML parameter files
     - `test/`: Integration tests if package is complex
   - Follow ROS 2 package naming: `{project}_{functionality}` (e.g., `humanoid_perception`)

3. **Simulation Specifications**:
   - Gazebo simulations MUST document:
     - Gazebo version (Fortress/Harmonic)
     - Physics engine (ODE/Bullet/DART) and timestep
     - Sensor noise parameters (e.g., Gaussian noise σ=0.01 for IMU)
     - Real-time factor target (e.g., RTF ≥ 1.0)
   - Isaac Sim simulations MUST document:
     - Isaac Sim version and Omniverse launcher version
     - RTX settings (ray tracing samples, path tracing enabled/disabled)
     - Domain randomization parameters if used
     - USD file versions and dependencies

4. **Error Handling**:
   - ROS 2 nodes MUST include:
     - Parameter validation on startup (raise ValueError if invalid)
     - Graceful shutdown on SIGINT (rclpy.shutdown() in exception handler)
     - QoS mismatch warnings (detect and log if publisher/subscriber QoS incompatible)
     - Timeout handling for service calls (rclpy.spin_until_future_complete with timeout)
   - Simulation scripts MUST check:
     - Model file existence before loading
     - Plugin library availability (LD_LIBRARY_PATH checks)
     - GPU availability for Isaac Sim (fail early with clear error message)

5. **Safety for Physical Robots**:
   - Real robot code MUST include:
     - **Emergency stop**: Subscribes to `/emergency_stop` topic, halts all motion immediately
     - **Watchdog timers**: Detect command timeout (e.g., no velocity command for 1s → stop)
     - **Joint limit enforcement**: Software checks before sending commands to hardware
     - **Collision detection**: Monitor force/torque sensors, stop on unexpected contact
     - **Rate limiting**: Cap velocity/acceleration to safe values (document max in comments)
   - Hardware deployment code MUST have a "SAFETY WARNING" comment block:
     ```python
     # SAFETY WARNING: This code controls physical hardware.
     # - Ensure workspace is clear of people and obstacles before running.
     # - Emergency stop button must be within arm's reach.
     # - Verify joint limits and velocity caps before deployment.
     # - Test in simulation FIRST before running on hardware.
     ```

6. **Docker Containers for Reproducibility**:
   - Provide Dockerfiles for complex environments:
     - Base image: `osrf/ros:humble-desktop-full` for ROS 2 projects
     - Base image: `nvcr.io/nvidia/isaac-sim:2023.1.1` for Isaac Sim projects
   - Dockerfile MUST include:
     - All apt dependencies with specific versions
     - Python packages from pinned requirements.txt
     - Environment variables (ROS_DOMAIN_ID, GAZEBO_MODEL_PATH)
   - Docker Compose for multi-container stacks (e.g., Gazebo + ROS bridge + web UI)

7. **Testing Requirements**:
   - **Contract tests**: Verify ROS 2 message schemas and service definitions
   - **Integration tests**: Test multi-node systems with `launch_testing`
   - **Simulation tests**: Automated tests that launch Gazebo, spawn robot, verify behavior
   - Tests MUST be runnable in CI/CD pipeline (GitHub Actions or GitLab CI)

**Rationale**: Non-reproducible code frustrates students and wastes time debugging environment issues rather than learning concepts. Unsafe code can damage expensive hardware or cause physical injuries. Error handling teaches professional software engineering practices that students will use in industry.

**Validation Process**:
- CI/CD pipeline runs all code examples in Docker containers on every commit
- Hardware deployment code reviewed by robotics safety experts
- Simulation tests run in headless mode (OSMesa/EGL for rendering without display)
- Code quality checked with ruff, mypy (type checker), and custom linters
- Manual code review checklist for safety-critical sections

---

## Governance & Amendment Process

### Constitution Authority

This constitution is the **supreme governance document** for the Physical AI & Humanoid Robotics Textbook project. All project practices, style guides, and workflows are subordinate to this constitution. In case of any conflict:

1. Constitution principles take precedence
2. Spec-Kit Plus methodology applies (AI-native spec-driven development)
3. Claude Code acts as implementation assistant following this constitution
4. Human maintainers have final authority on ambiguous cases

### Amendment Procedure

To amend this constitution, the following process MUST be followed:

1. **Proposal Phase** (1-2 weeks):
   - Document proposed change in GitHub Issue with label `constitution-amendment`
   - Include: Rationale, affected principles, implementation impact, migration plan
   - Tag curriculum lead and relevant subject matter experts (SMEs)

2. **Review Phase** (1-2 weeks):
   - Technical review by at least 2 SMEs with domain expertise
   - Impact analysis: Which templates, chapters, and code examples require updates?
   - Public comment period for community feedback (if open-source)

3. **Approval Phase** (1 week):
   - Requires approval from:
     - Curriculum lead (final authority)
     - At least 2 SMEs in affected domains (e.g., ROS 2 expert + Isaac expert)
   - Approval documented in GitHub Issue with explicit "Approved" comments

4. **Versioning Decision**:
   - **MAJOR version bump** (X.0.0): Backward-incompatible changes
     - Removing or redefining principles
     - Changing governance procedures
     - Deprecating entire modules or platforms
   - **MINOR version bump** (0.X.0): Backward-compatible additions
     - Adding new principles or sections
     - Expanding guidance within existing principles
     - Adding new platforms/tools to curriculum
   - **PATCH version bump** (0.0.X): Clarifications only
     - Typo fixes, wording improvements
     - Adding examples to existing rules
     - Reformatting without semantic changes

5. **Migration Phase** (1-4 weeks, depends on scope):
   - Update `.specify/memory/constitution.md` with new version and content
   - Propagate changes to dependent templates:
     - `.specify/templates/plan-template.md` (Constitution Check section)
     - `.specify/templates/spec-template.md` (Requirements alignment)
     - `.specify/templates/tasks-template.md` (Verification checkpoints)
   - Update affected chapters and code examples
   - Run full CI/CD pipeline to verify no breakage
   - Create migration guide for students/contributors if major changes

6. **Communication Phase** (1 week):
   - Announce changes in project README and Docusaurus changelog
   - If major version: Publish migration guide and record video walkthrough
   - Update constitution version in all chapter frontmatter if needed
   - Tag GitHub release with constitution version (e.g., `constitution-v3.0.0`)

### Compliance Review

Ongoing compliance is ensured through three mechanisms:

1. **Pre-Merge Review** (automated + manual):
   - All pull requests MUST pass constitution compliance checklist:
     - [ ] Code examples are tested and runnable
     - [ ] Safety warnings present for hardware operations
     - [ ] Citations link to official documentation
     - [ ] Cross-references use relative links and are not broken
     - [ ] Frontmatter metadata complete
     - [ ] Terminology consistent with glossary
     - [ ] Simulation specs documented
   - GitHub Actions workflow enforces automated checks (linting, dead links, build)
   - Human reviewers verify safety and educational clarity

2. **Quarterly Audits** (every 3 months):
   - Random sample audit: Select 10% of chapters and code examples
   - Verify adherence to all 6 principles
   - Check for terminology drift or outdated version references
   - Identify common violations and update tooling to prevent
   - Results documented in `docs/audits/YYYY-QX-audit.md`

3. **Incident-Driven Review** (as needed):
   - Any safety incident, hardware damage, or critical inaccuracy triggers immediate review
   - Incident reported in GitHub Issue with label `constitution-incident`
   - Root cause analysis: Which principle was violated? Why did it pass review?
   - Update constitution or tooling to prevent recurrence
   - Document in `docs/incidents/YYYY-MM-DD-incident.md`

### Conflict Resolution

If contributors disagree on constitutional interpretation:

1. **First**: Consult constitution rationale (each principle explains "why")
2. **Second**: Seek curriculum lead clarification (authoritative interpretation)
3. **Third**: If ambiguous, propose clarifying amendment through standard process

### Continuous Improvement

This constitution is a living document. Every 6 months:

- Survey students for pain points and unclear guidance
- Analyze common PR rejections to identify frequent violations
- Review new robotics standards or platform updates (e.g., ROS 2 new releases)
- Consider amendments to incorporate lessons learned

---

## Version History

| Version | Date | Changes | Rationale |
|---------|------|---------|-----------|
| **3.0.0** | 2025-12-09 | Complete specification of all 6 principles with concrete requirements, validation processes, and governance procedures. Removed all placeholder content. Added Docusaurus-specific guidelines, safety protocols, and accessibility requirements. | Major version bump: Transformed from template structure to fully enforceable constitution. All principles now have measurable compliance criteria. |
| **2.0.0** | 2025-12-08 | Complete restructure for Physical AI & Humanoid Robotics textbook curriculum. Added module/chapter structure, hardware requirements, learning outcomes, and assessment strategy. Shifted from generic software development to educational robotics focus. | Major version bump: Fundamental shift from software project to educational textbook governance. |
| **1.1.0** | 2025-12-08 | Initial Physical AI & Humanoid Robotics principles drafted. Defined course structure, prerequisites, and hardware tiers. | Minor version bump: Added new principles specific to robotics curriculum. |
| **1.0.0** | 2025-12-07 | Foundation constitution created with basic governance structure. | Initial ratification. |

---

## Appendix A: Textbook Metadata

**Full Title**: Physical AI & Humanoid Robotics: From Simulation to Reality
**Subtitle**: A Hands-On Guide to Embodied AI with ROS 2, Gazebo, Unity, and NVIDIA Isaac
**Target Audience**: Undergraduate/graduate students in CS, robotics, AI; industry professionals upskilling
**Prerequisites**: Python 3.10+, basic machine learning, Linux command line, linear algebra
**Course Duration**: 13 weeks (65 hours estimated total time)
**Assessment**: Formative quizzes (35%), projects (50%), capstone (20%)
**Platform**: Docusaurus 3.0+, deployed via GitHub Pages
**License**: TBD (recommended: CC BY-NC-SA 4.0 for content, MIT for code)
**Multilingual**: English (primary), Urdu (toggle-enabled)

---

## Appendix B: Hardware Reference

### Digital Twin Workstation (Minimum Spec)

- **GPU**: NVIDIA RTX 3060 (12GB VRAM), CUDA 12.1+
- **CPU**: Intel i7-12700 or AMD Ryzen 7 5800X (8+ cores)
- **RAM**: 32GB DDR4
- **Storage**: 500GB NVMe SSD
- **OS**: Ubuntu 22.04 LTS
- **Cost**: ~$1,500-2,000
- **Cloud Alternative**: AWS EC2 g5.xlarge (~$1.50/hr)

### Physical AI Edge Kit (Optional, for Real Hardware)

- **Compute**: NVIDIA Jetson Orin Nano 8GB - $499
- **Depth Camera**: Intel RealSense D435i - $279
- **IMU**: BNO085 9-DOF - $29
- **Microphone**: USB 4-mic array - $49
- **Power**: 5V/4A USB-C PD supply - $25
- **Accessories**: Cables, mounts, microSD 128GB - $50
- **Total**: ~$931

### Robot Lab Options

- **Proxy** (Simulation only): $0 (Gazebo/Isaac Sim)
- **Miniature** (TurtleBot 4 + arm): $1,500-2,000
- **Premium** (Unitree H1 or similar): $90,000+ (institutional labs)

---

## Appendix C: Software Versions

**Locked Versions** (as of 2025-12-09):

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Python 3.10+ (tested on 3.10.12)
- Gazebo Fortress (preferred) or Harmonic
- Unity 2022.3 LTS + Unity Robotics Hub 0.7.0+
- NVIDIA Isaac Sim 2023.1.1+
- CUDA 12.1+ / cuDNN 8.9+
- Docker 24.0+
- Docusaurus 3.0+

**Update Policy**: Software versions updated annually to align with ROS 2 LTS releases and NVIDIA platform updates. When versions change, migration guides provided.

---

## Appendix D: Glossary Integration

The project maintains a comprehensive glossary at `docs/glossary.md` with canonical definitions for all technical terms. When introducing new terminology:

1. Add definition to glossary with: Term, definition, acronym (if applicable), context, related terms
2. Link first usage in each chapter to glossary entry
3. Use consistent term throughout all content (no synonyms)
4. For Urdu translations, provide transliteration + (English term)

Example glossary entry:

```markdown
### VSLAM (Visual Simultaneous Localization and Mapping)

**Definition**: A technique for constructing a map of an unknown environment while simultaneously tracking the robot's location within it, using only visual input from cameras.

**Acronym**: VSLAM (pronounced "vee-slam")

**Context**: Used extensively in Chapter 3.3 for navigation. NVIDIA NVSLAM provides GPU-accelerated implementation.

**Related Terms**: SLAM, Odometry, Pose Estimation, Depth Camera

**Urdu**: وِژُوَل سائمَلٹینیَس لوکَلائزیشن اینڈ میپِنگ (VSLAM)
```

---

**End of Constitution**

---

**Summary**: This constitution defines the governance framework for an AI-native, spec-driven Physical AI textbook. It enforces 6 core principles (Accuracy, Clarity, Consistency, Hardware, Docusaurus, Code Quality) with concrete validation processes. All content, code, and simulations MUST comply. Amendments follow a formal proposal-review-approval process with semantic versioning. Compliance is ensured through pre-merge checks, quarterly audits, and incident-driven reviews.

For questions or clarifications on constitutional interpretation, consult the curriculum lead or propose an amendment via GitHub Issue.