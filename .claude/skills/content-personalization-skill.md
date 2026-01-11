# content-personalization-skill ⭐

## Description

Provides reusable logic for adapting educational content based on user background while preserving correctness.

## Components

### Skill-Level Adaptation
- Beginner-friendly explanations with analogies
- Intermediate content with technical depth
- Advanced content with research references
- Progressive complexity scaling
- Prerequisite knowledge checking
- Vocabulary and terminology adaptation

### Hardware-Aware Explanations
- Adapt examples based on available hardware (Arduino, Raspberry Pi, custom boards)
- Provide alternative implementations for different platforms
- Consider hardware constraints (memory, processing power, sensors)
- Reference specific components user has access to
- Offer simulations when physical hardware unavailable

### Depth and Verbosity Control
- Concise explanations for quick learners
- Detailed explanations with step-by-step breakdowns
- Visual vs text-heavy presentation preferences
- Example density (few vs many examples)
- Practice problem frequency and difficulty

## Responsibilities

### Adjust Explanation Depth Based on User Profile

**Depth Levels:**

**Level 1 - Beginner (Foundational):**
- Use everyday analogies and simple language
- Break concepts into small, digestible pieces
- Provide extensive examples and visual aids
- Avoid jargon; define all technical terms
- Focus on "what" and "why" before "how"
- Include hands-on activities and experiments

Example:
```
Topic: What is a sensor?

Beginner Explanation:
A sensor is like your eyes, ears, or nose for a robot. Just like you use
your eyes to see if a light is on, a robot uses a light sensor to detect
brightness. Think of it as giving your robot the ability to "feel" the
world around it.

Example: A temperature sensor is like putting your hand in water to check
if it's hot or cold. The sensor tells the robot the temperature as a number.
```

**Level 2 - Intermediate (Building Understanding):**
- Introduce technical terminology with context
- Explain underlying mechanisms
- Connect concepts to related topics
- Include code examples and implementations
- Discuss trade-offs and design decisions
- Reference standard practices

Example:
```
Topic: What is a sensor?

Intermediate Explanation:
A sensor is a transducer that converts physical phenomena (light,
temperature, pressure) into electrical signals that a microcontroller
can read. Sensors typically output analog voltages or digital signals.

Example: An ultrasonic distance sensor (HC-SR04) emits sound waves and
measures the time for the echo to return. Distance = (time × speed of sound) / 2.

Code:
```python
import RPi.GPIO as GPIO
import time

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, True)
time.sleep(0.00001)
GPIO.output(TRIG, False)

while GPIO.input(ECHO) == 0:
    pulse_start = time.time()
while GPIO.input(ECHO) == 1:
    pulse_end = time.time()

distance = (pulse_end - pulse_start) * 17150
print(f"Distance: {distance} cm")
```
```

**Level 3 - Advanced (Deep Dive):**
- Discuss advanced concepts, algorithms, and mathematics
- Reference research papers and academic sources
- Explore edge cases and optimizations
- Compare multiple implementation approaches
- Discuss real-world applications and limitations
- Include performance considerations

Example:
```
Topic: Sensor Fusion for Robotics

Advanced Explanation:
Sensor fusion combines data from multiple sensors using algorithms like
Kalman filters or complementary filters to produce more accurate state
estimates than any single sensor. This is crucial for robust localization
and navigation.

For IMU-based orientation estimation, a complementary filter combines:
- Accelerometer (stable long-term, noisy short-term)
- Gyroscope (accurate short-term, drifts over time)

Formula:
angle = α × (angle + gyro × dt) + (1 - α) × accel_angle

Where α ≈ 0.98 is the complementary filter coefficient.

Advanced implementations use Extended Kalman Filters (EKF) for non-linear
systems or particle filters for multi-modal distributions.

References:
- Thrun, S. et al. (2005). Probabilistic Robotics. MIT Press.
- Madgwick, S. (2010). AHRS algorithm for low-cost IMU sensor fusion.
```

**Adaptive Strategies:**

1. **Progressive Disclosure:**
   - Start with simplified explanation
   - Offer "learn more" expansions for deeper dives
   - Provide multiple levels in collapsible sections

2. **Prerequisite Checking:**
   ```python
   def check_prerequisites(user_profile, topic):
       required = get_prerequisites(topic)
       completed = user_profile.completed_topics

       missing = [req for req in required if req not in completed]

       if missing:
           return {
               "ready": False,
               "missing": missing,
               "suggestions": get_prerequisite_content(missing)
           }
       return {"ready": True}
   ```

3. **Dynamic Complexity Scaling:**
   - Monitor user comprehension through quiz performance
   - Adjust complexity based on engagement metrics
   - Offer remedial content if user struggles
   - Provide challenge problems for fast learners

### Preserve Factual Accuracy

**Accuracy Principles:**

**Never Sacrifice Correctness:**
- Simplification ≠ Inaccuracy
- Use analogies that preserve core concepts
- Mark approximations and simplifications explicitly
- Provide accurate information at all skill levels

**Good Simplification:**
```
"A for-loop repeats code a specific number of times."
(Beginner-friendly and accurate)
```

**Bad Simplification:**
```
"Variables store permanent values that never change."
(Inaccurate - confuses variables with constants)
```

**Accuracy Checks:**

1. **Fact Verification:**
   - Cross-reference technical details with authoritative sources
   - Validate code examples through testing
   - Review mathematical formulas for correctness
   - Cite sources for advanced claims

2. **Simplification Review:**
   - Ensure analogies don't create misconceptions
   - Test simplified explanations with learners
   - Provide "technically speaking" clarifications when needed
   - Mark edge cases that simplified explanation doesn't cover

3. **Code Example Validation:**
   - All code must be tested and functional
   - Include error handling for real-world robustness
   - Specify hardware/software versions and dependencies
   - Provide expected outputs and troubleshooting tips

**Example: Accurate Simplification**
```
Topic: How does a motor work?

Beginner (Simplified but Accurate):
"A DC motor spins when electricity flows through it. The direction of
spin depends on which way the electricity flows. We can control speed
by changing how much electricity we send."

Technical Note: "Technically, current through coils creates a magnetic
field that interacts with permanent magnets, producing rotational force
(torque). We control speed using Pulse Width Modulation (PWM)."

Advanced (Full Detail):
"DC motors operate on the principle of electromagnetic induction. When
current flows through the armature windings, it creates a magnetic field
that interacts with the stator's permanent magnetic field, producing
torque according to the Lorentz force law: F = I × L × B.

Speed control is achieved through PWM, where the effective voltage is
varied by rapidly switching the supply on and off. The duty cycle (% on-time)
determines average voltage and thus speed. Motor speed is proportional to
back-EMF: V = Ke × ω, where Ke is the motor constant and ω is angular velocity."
```

### Support Personalized Learning Flows

**Flow Customization:**

**1. Learning Path Adaptation:**
```python
def generate_learning_path(user_profile):
    """Generate personalized learning path based on user background"""

    # Extract user context
    skill_level = user_profile.skill_level
    goals = user_profile.learning_goals
    hardware = user_profile.available_hardware
    pace = user_profile.pace_preference

    # Start with foundations
    path = []

    if skill_level == "beginner":
        path.extend([
            "intro-to-robotics",
            "basic-programming-python",
            "understanding-sensors",
            "first-robot-project"
        ])
    elif skill_level == "intermediate":
        # Skip basics, focus on specific goals
        if "autonomous-navigation" in goals:
            path.extend([
                "sensor-fusion-basics",
                "pathfinding-algorithms",
                "pid-control",
                "navigation-project"
            ])

    # Adapt to hardware
    if "arduino" in hardware:
        path.append("arduino-robotics-guide")
    elif "raspberry-pi" in hardware:
        path.append("raspberry-pi-robotics")

    # Adjust pacing
    if pace == "self_paced":
        # Add optional deep-dives and projects
        path = add_optional_content(path)

    return path
```

**2. Content Format Adaptation:**
```python
def adapt_content_format(content, user_preferences):
    """Adapt content presentation to user preferences"""

    if user_preferences.learning_style == "visual":
        # Prioritize diagrams, videos, flowcharts
        content = enhance_with_visuals(content)

    elif user_preferences.learning_style == "kinesthetic":
        # Emphasize hands-on activities
        content = add_practice_exercises(content)

    elif user_preferences.learning_style == "reading":
        # Provide detailed text explanations
        content = expand_text_explanations(content)

    # Adjust verbosity
    if user_preferences.verbosity == "concise":
        content = summarize_key_points(content)
    elif user_preferences.verbosity == "detailed":
        content = add_detailed_explanations(content)

    return content
```

**3. Dynamic Difficulty Adjustment:**
```python
def adjust_difficulty(user_performance):
    """Dynamically adjust content difficulty based on performance"""

    recent_scores = user_performance.get_recent_quiz_scores(n=5)
    avg_score = sum(recent_scores) / len(recent_scores)

    if avg_score > 90:
        # User excelling - increase difficulty
        return {
            "action": "increase_difficulty",
            "recommendation": "Try advanced topics or challenge problems"
        }
    elif avg_score < 60:
        # User struggling - provide support
        return {
            "action": "provide_support",
            "recommendation": "Review fundamentals and try practice exercises"
        }
    else:
        # User progressing well
        return {
            "action": "maintain_pace",
            "recommendation": "Continue current difficulty level"
        }
```

**4. Hardware-Aware Personalization:**
```python
def personalize_for_hardware(content, user_hardware):
    """Adapt examples and projects to user's available hardware"""

    # Map generic concepts to user's specific hardware
    hardware_map = {
        "microcontroller": user_hardware.get("mcu", "Arduino Uno"),
        "distance_sensor": user_hardware.get("distance", "HC-SR04"),
        "motor_driver": user_hardware.get("motor", "L298N"),
        "camera": user_hardware.get("camera", "Raspberry Pi Camera")
    }

    # Replace generic examples with user-specific ones
    for generic, specific in hardware_map.items():
        content = content.replace(
            f"{{hardware.{generic}}}",
            specific
        )

    # Add hardware-specific tips
    if "raspberry-pi" in user_hardware:
        content += "\n\n**Raspberry Pi Tip:** " + get_rpi_tip(content.topic)

    return content
```

**5. Goal-Oriented Paths:**
```python
def create_goal_oriented_path(user_goal):
    """Create learning path focused on specific goal"""

    goal_paths = {
        "build_line_follower": [
            "sensors-for-line-detection",
            "motor-control-basics",
            "pid-controller-intro",
            "line-follower-project"
        ],
        "build_robotic_arm": [
            "servo-motors-explained",
            "inverse-kinematics-basics",
            "gripper-mechanisms",
            "robotic-arm-project"
        ],
        "learn_computer_vision": [
            "image-processing-basics",
            "opencv-introduction",
            "object-detection",
            "vision-robot-project"
        ]
    }

    return goal_paths.get(user_goal, default_learning_path())
```

## Usage Guidelines

When this skill is invoked, agents should:

1. **Always check user profile** before generating content
2. **Start at appropriate level** - don't assume prior knowledge
3. **Preserve accuracy** - never sacrifice correctness for simplicity
4. **Provide scaffolding** - help users bridge knowledge gaps
5. **Monitor comprehension** - adjust based on user signals
6. **Respect preferences** - honor user's chosen learning style and pace

## Best Practices

### Content Adaptation
- **Test simplifications** with real learners to avoid misconceptions
- **Provide multiple representations** (text, visual, code, hands-on)
- **Use progressive disclosure** - start simple, offer deeper dives
- **Include success criteria** so users know when they've mastered a topic

### Hardware Considerations
- **Default to common hardware** (Arduino Uno, Raspberry Pi 4) when user hardware unknown
- **Provide alternatives** - show multiple ways to achieve same result
- **Consider accessibility** - not all users have access to expensive hardware
- **Simulation options** - offer virtual alternatives (TinkerCAD, Webots, Gazebo)

### Accuracy and Safety
- **Review technical content** with subject matter experts
- **Test all code examples** on target hardware
- **Mark assumptions** - be explicit about simplifications
- **Provide corrections** - if a user has misconceptions, gently correct

### Engagement
- **Celebrate progress** - acknowledge when users advance in skill
- **Offer choices** - let users choose projects aligned with interests
- **Include real-world applications** - show why concepts matter
- **Balance challenge and support** - keep users in "zone of proximal development"

## Personalization Strategies

### 1. Adaptive Questioning
```python
# Adjust question difficulty based on user level
def generate_quiz(topic, user_level):
    if user_level == "beginner":
        return [
            {"type": "multiple_choice", "difficulty": "easy"},
            {"type": "fill_blank", "difficulty": "easy"}
        ]
    elif user_level == "intermediate":
        return [
            {"type": "multiple_choice", "difficulty": "medium"},
            {"type": "code_completion", "difficulty": "medium"}
        ]
    else:  # advanced
        return [
            {"type": "code_from_scratch", "difficulty": "hard"},
            {"type": "debug_challenge", "difficulty": "hard"}
        ]
```

### 2. Contextual Examples
```python
# Use examples relevant to user's interests
def select_example(concept, user_interests):
    if "space" in user_interests:
        return f"Example: {concept} is used in Mars rovers to..."
    elif "medical" in user_interests:
        return f"Example: {concept} is used in surgical robots to..."
    else:
        return f"Example: {concept} is commonly used to..."
```

### 3. Feedback Loops
```python
# Collect feedback to improve personalization
def collect_personalization_feedback(user_id, content_id):
    """Ask user if content was appropriate"""
    feedback = ask_user({
        "content_id": content_id,
        "questions": [
            "Was this explanation at the right level for you?",
            "Did you find the examples helpful?",
            "Would you like more or less detail next time?"
        ]
    })

    # Update user profile based on feedback
    update_preferences(user_id, feedback)
```

## Integration Patterns

### Middleware for Personalization
```python
@app.middleware("http")
async def personalization_middleware(request: Request, call_next):
    # Get user context
    user = await get_current_user(request)

    # Build personalization context
    context = {
        "skill_level": user.skill_level,
        "hardware": user.available_hardware,
        "preferences": user.learning_preferences,
        "progress": user.progress_data
    }

    # Attach to request
    request.state.personalization = context

    response = await call_next(request)
    return response

@app.get("/api/content/{topic}")
async def get_content(topic: str, request: Request):
    # Fetch base content
    content = await get_topic_content(topic)

    # Personalize based on user context
    personalized = personalize_content(
        content,
        request.state.personalization
    )

    return personalized
```

### RAG-Based Personalization
```python
async def get_personalized_answer(query: str, user_context: dict):
    """Retrieve and personalize content using RAG"""

    # Retrieve relevant content chunks
    chunks = await vector_db.search(
        query=query,
        filters={
            "skill_level": user_context["skill_level"]
        }
    )

    # Build prompt with user context
    prompt = f"""
    User Profile:
    - Skill Level: {user_context["skill_level"]}
    - Hardware: {user_context["hardware"]}
    - Learning Style: {user_context["preferences"]["learning_style"]}

    Question: {query}

    Context: {chunks}

    Provide an answer tailored to this user's level and preferences.
    Preserve technical accuracy while adapting explanation depth.
    """

    # Generate personalized response
    response = await llm.generate(prompt)

    return response
```

## Metrics and Evaluation

### Track Personalization Effectiveness
- **Comprehension rate**: Quiz scores after personalized content
- **Engagement**: Time spent, completion rates
- **User satisfaction**: Direct feedback and ratings
- **Learning velocity**: Time to master topics
- **Retention**: Long-term knowledge retention tests

### A/B Testing
- Test different personalization strategies
- Compare outcomes between personalized vs generic content
- Iterate based on data-driven insights
