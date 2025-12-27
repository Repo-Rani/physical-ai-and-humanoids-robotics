---
id: module-4-chapter-2
title: "OpenAI Integration for Cognitive Planning"
sidebar_label: "4.2 OpenAI Integration"
sidebar_position: 2
description: "Integrate OpenAI GPT models with ROS 2: natural language task planning, code generation for robot control, and conversational interfaces for humanoids"
keywords: [openai, gpt, cognitive-planning, natural-language, task-planning, code-generation]
estimated_time: 90
prerequisites:
  - module-4-chapter-1
  - module-1-chapter-3
learning_outcomes:
  - Integrate OpenAI API with ROS 2 systems
  - Design prompts for robot task planning
  - Generate executable robot code from natural language
  - Implement safety constraints for LLM-generated actions
  - Build conversational interfaces for robot control
hardware_tier: miniature
---

# باب 4.2: OpenAI انضمام برائے معرفی منصوبہ بندی

بڑے زبان کے ماڈلز (LLMs) جیسے GPT-4 روبوٹس کو فطری زبان کے احکامات سمجھنے اور پیچیدہ کاموں کے منصوبے بنانے کی صلاحیت دیتے ہیں۔ یہ باب دکھاتا ہے کہ روبوٹ کے معرفی کنٹرول کے لیے OpenAI کے API کو کیسے ضم کیا جائے۔

## روبوٹکس کے لیے LLMs کیوں؟

بڑے زبان کے ماڈلز روبوٹکس میں فطری زبان کی سمجھ اور استدلال کی غیر معمولی صلاحیتیں لاتے ہیں:

**صلاحیتیں:**
- **فطری انٹرفیس:** روزمرہ کی زبان میں روبوٹس کو کنٹرول کریں
- **کام کی تقسیم:** پیچیدہ ہدفوں کو قابل عمل اقدامات میں توڑیں
- **کوڈ جنریشن:** ہدایات کو روبوٹ پروگراموں میں ترجمہ کریں
- **استدلال:** سیاق و سباق، پابندیاں، اور پیش شرطیں سمجھیں
- **مطابقت:** پروگرامنگ کے بغیر تغیرات کو سنبھالیں

**پابندیاں:**
- **توہمات:** غلط منصوبے پیدا کر سکتے ہیں
- **کوئی جسمانی بنیاد نہیں:** حقیقی دنیا کی طبیعیات کو نہیں سمجھتا
- **تاخیر:** API کالز تاخیر کا باعث بنتی ہیں (100-2000ms)
- **لاگت:** ٹوکن کا استعمال بڑے پیمانے پر مہنگا ہو سکتا ہے

## روبوٹکس کے لیے OpenAI API سیٹ اپ

### API کلید کی تشکیل

```bash
# Install OpenAI Python library
pip install openai

# Set API key (do NOT hardcode in code!)
export OPENAI_API_KEY="sk-proj-..."

# Verify installation
python -c "import openai; print(openai.__version__)"
```

### ROS 2 انضمام کی ساخت

OpenAI انضمام کے لیے ROS 2 پیکیج بنائیں:

```bash
cd ~/ros2_ws/src
ros2 pkg create cognitive_planning \
  --build-type ament_python \
  --dependencies rclpy std_msgs geometry_msgs

cd cognitive_planning
mkdir cognitive_planning
```

### محفوظ API کلید کا انتظام

```python
# cognitive_planning/config/secrets.yaml (add to .gitignore!)
openai:
  api_key: "sk-proj-YOUR_KEY_HERE"
  organization: "org-YOUR_ORG"  # Optional
  model: "gpt-4-turbo-preview"
  max_tokens: 2000
  temperature: 0.1  # Low temperature for deterministic planning
```

**پائیتھون تشکیل لوڈر:**
```python
import os
import yaml
from pathlib import Path

class OpenAIConfig:
    def __init__(self):
        # Try environment variable first
        self.api_key = os.getenv('OPENAI_API_KEY')

        # Fall back to config file
        if not self.api_key:
            config_path = Path(__file__).parent / 'config' / 'secrets.yaml'
            if config_path.exists():
                with open(config_path) as f:
                    config = yaml.safe_load(f)
                    self.api_key = config['openai']['api_key']
                    self.model = config['openai'].get('model', 'gpt-4-turbo-preview')
            else:
                raise ValueError("No OpenAI API key found!")

        self.validate_key()

    def validate_key(self):
        if not self.api_key or not self.api_key.startswith('sk-'):
            raise ValueError("Invalid OpenAI API key format")
```

## کام کی منصوبہ بندی کے لیے پرومپٹ انجینئرنگ

### سسٹم پرومپٹ ڈیزائن

```python
ROBOTICS_SYSTEM_PROMPT = """You are an AI assistant integrated with a humanoid robot.
Your role is to help plan and execute tasks by generating step-by-step plans.

**Robot Capabilities:**
- Navigate to locations (move_to_location)
- Pick up objects (pick_object)
- Place objects (place_object)
- Open/close drawers (manipulate_drawer)
- Recognize objects via camera (detect_objects)

**Robot Limitations:**
- Cannot lift objects heavier than 5kg
- Maximum reach: 0.8m from base
- Requires clear path for navigation
- Cannot manipulate fragile items

**Output Format:**
Generate plans as Python lists of dictionaries:
[
  {"action": "move_to_location", "target": "kitchen"},
  {"action": "detect_objects", "location": "counter"},
  {"action": "pick_object", "object": "cup"},
  {"action": "move_to_location", "target": "table"},
  {"action": "place_object", "object": "cup", "location": "table"}
]

**Safety Rules:**
1. Always check object weight before picking
2. Verify path is clear before navigation
3. Avoid collisions with humans
4. Stop if uncertain about feasibility
"""
```

### چند مثالیں

```python
FEW_SHOT_EXAMPLES = [
    {
        "user": "Bring me a water bottle from the fridge",
        "assistant": """[
  {"action": "move_to_location", "target": "fridge"},
  {"action": "detect_objects", "location": "fridge"},
  {"action": "open_drawer", "drawer_id": "fridge_door"},
  {"action": "pick_object", "object": "water_bottle"},
  {"action": "close_drawer", "drawer_id": "fridge_door"},
  {"action": "move_to_location", "target": "user"},
  {"action": "place_object", "object": "water_bottle", "location": "hand"}
]"""
    },
    {
        "user": "Clean up the table",
        "assistant": """[
  {"action": "move_to_location", "target": "table"},
  {"action": "detect_objects", "location": "table"},
  {"action": "for_each_object", "objects": "detected_objects", "sub_actions": [
      {"action": "pick_object", "object": "$current_object"},
      {"action": "move_to_location", "target": "trash_bin"},
      {"action": "place_object", "object": "$current_object", "location": "trash_bin"},
      {"action": "move_to_location", "target": "table"}
  ]}
]"""
    }
]
```

### متحرک سیاق و سباق انجیکشن

```python
def build_contextual_prompt(user_instruction, robot_state, environment_state):
    context = f"""**Current Robot State:**
- Location: {robot_state['location']}
- Holding: {robot_state.get('holding_object', 'nothing')}
- Battery: {robot_state['battery_percent']}%
- Nearby Objects: {', '.join(environment_state['visible_objects'])}

**User Instruction:**
{user_instruction}

Generate an executable action plan considering the current state."""

    return context
```

## قابل عمل روبوٹ کوڈ کی پیدائش

### منصوبہ جنریشن نوڈ

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import json

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner')

        # Initialize OpenAI client
        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10
        )

        # Publish generated plans
        self.plan_pub = self.create_publisher(String, 'task_plan', 10)

    def command_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")

        # Generate plan using GPT-4
        plan = self.generate_plan(msg.data)

        # Publish plan
        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.plan_pub.publish(plan_msg)

    def generate_plan(self, instruction):
        messages = [
            {"role": "system", "content": ROBOTICS_SYSTEM_PROMPT},
            *[{"role": ex["user"], "content": ex["user"]},
              {"role": "assistant", "content": ex["assistant"]}
              for ex in FEW_SHOT_EXAMPLES],
            {"role": "user", "content": instruction}
        ]

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=messages,
                temperature=0.1,
                max_tokens=1500
            )

            plan_text = response.choices[0].message.content
            plan = json.loads(plan_text)
            self.get_logger().info(f"Generated plan: {plan}")
            return plan

        except Exception as e:
            self.get_logger().error(f"Plan generation failed: {e}")
            return []

def main():
    rclpy.init()
    node = CognitivePlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### کسٹم ایکشنز کے لیے کوڈ جنریشن

```python
def generate_python_code(natural_language_instruction):
    """Generate Python code for robot control from natural language."""

    code_prompt = f"""Generate Python code using ROS 2 to accomplish this task:
"{natural_language_instruction}"

Available APIs:
- robot.move_to(x, y, theta) - Navigate to pose
- robot.pick(object_name) - Pick up object
- robot.place(x, y, z) - Place held object
- camera.detect_objects() - Returns list of detected objects

Requirements:
- Include error handling
- Add comments explaining each step
- Use async/await for actions
- Return success/failure status

Output ONLY valid Python code, no explanations."""

    response = client.chat.completions.create(
        model="gpt-4-turbo-preview",
        messages=[{"role": "user", "content": code_prompt}],
        temperature=0.2
    )

    generated_code = response.choices[0].message.content
    # Remove markdown code blocks if present
    generated_code = generated_code.replace("```python", "").replace("```", "").strip()

    return generated_code

# Example usage
instruction = "Navigate to the table, find a red cup, and bring it to the user"
code = generate_python_code(instruction)
print(code)
```

## حفاظتی پابندیاں اور تصدیق

### منصوبہ تصدیق سسٹم

```python
class PlanValidator:
    def __init__(self):
        self.max_steps = 50
        self.safe_locations = ['kitchen', 'living_room', 'office']
        self.forbidden_actions = ['throw', 'break', 'damage']

    def validate_plan(self, plan):
        """Validate generated plan for safety and feasibility."""
        errors = []

        # Check plan length
        if len(plan) > self.max_steps:
            errors.append(f"Plan too long: {len(plan)} steps (max {self.max_steps})")

        # Check each action
        for i, step in enumerate(plan):
            # Validate action exists
            if 'action' not in step:
                errors.append(f"Step {i}: Missing 'action' field")
                continue

            action = step['action']

            # Check for forbidden actions
            if any(forbidden in action.lower() for forbidden in self.forbidden_actions):
                errors.append(f"Step {i}: Forbidden action '{action}'")

            # Validate locations
            if 'target' in step:
                if step['target'] not in self.safe_locations:
                    errors.append(f"Step {i}: Unknown location '{step['target']}'")

            # Check object manipulation preconditions
            if action == 'place_object':
                if i == 0 or not any(s['action'] == 'pick_object' for s in plan[:i]):
                    errors.append(f"Step {i}: Cannot place without picking first")

        return len(errors) == 0, errors

    def sanitize_plan(self, plan):
        """Remove unsafe actions from plan."""
        safe_plan = []
        holding_object = False

        for step in plan:
            action = step.get('action', '')

            # Filter forbidden actions
            if any(forbidden in action.lower() for forbidden in self.forbidden_actions):
                continue

            # Enforce pick-place logic
            if action == 'pick_object':
                if not holding_object:
                    safe_plan.append(step)
                    holding_object = True
            elif action == 'place_object':
                if holding_object:
                    safe_plan.append(step)
                    holding_object = False
            else:
                safe_plan.append(step)

        return safe_plan
```

### قبل از نفاذ تصدیق

```python
class SafeExecutor:
    def execute_plan_with_confirmation(self, plan, validator):
        """Execute plan with human confirmation for risky actions."""

        # Validate plan
        is_valid, errors = validator.validate_plan(plan)
        if not is_valid:
            print("Plan validation failed:")
            for error in errors:
                print(f"  - {error}")
            return False

        # Sanitize plan
        safe_plan = validator.sanitize_plan(plan)

        # Display plan to user
        print("\nProposed Plan:")
        for i, step in enumerate(safe_plan):
            print(f"{i+1}. {step['action']}: {step}")

        # Request confirmation
        response = input("\nExecute this plan? (yes/no): ")
        if response.lower() != 'yes':
            print("Plan execution cancelled by user")
            return False

        # Execute plan
        for step in safe_plan:
            success = self.execute_action(step)
            if not success:
                print(f"Action failed: {step}")
                return False

        return True

    def execute_action(self, action_dict):
        """Execute single action (placeholder)."""
        print(f"Executing: {action_dict}")
        return True  # Simulate success
```

## GPT کو ROS 2 ایکشن سرورز کے ساتھ ضم کرنا

### ایکشن انٹرفیس کی تعریف

```bash
# Create action definition
# File: cognitive_planning_interfaces/action/ExecuteTask.action

# Goal
string instruction                 # Natural language instruction
---
# Result
bool success                       # Did task complete successfully?
string result_message             # Human-readable result
int32 steps_completed             # Number of plan steps executed
---
# Feedback
string current_action             # Currently executing action
int32 step_number                 # Current step in plan
int32 total_steps                 # Total number of steps
float32 completion_percentage     # 0-100
```

### ایکشن سرور کی نفاذ

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from cognitive_planning_interfaces.action import ExecuteTask
from openai import OpenAI
import json

class TaskExecutionServer(Node):
    def __init__(self):
        super().__init__('task_execution_server')

        self.action_server = ActionServer(
            self,
            ExecuteTask,
            'execute_task',
            self.execute_callback
        )

        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        self.validator = PlanValidator()

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing task: {goal_handle.request.instruction}")

        # Generate plan from natural language
        plan = self.generate_plan(goal_handle.request.instruction)

        # Validate plan
        is_valid, errors = self.validator.validate_plan(plan)
        if not is_valid:
            goal_handle.abort()
            return ExecuteTask.Result(
                success=False,
                result_message=f"Invalid plan: {errors}",
                steps_completed=0
            )

        # Execute plan with feedback
        total_steps = len(plan)
        for i, action in enumerate(plan):
            # Publish feedback
            feedback = ExecuteTask.Feedback()
            feedback.current_action = action['action']
            feedback.step_number = i + 1
            feedback.total_steps = total_steps
            feedback.completion_percentage = (i + 1) / total_steps * 100
            goal_handle.publish_feedback(feedback)

            # Execute action
            success = await self.execute_action(action)
            if not success:
                goal_handle.abort()
                return ExecuteTask.Result(
                    success=False,
                    result_message=f"Action failed: {action}",
                    steps_completed=i
                )

        # Success
        goal_handle.succeed()
        return ExecuteTask.Result(
            success=True,
            result_message="Task completed successfully",
            steps_completed=total_steps
        )

    def generate_plan(self, instruction):
        # (Same as previous generate_plan method)
        pass

    async def execute_action(self, action_dict):
        # Execute action using robot controllers
        self.get_logger().info(f"Executing: {action_dict}")
        # TODO: Call appropriate ROS 2 service/action
        return True
```

## روبوٹ کنٹرول کے لیے مکالماتی انٹرفیس

### مکالمہ مینیجر

```python
class ConversationalRobotNode(Node):
    def __init__(self):
        super().__init__('conversational_robot')

        self.client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        self.conversation_history = []

        # Subscribe to speech input
        self.speech_sub = self.create_subscription(
            String, 'speech_input', self.speech_callback, 10
        )

        # Publish responses
        self.response_pub = self.create_publisher(String, 'robot_response', 10)

    def speech_callback(self, msg):
        user_input = msg.data
        self.conversation_history.append({"role": "user", "content": user_input})

        # Generate response
        response = self.generate_response()

        self.conversation_history.append({"role": "assistant", "content": response})

        # Publish response (for text-to-speech)
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

        # Check if response contains executable plan
        if self.is_action_command(user_input):
            self.execute_command(user_input)

    def generate_response(self):
        """Generate conversational response using GPT."""
        messages = [
            {"role": "system", "content": """You are a helpful humanoid robot assistant.
Respond naturally to user requests. If asked to perform physical tasks, acknowledge
and explain what you'll do. Keep responses concise (2-3 sentences)."""},
            *self.conversation_history[-10:]  # Last 10 messages for context
        ]

        response = self.client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=messages,
            temperature=0.7,
            max_tokens=150
        )

        return response.choices[0].message.content

    def is_action_command(self, text):
        """Detect if text contains actionable command."""
        action_keywords = ['bring', 'fetch', 'get', 'move', 'pick', 'place', 'open', 'close']
        return any(keyword in text.lower() for keyword in action_keywords)

    def execute_command(self, instruction):
        """Execute physical action based on instruction."""
        self.get_logger().info(f"Executing command: {instruction}")
        # Call task execution action server
        # (Implementation using action client)
```

## ابہام کا انتظام اور وضاحت

### ابہام کا پتہ لگانا

```python
def detect_ambiguity(instruction):
    """Check if instruction needs clarification."""

    clarification_prompt = f"""Analyze this robot instruction for ambiguity:
"{instruction}"

Is this instruction clear and unambiguous? Consider:
- Are all object references specific? (e.g., "the cup" vs "a cup")
- Are locations clearly defined?
- Are actions well-specified?

Respond with JSON:
{{
  "is_ambiguous": true/false,
  "ambiguities": ["list", "of", "unclear", "aspects"],
  "clarifying_questions": ["question 1", "question 2"]
}}"""

    response = client.chat.completions.create(
        model="gpt-4-turbo-preview",
        messages=[{"role": "user", "content": clarification_prompt}],
        temperature=0
    )

    result = json.loads(response.choices[0].message.content)
    return result

# Example
instruction = "Bring me a cup"
analysis = detect_ambiguity(instruction)

if analysis['is_ambiguous']:
    print("Clarification needed:")
    for question in analysis['clarifying_questions']:
        print(f"  - {question}")
# Output:
#   - Which cup? There are multiple cups visible.
#   - Where should I bring it?
```

### بات چیت کے ذریعے وضاحت کا لوپ

```python
class ClarificationManager:
    def __init__(self, openai_client):
        self.client = openai_client

    def execute_with_clarification(self, instruction, max_clarifications=3):
        """Execute instruction, requesting clarifications as needed."""

        original_instruction = instruction
        clarification_history = []

        for _ in range(max_clarifications):
            # Check for ambiguity
            analysis = detect_ambiguity(instruction)

            if not analysis['is_ambiguous']:
                # Instruction is clear, generate plan
                plan = self.generate_plan(instruction)
                return plan

            # Request clarification from user
            print(f"\n❓ I need clarification:")
            for question in analysis['clarifying_questions']:
                print(f"   {question}")

            user_response = input("\nYour response: ")
            clarification_history.append(user_response)

            # Update instruction with clarification
            instruction = self.incorporate_clarification(
                original_instruction,
                clarification_history
            )

        # Max clarifications reached, generate best-effort plan
        print("⚠️  Proceeding with best interpretation")
        return self.generate_plan(instruction)

    def incorporate_clarification(self, original, clarifications):
        """Merge original instruction with clarifications."""

        merge_prompt = f"""Original instruction: "{original}"

Clarifications provided:
{chr(10).join(f'- {c}' for c in clarifications)}

Generate a refined, unambiguous instruction incorporating all clarifications."""

        response = self.client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[{"role": "user", "content": merge_prompt}],
            temperature=0
        )

        return response.choices[0].message.content
```

## پیچیدہ کاموں کے لیے Chain-of-Thought پرومپٹنگ

### CoT پرومپٹ کی ساخت

```python
COT_SYSTEM_PROMPT = """You are a robot task planner. For complex tasks, think step-by-step.

Use this reasoning structure:
1. **Goal Analysis:** What is the end goal?
2. **Preconditions:** What must be true before starting?
3. **Sub-goals:** Break into smaller objectives
4. **Action Sequence:** List concrete actions
5. **Verification:** How to confirm success?

Think through each step before generating the final plan."""

def generate_plan_with_cot(instruction):
    """Generate plan using chain-of-thought reasoning."""

    messages = [
        {"role": "system", "content": COT_SYSTEM_PROMPT},
        {"role": "user", "content": f"""Task: {instruction}

First, think through the task step-by-step, then generate the action plan."""}
    ]

    response = client.chat.completions.create(
        model="gpt-4-turbo-preview",
        messages=messages,
        temperature=0.1,
        max_tokens=2000
    )

    # Response will contain reasoning + plan
    full_response = response.choices[0].message.content

    # Extract plan from response
    plan_start = full_response.find('[')
    plan_end = full_response.rfind(']') + 1
    plan_json = full_response[plan_start:plan_end]

    return json.loads(plan_json), full_response
```

### مثال: CoT کے ساتھ پیچیدہ کام

```python
instruction = "Prepare coffee for a guest in the living room"

plan, reasoning = generate_plan_with_cot(instruction)

print("=== Reasoning ===")
print(reasoning)
print("\n=== Generated Plan ===")
print(json.dumps(plan, indent=2))

# Expected reasoning:
# Goal Analysis: Make coffee and deliver to guest
# Preconditions: Coffee machine available, cup available, guest location known
# Sub-goals:
#   1. Navigate to kitchen
#   2. Prepare coffee
#   3. Transport to guest
# Action Sequence: [detailed plan]
# Verification: Guest confirms receipt of coffee
```

## کیس اسٹڈیز

### کیس اسٹڈی 1: LLM کی رہنمائی میں اشیاء کی ترتیب

**کام:** "میز پر موجود اشیاء کو رنگ کے مطابق ترتیب دیں"

**پیدا شدہ منصوبہ:**
```python
[
  {"action": "move_to_location", "target": "table"},
  {"action": "detect_objects", "location": "table"},
  {"action": "analyze_objects", "property": "color"},  # Custom action
  {"action": "for_each_color", "colors": ["red", "blue", "green"], "sub_actions": [
      {"action": "pick_objects_by_color", "color": "$current_color"},
      {"action": "place_in_container", "container": "$current_color_bin"}
  ]}
]
```

**اہم بصیرت:** LLM نے رنگ کے پتہ لگانے اور تکراری ترتیب کی ضرورت کا خود بخود اندازہ لگایا، صریح پروگرامنگ کے بغیر۔

### کیس اسٹڈی 2: مطابقتی ہیرا پھیری

**کام:** "کاؤنٹر پر پھیلے ہوئے مواد کو صاف کریں"

**LLM استدلال:**
```
This task requires:
1. Identifying the spill location
2. Selecting appropriate cleaning tool (paper towel/sponge)
3. Wiping motion
4. Disposal

Challenges:
- Spill extent unknown → must detect visually
- Tool selection based on spill type
```

**پیدا شدہ منصوبہ شامل کرتا ہے:**
- بصری معائنہ کا مرحلہ
- مشروط آلہ کا انتخاب
- پھیلاؤ کی شکل کے مطابق مطابقتی صاف کرنے کا راستہ

### کیس اسٹڈی 3: غلطی کی بازیابی

**کام:** "نیلی مگ اٹھائیں"

**نفاذ کی غلطی:** مگ پکڑتے وقت پھسل جاتی ہے

**LLM بازیابی کی حکمت عملی:**
```python
# LLM generates recovery plan when notified of failure
recovery_prompt = f"The robot failed to pick up the blue mug (grasp failure). Generate a recovery plan."

recovery_plan = [
    {"action": "release_gripper"},
    {"action": "adjust_grasp_approach", "offset": [0, 0, 0.05]},
    {"action": "increase_gripper_force", "force": 1.2},
    {"action": "retry_pick", "object": "blue_mug"}
]
```

## کارکردگی اور لاگت کی بہتری

### بار بار پوچھے گئے سوالات کے لیے کیشنگ

```python
from functools import lru_cache
import hashlib

class CachedLLMPlanner:
    def __init__(self):
        self.cache = {}

    def generate_plan_cached(self, instruction):
        # Hash instruction for cache key
        cache_key = hashlib.md5(instruction.encode()).hexdigest()

        if cache_key in self.cache:
            print("✓ Using cached plan")
            return self.cache[cache_key]

        # Generate new plan
        plan = self.generate_plan(instruction)
        self.cache[cache_key] = plan

        return plan
```

### ٹوکن کے استعمال کی نگرانی

```python
class TokenMonitor:
    def __init__(self):
        self.total_tokens = 0
        self.total_cost = 0.0

    def track_usage(self, response):
        tokens_used = response.usage.total_tokens
        self.total_tokens += tokens_used

        # GPT-4 Turbo pricing (as of 2024)
        prompt_tokens = response.usage.prompt_tokens
        completion_tokens = response.usage.completion_tokens

        cost = (prompt_tokens * 0.01 / 1000) + (completion_tokens * 0.03 / 1000)
        self.total_cost += cost

        print(f"Tokens used: {tokens_used} | Cost: ${cost:.4f} | Total: ${self.total_cost:.2f}")
```

## عملی مشق

LLM سے چلنے والے کام کی منصوبہ بندی کے مکمل سسٹم کی تعمیر کریں:

1. **سیٹ اپ:** OpenAI انضمام کے ساتھ ROS 2 پیکیج بنائیں
2. **نفاذ:** منصوبہ بندی کے ساتھ CognitivePlannerNode بنائیں
3. **تصدیق:** حفاظتی چیکس کے لیے PlanValidator شامل کریں
4. **ٹیسٹ:** آواز کے احکامات بھیجیں، پیدا شدہ منصوبے دیکھیں
5. **جائزہ:** کامیابی کی شرح، ٹوکن کا استعمال، تاخیر کی پیمائش کریں

**کامیابی کے معیار:**
- [ ] 10 مختلف ہدایات کے لیے درست منصوبے پیدا کرتا ہے
- [ ] منصوبہ بندی کی تصدیق حفاظتی خلاف ورزیوں کا پتہ لگاتی ہے
- [ ] اوسط تاخیر < 3 سیکنڈ
- [ ] ہر منصوبے کی لاگت < $0.05

## خلاصہ

OpenAI کے GPT ماڈلز روبوٹس کے لیے فطری زبان کے کنٹرول اور معرفی منصوبہ بندی کو ممکن بناتے ہیں۔ پرومپٹ انجینئرنگ، حفاظتی تصدیق، اور ROS 2 انضمام کو ملانے سے، انسان نما روبوٹ روزمرہ کی زبان میں بیان کیے گئے پیچیدہ کاموں کو سمجھ اور انجام دے سکتے ہیں۔ تاہم، LLMs کو محفوظ اور قابل اعتماد آپریشن کو یقینی بنانے کے لیے احتیاط سے محدود اور تصدیق کی جانی چاہیے۔

## اگلا باب

[باب 4.3: انسان نما روبوٹس کے لیے ملٹی موڈل تعامل](./chapter-4-3.md) میں، آپ زبان سے آگے بڑھ کر، امیری مکالمہ، اشاروں، اور بصری سمجھ کو شامل کریں گے تاکہ انسان اور روبوٹ کے درمیان غنی تعامل کو ممکن بنایا جا سکے۔