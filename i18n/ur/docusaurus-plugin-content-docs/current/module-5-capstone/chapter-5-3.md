---
id: module-5-chapter-3
title: "Voice Processing and Command Interface"
sidebar_label: "5.3 Voice Interface"
sidebar_position: 3
description: "Implement voice command interface: speech recognition with Whisper, intent classification, command execution, and natural language feedback for humanoid control"
keywords: [voice-interface, speech-recognition, whisper, intent-classification, natural-language]
estimated_time: 75
prerequisites:
  - module-4-chapter-3
  - module-5-chapter-2
learning_outcomes:
  - Integrate Whisper speech recognition with ROS 2
  - Classify user intents from transcribed speech
  - Map natural language commands to robot actions
  - Generate spoken feedback with text-to-speech
  - Handle speech recognition errors gracefully
hardware_tier: premium
---

# باب 5.3: آواز کی پروسیسرنگ اور کمانڈ انٹرفیس

آواز کی بات چیت ہیومینوئیڈ روبوٹس کے لیے ایک اہم صلاحیت ہے۔ یہ باب گفتگو کی شناخت سے عمل کی انجام دہی تک مکمل آواز کے کمانڈ پائپ لائن کو نافذ کرتا ہے۔

## 5.3.1 آواز پر مبنی انسان-روبوٹ بات چیت کا تعارف

آواز کے کمانڈز انسان-روبوٹ بات چیت کے لیے سب سے قدرتی اور بصیرت افزا انٹرفیس فراہم کرتے ہیں۔ روایتی ان پٹ طریقوں کے برعکس، گفتگو صارفین کو روبوٹس کے ساتھ اپنی مادری زبان میں بات چیت کرنے کی اجازت دیتی ہے، جس سے روبوٹکس غیر تکنیکی صارفین کے لیے قابل رسائی بن جاتا ہے۔ گھر، ہسپتال یا عوامی مقامات میں کام کرنے والے ہیومینوئیڈ روبوٹس کے لیے، آواز کی بات چیت یکساں تعاون کے لیے ضروری ہو جاتی ہے۔

آواز کی پروسیسرنگ پائپ لائن میں کئی اہم مراحل شامل ہوتے ہیں:
- **گفتگو کی شناخت**: آڈیو کو متن میں تبدیل کرنا
- **ارادہ کی سمجھ**: نقل شدہ گفتگو سے معنی نکالنا
- **عمل کی منصوبہ بندی**: ارادوں کو روبوٹ کی صلاحیتوں سے منسلک کرنا
- **انجام**: درخواست شدہ کام انجام دینا
- **فیڈبیک کی تخلیق**: گفتگو کے ذریعے اقدامات کی تصدیق کرنا

## 5.3.2 نظام کی ساخت کا جائزہ

ہمارا آواز کے کمانڈ نظام ROS 2 کے ماحولیاتی نظام میں متعدد اجزاء کو مربوط کرتا ہے:

```python
# voice_interface/voice_pipeline_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import numpy as np
import threading
import queue

class VoiceCommandPipeline(Node):
    def __init__(self):
        super().__init__('voice_pipeline')
        
        # Audio input subscription
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio/input',
            self.audio_callback,
            10
        )
        
        # Transcription publisher
        self.transcript_pub = self.create_publisher(
            String,
            'voice/transcript',
            10
        )
        
        # Intent publisher
        self.intent_pub = self.create_publisher(
            String,
            'voice/intent',
            10
        )
        
        # Feedback publisher (for TTS)
        self.feedback_pub = self.create_publisher(
            String,
            'voice/feedback',
            10
        )
        
        # Audio buffer for processing
        self.audio_queue = queue.Queue()
        self.processing_thread = threading.Thread(
            target=self.process_audio_stream,
            daemon=True
        )
        self.processing_thread.start()
        
        self.get_logger().info('Voice pipeline initialized')
    
    def audio_callback(self, msg):
        """Buffer incoming audio data"""
        audio_array = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_queue.put(audio_array)
    
    def process_audio_stream(self):
        """Continuous audio processing loop"""
        while rclpy.ok():
            try:
                audio_chunk = self.audio_queue.get(timeout=1.0)
                # Processing happens in subsequent sections
            except queue.Empty:
                continue
```

## 5.3.3 OpenAI Whisper کے ساتھ گفتگو کی شناخت

Whisper متعدد زبانوں میں جدید گفتگو کی شناخت کی درستگی فراہم کرتا ہے۔ ہم اسے ریئل ٹائم نقل کے لیے مربوط کرتے ہیں:

```python
# voice_interface/whisper_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import whisper
import numpy as np
import io
import wave
from collections import deque

class WhisperRecognitionNode(Node):
    def __init__(self):
        super().__init__('whisper_recognition')
        
        # Load Whisper model
        model_size = self.declare_parameter('model_size', 'base').value
        self.model = whisper.load_model(model_size)
        self.get_logger().info(f'Loaded Whisper model: {model_size}')
        
        # Audio parameters
        self.sample_rate = 16000
        self.chunk_duration = 3.0  # seconds
        self.buffer_size = int(self.sample_rate * self.chunk_duration)
        self.audio_buffer = deque(maxlen=self.buffer_size)
        
        # ROS interfaces
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio/input',
            self.audio_callback,
            10
        )
        
        self.transcript_pub = self.create_publisher(
            String,
            'voice/transcript',
            10
        )
        
        # Processing timer
        self.create_timer(1.0, self.process_audio)
        
    def audio_callback(self, msg):
        """Accumulate audio samples"""
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        self.audio_buffer.extend(audio_data)
    
    def process_audio(self):
        """Transcribe accumulated audio"""
        if len(self.audio_buffer) < self.sample_rate:
            return
        
        # Convert buffer to audio format
        audio_array = np.array(self.audio_buffer, dtype=np.float32)
        audio_array = audio_array / 32768.0  # Normalize
        
        try:
            # Transcribe with Whisper
            result = self.model.transcribe(
                audio_array,
                language='en',
                task='transcribe',
                fp16=False
            )
            
            text = result['text'].strip()
            
            if text:
                self.get_logger().info(f'Transcribed: {text}')
                msg = String()
                msg.data = text
                self.transcript_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WhisperRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 5.3.4 GPT ماڈلز کے ساتھ ارادہ کی درجہ بندی

ایک بار گفتگو نقل ہو جانے کے بعد، ہمیں صارف کے ارادہ کو سمجھنا اور متعلقہ پیرامیٹرز نکالنا ہوتے ہیں:

```python
# voice_interface/intent_classifier.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class IntentClassifier(Node):
    def __init__(self):
        super().__init__('intent_classifier')
        
        # Configure OpenAI
        api_key = self.declare_parameter('openai_api_key', '').value
        openai.api_key = api_key
        
        # Define supported intents
        self.intent_schema = {
            "navigation": ["go", "move", "navigate", "walk"],
            "manipulation": ["pick", "grab", "place", "put"],
            "interaction": ["greet", "wave", "shake", "look"],
            "information": ["what", "where", "who", "when", "tell"]
        }
        
        self.transcript_sub = self.create_subscription(
            String,
            'voice/transcript',
            self.transcript_callback,
            10
        )
        
        self.intent_pub = self.create_publisher(
            String,
            'voice/intent',
            10
        )
    
    def transcript_callback(self, msg):
        """Classify intent from transcript"""
        transcript = msg.data
        intent_data = self.classify_intent(transcript)
        
        if intent_data:
            intent_msg = String()
            intent_msg.data = json.dumps(intent_data)
            self.intent_pub.publish(intent_msg)
            self.get_logger().info(f'Intent: {intent_data}')
    
    def classify_intent(self, text):
        """Use GPT to extract structured intent"""
        system_prompt = """You are an intent classifier for a humanoid robot.
Extract the intent and parameters from user commands.

Return JSON with:
- intent: primary action (navigation/manipulation/interaction/information)
- action: specific verb
- target: object or location
- parameters: additional details

Examples:
"Go to the kitchen" -> {"intent": "navigation", "action": "go", "target": "kitchen"}
"Pick up the red cup" -> {"intent": "manipulation", "action": "pick", "target": "red cup"}
"""
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": text}
                ],
                temperature=0.1,
                max_tokens=200
            )
            
            result = response.choices[0].message.content
            intent_data = json.loads(result)
            return intent_data
            
        except Exception as e:
            self.get_logger().error(f'Intent classification failed: {e}')
            return None
```

## 5.3.5 کمانڈ کی تحلیل اور عمل کی منسلک

ارادوں کو قابل انجام روبوٹ اقدامات میں تبدیل کرنے کے لیے قدرتی زبان کو ROS 2 عمل انٹرفیس سے منسلک کرنا ہوتا ہے:

```python
# voice_interface/action_executor.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from manipulation_msgs.action import PickPlace
import json

class VoiceActionExecutor(Node):
    def __init__(self):
        super().__init__('voice_action_executor')
        
        # Action clients
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.pick_client = ActionClient(
            self,
            PickPlace,
            'pick_place'
        )
        
        # Known locations database
        self.locations = {
            'kitchen': {'x': 5.0, 'y': 2.0, 'z': 0.0},
            'living room': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'bedroom': {'x': -3.0, 'y': 4.0, 'z': 0.0}
        }
        
        self.intent_sub = self.create_subscription(
            String,
            'voice/intent',
            self.intent_callback,
            10
        )
        
        self.feedback_pub = self.create_publisher(
            String,
            'voice/feedback',
            10
        )
    
    def intent_callback(self, msg):
        """Execute action based on intent"""
        try:
            intent_data = json.loads(msg.data)
            intent_type = intent_data.get('intent')
            
            if intent_type == 'navigation':
                self.execute_navigation(intent_data)
            elif intent_type == 'manipulation':
                self.execute_manipulation(intent_data)
            elif intent_type == 'interaction':
                self.execute_interaction(intent_data)
            else:
                self.send_feedback("I don't understand that command.")
                
        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            self.send_feedback("Sorry, I couldn't complete that action.")
    
    def execute_navigation(self, intent):
        """Navigate to target location"""
        target = intent.get('target', '').lower()
        
        if target not in self.locations:
            self.send_feedback(f"I don't know where {target} is.")
            return
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.locations[target]['x']
        goal_msg.pose.pose.position.y = self.locations[target]['y']
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.send_feedback(f"Moving to {target}.")
        
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
    
    def execute_manipulation(self, intent):
        """Execute pick/place action"""
        action = intent.get('action')
        target = intent.get('target')
        
        self.send_feedback(f"Attempting to {action} {target}.")
        
        goal_msg = PickPlace.Goal()
        goal_msg.object_name = target
        goal_msg.action_type = action
        
        self.pick_client.wait_for_server()
        self.pick_client.send_goal_async(goal_msg)
    
    def execute_interaction(self, intent):
        """Execute social interaction"""
        action = intent.get('action')
        self.send_feedback(f"Performing {action} gesture.")
        # Trigger gesture controller
    
    def send_feedback(self, text):
        """Publish feedback for TTS"""
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(f'Feedback: {text}')
```

## 5.3.6 متن سے گفتگو کا جواب پیدا کرنا

گفتگو کے ذریعے فیڈبیک فراہم کرنا بات چیت کے حلقہ کو مکمل کرتا ہے:

```python
# voice_interface/tts_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import pyttsx3
import numpy as np

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # Initialize TTS engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 0.9)
        
        # Select voice
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[0].id)
        
        self.feedback_sub = self.create_subscription(
            String,
            'voice/feedback',
            self.feedback_callback,
            10
        )
        
        self.audio_pub = self.create_publisher(
            AudioData,
            'audio/output',
            10
        )
    
    def feedback_callback(self, msg):
        """Convert text to speech"""
        text = msg.data
        self.get_logger().info(f'Speaking: {text}')
        
        try:
            self.engine.say(text)
            self.engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 5.3.7 ہینڈز فری آپریشن کے لیے ویک ورڈ کی شناخت

ویک ورڈ کی شناخت کو نافذ کرنا روبوٹ کو صرف اس وقت فعال کرتا ہے جب اسے مخاطب کیا جاتا ہے:

```python
# voice_interface/wake_word_detector.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData
import pvporcupine
import numpy as np

class WakeWordDetector(Node):
    def __init__(self):
        super().__init__('wake_word_detector')
        
        # Initialize Porcupine wake word engine
        access_key = self.declare_parameter('porcupine_key', '').value
        self.porcupine = pvporcupine.create(
            access_key=access_key,
            keywords=['jarvis', 'computer']
        )
        
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio/raw',
            self.audio_callback,
            10
        )
        
        self.wake_pub = self.create_publisher(
            Bool,
            'voice/wake_detected',
            10
        )
        
        self.is_active = False
        self.timeout_timer = None
        
    def audio_callback(self, msg):
        """Detect wake word in audio stream"""
        pcm = np.frombuffer(msg.data, dtype=np.int16)
        
        # Process in frames
        frame_length = self.porcupine.frame_length
        for i in range(0, len(pcm) - frame_length, frame_length):
            frame = pcm[i:i + frame_length]
            keyword_index = self.porcupine.process(frame)
            
            if keyword_index >= 0:
                self.get_logger().info('Wake word detected!')
                self.activate_listening()
    
    def activate_listening(self):
        """Enable voice command processing"""
        self.is_active = True
        
        msg = Bool()
        msg.data = True
        self.wake_pub.publish(msg)
        
        # Auto-deactivate after 10 seconds
        if self.timeout_timer:
            self.timeout_timer.cancel()
        
        self.timeout_timer = self.create_timer(
            10.0,
            self.deactivate_listening
        )
    
    def deactivate_listening(self):
        """Disable voice command processing"""
        self.is_active = False
        msg = Bool()
        msg.data = False
        self.wake_pub.publish(msg)
        
        if self.timeout_timer:
            self.timeout_timer.cancel()
```

## 5.3.8 غلطیوں کا انتظام اور ابہام کی حل

مضبوط آواز کے نظام کو شناخت کی غلطیوں اور مبہم کمانڈز کا انتظام کرنا ہوتا ہے:

```python
# voice_interface/error_handler.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from difflib import get_close_matches

class VoiceErrorHandler(Node):
    def __init__(self):
        super().__init__('voice_error_handler')
        
        # Known vocabulary
        self.known_objects = ['cup', 'bottle', 'book', 'phone']
        self.known_locations = ['kitchen', 'bedroom', 'living room']
        self.known_actions = ['pick', 'place', 'go', 'move']
        
        self.intent_sub = self.create_subscription(
            String,
            'voice/intent_raw',
            self.validate_intent,
            10
        )
        
        self.validated_pub = self.create_publisher(
            String,
            'voice/intent',
            10
        )
        
        self.clarification_pub = self.create_publisher(
            String,
            'voice/clarification',
            10
        )
    
    def validate_intent(self, msg):
        """Validate and correct intent data"""
        try:
            intent = json.loads(msg.data)
            
            # Check confidence threshold
            if intent.get('confidence', 1.0) < 0.6:
                self.request_clarification(
                    "I didn't quite understand. Could you repeat that?"
                )
                return
            
            # Validate action
            action = intent.get('action', '').lower()
            if action not in self.known_actions:
                correction = self.find_closest_match(
                    action,
                    self.known_actions
                )
                if correction:
                    intent['action'] = correction
                    self.get_logger().warn(
                        f'Corrected action: {action} -> {correction}'
                    )
            
            # Validate target
            target = intent.get('target', '').lower()
            if intent['intent'] == 'manipulation':
                correction = self.find_closest_match(
                    target,
                    self.known_objects
                )
            elif intent['intent'] == 'navigation':
                correction = self.find_closest_match(
                    target,
                    self.known_locations
                )
            
            if correction and correction != target:
                self.request_confirmation(
                    f"Did you mean {correction}?",
                    intent
                )
                return
            
            # Publish validated intent
            validated_msg = String()
            validated_msg.data = json.dumps(intent)
            self.validated_pub.publish(validated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Validation error: {e}')
    
    def find_closest_match(self, word, vocabulary):
        """Find closest matching word"""
        matches = get_close_matches(word, vocabulary, n=1, cutoff=0.6)
        return matches[0] if matches else None
    
    def request_clarification(self, message):
        """Ask user to clarify"""
        msg = String()
        msg.data = message
        self.clarification_pub.publish(msg)
    
    def request_confirmation(self, message, intent):
        """Request confirmation for corrected intent"""
        msg = String()
        msg.data = json.dumps({
            'type': 'confirmation',
            'message': message,
            'intent': intent
        })
        self.clarification_pub.publish(msg)
```

## 5.3.9 کئی مرحلہ کاموں کا انجام

مختلف کمانڈز اکثر کئی مسلسل اقدامات کی ضرورت ہوتی ہے:

```python
# voice_interface/task_planner.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VoiceTaskPlanner(Node):
    def __init__(self):
        super().__init__('voice_task_planner')
        
        self.intent_sub = self.create_subscription(
            String,
            'voice/intent',
            self.plan_task,
            10
        )
        
        self.action_pub = self.create_publisher(
            String,
            'robot/action_sequence',
            10
        )
        
        # Task templates
        self.task_templates = {
            'bring': [
                {'type': 'navigation', 'target': 'object_location'},
                {'type': 'manipulation', 'action': 'pick'},
                {'type': 'navigation', 'target': 'user_location'},
                {'type': 'manipulation', 'action': 'place'}
            ],
            'clean': [
                {'type': 'navigation', 'target': 'area'},
                {'type': 'manipulation', 'action': 'pick_clutter'},
                {'type': 'navigation', 'target': 'disposal'},
                {'type': 'manipulation', 'action': 'place'}
            ]
        }
    
    def plan_task(self, msg):
        """Decompose complex commands into action sequences"""
        try:
            intent = json.loads(msg.data)
            action = intent.get('action', '').lower()
            
            if action in self.task_templates:
                sequence = self.create_action_sequence(
                    self.task_templates[action],
                    intent
                )
                
                sequence_msg = String()
                sequence_msg.data = json.dumps(sequence)
                self.action_pub.publish(sequence_msg)
                
                self.get_logger().info(
                    f'Planned {len(sequence)} step task'
                )
                
        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
    
    def create_action_sequence(self, template, intent):
        """Instantiate action sequence from template"""
        sequence = []
        
        for step in template:
            action = step.copy()
            
            # Fill in parameters from intent
            if 'target' in action and action['target'] == 'object_location':
                action['target'] = intent.get('target')
            
            sequence.append(action)
        
        return sequence
```

## 5.3.10 لانچ کی تشکیل

مکمل نظام لانچ فائل:

```python
# launch/voice_interface.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'whisper_model',
            default_value='base',
            description='Whisper model size'
        ),
        
        Node(
            package='voice_interface',
            executable='wake_word_detector',
            name='wake_word_detector',
            parameters=[{
                'porcupine_key': 'YOUR_KEY_HERE'
            }]
        ),
        
        Node(
            package='voice_interface',
            executable='whisper_recognition',
            name='whisper_recognition',
            parameters=[{
                'model_size': LaunchConfiguration('whisper_model')
            }]
        ),
        
        Node(
            package='voice_interface',
            executable='intent_classifier',
            name='intent_classifier',
            parameters=[{
                'openai_api_key': 'YOUR_KEY_HERE'
            }]
        ),
        
        Node(
            package='voice_interface',
            executable='error_handler',
            name='error_handler'
        ),
        
        Node(
            package='voice_interface',
            executable='task_planner',
            name='task_planner'
        ),
        
        Node(
            package='voice_interface',
            executable='action_executor',
            name='action_executor'
        ),
        
        Node(
            package='voice_interface',
            executable='tts_node',
            name='tts_node'
        )
    ])
```

## 5.3.11 ٹیسٹنگ اور جائزہ

آواز کے انٹرفیس کی کارکردگی کی پیمائش:

```python
# scripts/evaluate_voice_system.py
import rclpy
from rclpy.node import Node
import time
import json

class VoiceSystemEvaluator(Node):
    def __init__(self):
        super().__init__('voice_evaluator')
        
        self.test_commands = [
            "Go to the kitchen",
            "Pick up the red cup",
            "Bring me the book from the table",
            "What time is it",
            "Wave hello"
        ]
        
        self.results = {
            'transcription_accuracy': [],
            'intent_accuracy': [],
            'execution_success': [],
            'response_latency': []
        }
    
    def run_evaluation(self):
        """Execute test suite"""
        for command in self.test_commands:
            start_time = time.time()
            
            # Simulate command
            result = self.test_command(command)
            
            latency = time.time() - start_time
            self.results['response_latency'].append(latency)
            
            self.get_logger().info(
                f'Command: {command} | '
                f'Success: {result["success"]} | '
                f'Latency: {latency:.2f}s'
            )
        
        self.print_summary()
    
    def print_summary(self):
        """Display evaluation metrics"""
        avg_latency = sum(self.results['response_latency']) / len(self.results['response_latency'])
        
        print("\n=== Voice System Evaluation ===")
        print(f"Average Response Latency: {avg_latency:.2f}s")
        print(f"Commands Tested: {len(self.test_commands)}")

def main(args=None):
    rclpy.init(args=args)
    evaluator = VoiceSystemEvaluator()
    evaluator.run_evaluation()
    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.3.12 کارکردگی کی بہتری

ریئل ٹائم کارکردگی کے لیے آواز کے پائپ لائن کی بہتری:

```python
# voice_interface/optimized_pipeline.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import concurrent.futures
import threading

class OptimizedVoicePipeline(Node):
    def __init__(self):
        super().__init__('optimized_voice_pipeline')
        
        # Thread pool for parallel processing
        self.executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=4
        )
        
        # Audio preprocessing cache
        self.audio_cache = {}
        self.cache_lock = threading.Lock()
        
        # Model warmup
        self.warmup_models()
    
    def warmup_models(self):
        """Preload and warmup ML models"""
        self.get_logger().info('Warming up models...')
        
        # Dummy inference to initialize CUDA
        dummy_audio = np.random.randn(16000).astype(np.float32)
        self.whisper_model.transcribe(dummy_audio)
        
        self.get_logger().info('Models ready')
    
    def parallel_process(self, audio_data):
        """Process audio in parallel stages"""
        # Submit tasks to thread pool
        future_vad = self.executor.submit(
            self.voice_activity_detection,
            audio_data
        )
        
        future_denoise = self.executor.submit(
            self.noise_reduction,
            audio_data
        )
        
        # Wait for preprocessing
        has_voice = future_vad.result()
        clean_audio = future_denoise.result()
        
        if has_voice:
            return self.transcribe(clean_audio)
        return None
    
    def voice_activity_detection(self, audio):
        """Quick VAD check before expensive transcription"""
        energy = np.sum(audio ** 2)
        return energy > 0.01  # Threshold
    
    def noise_reduction(self, audio):
        """Simple spectral subtraction"""
        # Simplified noise reduction
        return audio
```

## 5.3.13 جدید قدرتی زبان کی سمجھ

سیاق و سباق سے آگاہ کمانڈ کی تشریح کو نافذ کرنا:

```python
# voice_interface/contextual_nlu.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import deque

class ContextualNLU(Node):
    def __init__(self):
        super().__init__('contextual_nlu')
        
        # Conversation history
        self.conversation_history = deque(maxlen=5)
        
        # Context tracking
        self.current_context = {
            'last_object': None,
            'last_location': None,
            'pending_confirmation': None
        }
        
        self.transcript_sub = self.create_subscription(
            String,
            'voice/transcript',
            self.process_with_context,
            10
        )
    
    def process_with_context(self, msg):
        """Resolve references using conversation history"""
        transcript = msg.data.lower()
        
        # Handle pronouns and references
        if 'it' in transcript or 'that' in transcript:
            if self.current_context['last_object']:
                transcript = transcript.replace(
                    'it',
                    self.current_context['last_object']
                )
        
        if 'there' in transcript:
            if self.current_context['last_location']:
                transcript = transcript.replace(
                    'there',
                    self.current_context['last_location']
                )
        
        # Handle confirmations
        if transcript in ['yes', 'yeah', 'sure', 'okay']:
            if self.current_context['pending_confirmation']:
                self.execute_pending_action()
                return
        
        # Update conversation history
        self.conversation_history.append({
            'transcript': transcript,
            'timestamp': self.get_clock().now()
        })
        
        # Process enhanced transcript
        self.classify_intent(transcript)
    
    def execute_pending_action(self):
        """Execute action awaiting confirmation"""
        action = self.current_context['pending_confirmation']
        self.get_logger().info(f'Executing confirmed action: {action}')
        self.current_context['pending_confirmation'] = None
```

## 5.3