---
id: module-4-chapter-5
title: "Conversational Robotics and Social Intelligence"
sidebar_label: "4.5 Conversational Robotics"
sidebar_position: 5
description: "Build socially intelligent humanoid robots: dialogue management, emotion recognition, context-aware responses, and long-term human-robot relationships"
keywords: [conversational-ai, dialogue-management, emotion-recognition, social-intelligence, hri]
estimated_time: 90
prerequisites:
  - module-4-chapter-2
  - module-4-chapter-3
learning_outcomes:
  - Implement dialogue state tracking for robots
  - Recognize human emotions from multimodal cues
  - Generate context-aware conversational responses
  - Design personality and social norms for robots
  - Measure social interaction quality metrics
hardware_tier: premium
---

# باب 4.5: گفتگو کی روبوٹکس اور سماجی ذہانت

انسانی ماحول میں کام کرنے والے انسانی شکل کے روبوٹوں کو فطری اور مناسب تعاملات کے لیے سماجی ذہانت کی ضرورت ہوتی ہے۔ یہ باب جذباتی آگہی کے ساتھ گفتگو کے نظام کی تعمیر پر محیط ہے۔

## 4.5.1 گفتگو کی حالت کی نگرانی اور انتظام

موثر روبوٹ گفتگوں کے لیے متعدد مرحلوں پر سیاق و سباق کو برقرار رکھنا اور گفتگو کے بہاؤ کا انتظام کرنا ضروری ہے۔

### گفتگو کی حالت کی ساخت

گفتگو کی حالت کا ٹریکر کئی اہم اجزاء کو برقرار رکھتا ہے:

```python
class DialogueState:
    def __init__(self):
        self.conversation_history = []
        self.current_intent = None
        self.entities = {}
        self.context_stack = []
        self.user_preferences = {}
        self.emotional_state = "neutral"
        
    def update(self, user_input, robot_response):
        self.conversation_history.append({
            'user': user_input,
            'robot': robot_response,
            'timestamp': time.time()
        })
        
    def get_context_window(self, turns=5):
        return self.conversation_history[-turns:]
```

### ارادہ کی شناخت اور سلاٹ بھرنے کا عمل

جدید گفتگو کے نظام صارفین کے ارادوں کی شناخت کرتے ہیں اور متعلقہ معلومات نکالتے ہیں:

- **ارادہ کی درجہ بندی**: یہ طے کرنا کہ صارف کیا چاہتا ہے (سلام، سوال، حکم، شکایت)
- **عنصر کی نکاسی**: اہم معلومات کی شناخت (نام، تاریخیں، مقامات، اشیاء)
- **سلاٹ بھرنے کا عمل**: کام کی تکمیل کے لیے ضروری پیرامیٹرز اکٹھا کرنا

### متعدد مرحلوں پر گفتگو کا انتظام

روبوٹوں کو پیچیدہ گفتگوں کا انتظام کرنا ہوتا ہے جو متعدد تبادلوں پر محیط ہوتے ہیں:

1. **سیاق و سباق کی برقراری**: یہ یاد رکھنا کہ پہلے کیا بات ہوئی تھی
2. **حوالہ جات کی تفہیم**: ضمیر اور ضمنی حوالہ جات کی سمجھ ("وہ ایک"، "نیلا ایک")
3. **وضاحت کی حکمت عملیاں**: غیر یقینی صورت میں پیچھے کے سوالات پوچھنا
4. **موضوع کی تبدیلی**: گفتگو کی سمت میں تبدیلیوں کا خوبی سے انتظام

## 4.5.2 چہرے کے اظہار سے جذبات کی شناخت

چہرے کے اظہار اہم جذباتی اشارے فراہم کرتے ہیں جنہیں روبوٹ سیکھ سکتے ہیں۔

### چہرے کے تجزیے کے لیے کمپیوٹر ویژن

جدید جذباتی شناخت کے نظام گہرائی سے سیکھنے کے طریقے استعمال کرتے ہیں:

```python
import cv2
import tensorflow as tf

class FacialEmotionRecognizer:
    def __init__(self, model_path):
        self.model = tf.keras.models.load_model(model_path)
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        self.emotions = ['angry', 'disgust', 'fear', 'happy', 
                        'sad', 'surprise', 'neutral']
    
    def detect_emotion(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        
        emotions_detected = []
        for (x, y, w, h) in faces:
            face_roi = gray[y:y+h, x:x+w]
            face_roi = cv2.resize(face_roi, (48, 48))
            face_roi = face_roi.reshape(1, 48, 48, 1) / 255.0
            
            prediction = self.model.predict(face_roi)
            emotion_idx = np.argmax(prediction)
            confidence = prediction[0][emotion_idx]
            
            emotions_detected.append({
                'emotion': self.emotions[emotion_idx],
                'confidence': float(confidence),
                'bbox': (x, y, w, h)
            })
        
        return emotions_detected
```

### چہرے کے عمل کی کوڈنگ نظام (FACS)

FACS چہرے کی حرکات کے تجزیے کے لیے ایک جامع فریم ورک فراہم کرتا ہے:

- **عمل کی اکائیاں (AUs)**: فردی پٹھوں کی حرکات (مثال کے طور پر، AU1: اندرونی بھوں کا اٹھانا)
- **جذبات کی نقشہ بندی**: AUs کے امتزاج مخصوص جذبات کی نشان دہی کرتے ہیں
- **شدت کی پیمائش**: اظہار کی شدت کا تعین

### جذبات کی شناخت کے لیے اہم چہرے کی خصوصیات

- **بھوں**: پوزیشن اور زاویہ حیرت، غصہ، یا غم کی نشان دہی کرتا ہے
- **آنکھیں**: کھلنا اور نگاہ کی سمت دلچسپی اور جذبات کا اظہار کرتی ہے
- **منہ**: شکل خوشی، نفرت، یا خوف کی نشان دہی کرتی ہے
- **کل چہرے کا تناؤ**: پٹھوں کی سرگرمی کے نمونے جذباتی حالت کا انکشاف کرتے ہیں

## 4.5.3 آواز کی لہجے سے جذبات کی شناخت

آواز میں بولے گئے الفاظ سے زیادہ جذباتی معلومات ہوتی ہیں۔

### جذبات کی شناخت کے لیے آواز کی خصوصیات

اہم آواز کے پیرامیٹرز میں شامل ہیں:

```python
import librosa
import numpy as np

class VoiceEmotionAnalyzer:
    def extract_features(self, audio_file):
        y, sr = librosa.load(audio_file, duration=3)
        
        # Pitch and fundamental frequency
        pitches, magnitudes = librosa.piptrack(y=y, sr=sr)
        pitch_mean = np.mean(pitches[pitches > 0])
        
        # Energy and intensity
        rms = librosa.feature.rms(y=y)[0]
        energy_mean = np.mean(rms)
        
        # Speaking rate (zero crossing rate)
        zcr = librosa.feature.zero_crossing_rate(y)[0]
        
        # Spectral features
        spectral_centroid = librosa.feature.spectral_centroid(y=y, sr=sr)[0]
        
        # MFCCs (Mel-frequency cepstral coefficients)
        mfccs = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)
        
        return {
            'pitch_mean': pitch_mean,
            'energy': energy_mean,
            'speaking_rate': np.mean(zcr),
            'spectral_centroid': np.mean(spectral_centroid),
            'mfccs': np.mean(mfccs, axis=1)
        }
```

### جذبات کے لہجے کے اشارے

- **پچ**: بلند پچ اکثر خوشی یا بے چینی کی نشان دہی کرتا ہے؛ کم پچ غم یا سکون کی نشان دہی کرتا ہے
- **شدت**: بلند آواز غصہ یا خوشی کی نشان دہی کر سکتی ہے؛ آہستہ آواز غم یا قریبیت کی نشان دہی کرتی ہے
- **بولنے کی رفتار**: تیز بولنا خوشی یا بے چینی کی نشان دہی کرتا ہے؛ سست بولنا غم یا غور و فکر کی نشان دہی کرتا ہے
- **آواز کی کیفیت**: سانس بھر کر، تناؤ یا کھردری آواز جذباتی حالت کا انکشاف کرتی ہے

## 4.5.4 جسمانی زبان اور جسمانی اشاروں کی سمجھ

جسمانی حالت کے ذریعے غیر لفظی مواصلات اہم جذباتی سیاق و سباق فراہم کرتی ہے۔

### جسمانی حالت کے تجزیے کا نظام

```python
import mediapipe as mp

class PostureAnalyzer:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        
    def analyze_posture(self, frame):
        results = self.pose.process(frame)
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            
            # Calculate shoulder angle (indicates tension/relaxation)
            shoulder_angle = self.calculate_shoulder_angle(landmarks)
            
            # Head orientation (indicates engagement)
            head_tilt = self.calculate_head_tilt(landmarks)
            
            # Body openness (arms position)
            arm_openness = self.calculate_arm_openness(landmarks)
            
            return {
                'shoulder_tension': shoulder_angle,
                'head_engagement': head_tilt,
                'body_openness': arm_openness,
                'overall_posture': self.classify_posture(
                    shoulder_angle, head_tilt, arm_openness
                )
            }
```

### اہم جسمانی زبان کے اشارے

- **کھلا بمقابلہ بند جسمانی حالت**: بازووں کو پار کرنا دفاعی رویے کی نشان دہی کرتا ہے؛ کھلے بازو قبولیت کی نشان دہی کرتے ہیں
- **جھکاؤ**: آگے جھکنا دلچسپی دکھاتا ہے؛ پیچھے جھکنا عدم دلچسپی کی نشان دہی کرتا ہے
- **اشارے**: ہاتھوں کی حرکات دلچسپی اور زور کی نشان دہی کرتی ہیں
- **ذاتی جگہ**: برقرار رکھا گیا فاصلہ آرام کی سطح کا انکشاف کرتا ہے

## 4.5.5 سیاق و سباق کے مطابق جوابات کی تیاری

روبوٹوں کو جذباتی طور پر ذہین اور حالات کے مطابق جوابات پیدا کرنے چاہئیں۔

### جذباتی آگہی کے ساتھ جوابات کی تیاری

```python
class EmotionAwareResponseGenerator:
    def __init__(self, llm_model):
        self.model = llm_model
        self.emotion_strategies = {
            'happy': 'Match enthusiasm, share positive feedback',
            'sad': 'Show empathy, offer support, speak gently',
            'angry': 'Remain calm, acknowledge feelings, de-escalate',
            'anxious': 'Be reassuring, provide clear information',
            'neutral': 'Maintain friendly professional tone'
        }
    
    def generate_response(self, user_input, detected_emotion, context):
        strategy = self.emotion_strategies.get(detected_emotion, 'neutral')
        
        prompt = f"""
        User said: {user_input}
        Detected emotion: {detected_emotion}
        Conversation context: {context}
        Response strategy: {strategy}
        
        Generate an appropriate robot response that:
        1. Acknowledges the user's emotional state
        2. Addresses their request or statement
        3. Maintains appropriate tone and empathy
        """
        
        response = self.model.generate(prompt)
        return response
```

### موافق مواصلاتی حکمت عملیاں

- **عکس بندی**: صارف کے مواصلاتی انداز کی چھوٹی چھوٹی نقل کرنا
- **تصدیق**: مسئلے کے حل سے پہلے جذبات کا اعتراف کرنا
- **مناسب مزاح**: سیاق و سباق کے مطابق مزاح کا استعمال کرنا
- **ہمدردانہ زبان**: "میں سمجھتا ہوں" یا "یہ مشکل ہوگا" جیسے جملے استعمال کرنا

## 4.5.6 روبوٹ کی شخصیت اور رویے کے نمونوں کی ڈیزائننگ

مستقل شخصیت روبوٹوں کو زیادہ قابل فہم اور قابل پیش گوئی بناتی ہے۔

### شخصیت کا فریم ورک

روبوٹوں کے لیے "بگ فائیو" شخصیت کے صفات کو اپنایا جا سکتا ہے:

```python
class RobotPersonality:
    def __init__(self, openness=0.7, conscientiousness=0.8, 
                 extraversion=0.6, agreeableness=0.8, neuroticism=0.2):
        self.traits = {
            'openness': openness,  # Creativity and curiosity
            'conscientiousness': conscientiousness,  # Organization and reliability
            'extraversion': extraversion,  # Sociability and enthusiasm
            'agreeableness': agreeableness,  # Friendliness and cooperation
            'neuroticism': neuroticism  # Emotional stability
        }
        
    def adjust_response_style(self, base_response):
        if self.traits['extraversion'] > 0.7:
            base_response = self.add_enthusiasm(base_response)
        
        if self.traits['agreeableness'] > 0.7:
            base_response = self.add_warmth(base_response)
            
        if self.traits['conscientiousness'] > 0.7:
            base_response = self.add_detail(base_response)
            
        return base_response
```

### رویے کی یکسانی

- **بولنے کے نمونے**: مستحکم لغت اور جملوں کی ساخت
- **جواب دینے کا وقت**: شخصیت کے مطابق قابل پیش گوئی تاخیر
- **اشاروں کا ذخیرہ**: مستحکم طور پر استعمال ہونے والی حرکات کا ایک طے شدہ سیٹ
- **جذباتی حد**: روبوٹ کے کردار کے مطابق جذباتی اظہار کی حدود

## 4.5.7 سماجی معیارات اور ثقافتی مناسبیت کا ماڈلنگ

روبوٹوں کو سماجی تعاملات میں ثقافتی فرق کو سمجھنا اور احترام کرنا چاہیے۔

### ثقافتی اپنانے کا نظام

```python
class CulturalAdaptationModule:
    def __init__(self):
        self.cultural_profiles = {
            'western': {
                'personal_space': 0.5,  # meters
                'eye_contact': 'direct',
                'greeting_style': 'handshake',
                'formality': 'medium'
            },
            'east_asian': {
                'personal_space': 0.7,
                'eye_contact': 'respectful_avoidance',
                'greeting_style': 'bow',
                'formality': 'high'
            },
            'middle_eastern': {
                'personal_space': 0.3,
                'eye_contact': 'direct_same_gender',
                'greeting_style': 'verbal_greeting',
                'formality': 'high'
            }
        }
    
    def adapt_behavior(self, detected_culture, interaction_type):
        profile = self.cultural_profiles.get(detected_culture, 'western')
        
        return {
            'maintain_distance': profile['personal_space'],
            'eye_contact_pattern': profile['eye_contact'],
            'greeting_approach': profile['greeting_style'],
            'formality_level': profile['formality']
        }
```

### عالمی سماجی معیارات

- **باری باری بولنا**: بولنے کے مناسب لمحات کا انتظار کرنا
- **شائستگی**: ادب کے الفاظ اور احترام کی زبان کا استعمال کرنا
- **رازداری**: رپورٹ کے بغیر زیادہ ذاتی سوالات نہ پوچھنا
- **سیاق و سباق کی آگہی**: عوامی بمقابلہ نجی ترتیبات کے لیے رویے کو ایڈجسٹ کرنا

## 4.5.8 طویل مدتی یادداشت کی تعمیر برائے ذاتی تعلقات

ماضی کے تعاملات کو یاد رکھنا گہرے اور زیادہ معنی خیز تعلقات کو ممکن بناتا ہے۔

### طویل مدتی یادداشت کی ساخت

```python
class LongTermMemory:
    def __init__(self):
        self.user_profiles = {}
        self.interaction_history = []
        
    def create_user_profile(self, user_id):
        self.user_profiles[user_id] = {
            'name': None,
            'preferences': {},
            'interests': [],
            'important_dates': {},
            'relationship_level': 'new',
            'conversation_topics': [],
            'emotional_patterns': []
        }
    
    def update_profile(self, user_id, interaction_data):
        profile = self.user_profiles[user_id]
        
        # Extract and store preferences
        if 'preference' in interaction_data:
            profile['preferences'].update(interaction_data['preference'])
        
        # Track conversation topics
        if 'topic' in interaction_data:
            profile['conversation_topics'].append({
                'topic': interaction_data['topic'],
                'timestamp': time.time(),
                'sentiment': interaction_data.get('sentiment', 'neutral')
            })
        
        # Update relationship level based on interaction frequency
        interaction_count = len([i for i in self.interaction_history 
                                if i['user_id'] == user_id])
        
        if interaction_count > 20:
            profile['relationship_level'] = 'close'
        elif interaction_count > 5:
            profile['relationship_level'] = 'familiar'
    
    def recall_relevant_memories(self, user_id, current_context):
        profile = self.user_profiles.get(user_id, {})
        
        relevant_memories = []
        
        # Find related past conversations
        for topic in profile.get('conversation_topics', []):
            if self.is_contextually_relevant(topic, current_context):
                relevant_memories.append(topic)
        
        return relevant_memories
```

### ذاتی بنانے کی حکمت عملیاں

- **نام کا استعمال**: صارفین کو ان کے پسندیدہ نام سے مخاطب کرنا
- **ترجیحات کو یاد رکھنا**: پسندیدگی، ناپسندیدگی، اور روزمرہ کے معمولات کو یاد رکھنا
- **مشتق تاریخ**: ماضی کی گفتگوں کا مناسب حوالہ دینا
- **تعلقات کی ترقی**: آشنائی کے ساتھ ساتھ غیر رسمی بننا

## 4.5.9 فعال سننے اور باری باری بولنے کا نفاذ

فطری گفتگو کے لیے روبوٹ