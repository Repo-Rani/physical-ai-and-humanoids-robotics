---
id: module-4-chapter-1
title: "Introduction to Vision-Language-Action Models"
sidebar_label: "4.1 VLA Models Overview"
sidebar_position: 1
description: "Introduction to Vision-Language-Action (VLA) models: multimodal AI for robotics, transformer architectures, and embodied intelligence for humanoid control"
keywords: [vla-models, vision-language-action, multimodal-ai, transformers, embodied-ai]
estimated_time: 75
prerequisites:
  - module-3-chapter-6
  - basic-deep-learning
learning_outcomes:
  - Understand Vision-Language-Action model architectures
  - Compare VLA models with traditional robotics pipelines
  - Identify when to use VLA vs. classical methods
  - Analyze multimodal transformer attention mechanisms
  - Evaluate VLA model capabilities and limitations
hardware_tier: premium
---

# باب 4.1: وژن-لینگویج-ایکشن ماڈلز کا تعارف

وژن-لینگویج-ایکشن (VLA) ماڈلز روبوٹکس میں ایک نیا انداز پیش کرتے ہیں، جو ایک متحدہ نیورل نیٹ ورک میں ادراک، سمجھ اور عمل کی پیدائش کو یکجا کرتے ہیں۔ یہ باب VLA تصورات اور ان کے ہیومینائڈ روبوٹکس پر اطلاق کا تعارف پیش کرتا ہے۔

## روبوٹکس میں VLA انقلاب

روایتی روبوٹکس پائپ لائنز ادراک، منصوبہ بندی، اور کنٹرول کو الگ الگ ماڈلز میں تقسیم کرتی ہیں۔ وژن-لینگویج-ایکشن ماڈلز ان اجزاء کو ایک ہی اینڈ-ٹو-اینڈ نیورل نیٹ ورک میں یکجا کرتے ہیں جو براہ راست سنسری ان پٹس اور قدرتی زبان سے روبوٹ کے اقدامات کی نقشہ سازی کرتے ہیں۔

### روایتی پائپ لائن بمقابلہ VLA

**روایتی سنس-پلان-ایکٹ:**
```
Camera → Object Detection → Pose Estimation → Motion Planning → Trajectory Execution
  ↓          ↓                   ↓                 ↓                  ↓
Hard-coded → Separate models → Symbolic planner → Control theory → Motor commands
```

**VLA طریقہ کار:**
```
Camera + Language → Unified Transformer → Action Tokens → Motor commands
         ↓                  ↓                    ↓
   Embeddings       Attention layers    Direct output
```

### کب VLA بمقابلہ کلاسیکی طریقوں کا استعمال کریں

| معیار | VLA ماڈلز استعمال کریں | کلاسیکی طریقے استعمال کریں |
|-----------|----------------|----------------------|
| کام کی پیچیدگی | متنوع، کھلے سرے کے کام | خوب تعریف شدہ، ساختہ بند کام |
| ڈیٹا کی دستیابی | بڑے ڈیٹا سیٹس دستیاب | محدود ڈومین مخصوص ڈیٹا |
| عمومی بنانا | زیرو-شاٹ ٹرانسفر کی ضرورت | ڈومین مخصوص بہتری |
| تشریح پذیری | بلیک باکس قبول کر سکتا ہے | وضاحت کی ضرورت ہے |
| کمپیوٹیشنل وسائل | GPU کلسٹرز دستیاب | ایمبیڈڈ/رسورسز محدود |
| حفاظت کی اہمیت | معتدل (سہولیات کے ساتھ) | زیادہ (فارملی تصدیق کی ضرورت) |

## ملٹی موڈل ٹرانسفورمر آرکیٹیکچر

VLA ماڈلز ٹرانسفورمر آرکیٹیکچرز کو وسعت دیتے ہیں تاکہ وژن، زبان، اور ایکشن موڈز کو ایک ساتھ ہینڈل کیا جا سکے۔

### بنیادی اجزاء

```python
# Conceptual VLA architecture
class VLAModel:
    def __init__(self):
        self.vision_encoder = VisionTransformer()  # Process images
        self.language_encoder = TextTransformer()   # Process instructions
        self.fusion_transformer = MultimodalTransformer()  # Combine modalities
        self.action_decoder = ActionHead()  # Generate actions

    def forward(self, images, text, robot_state):
        # Encode visual observations
        vision_tokens = self.vision_encoder(images)  # [B, N_vis, D]

        # Encode language instructions
        language_tokens = self.language_encoder(text)  # [B, N_lang, D]

        # Encode robot proprioception
        state_tokens = self.encode_state(robot_state)  # [B, N_state, D]

        # Fuse all modalities with attention
        fused = self.fusion_transformer(
            torch.cat([vision_tokens, language_tokens, state_tokens], dim=1)
        )

        # Generate action sequence
        actions = self.action_decoder(fused)  # [B, N_actions, action_dim]
        return actions
```

### ملٹی موڈل سیاق میں ایٹنشن میکانزم

VLA ماڈلز کراس-ایٹنشن کا استعمال کرتے ہیں تاکہ موڈز کے درمیان معلومات کو ہمراہ کیا جا سکے:

**وژن-لینگویج ایٹنشن:**
- زبان کے ٹوکنز متعلقہ تصویر کے علاقوں پر توجہ دیتے ہیں
- مثال: "سرخ کپ اٹھاؤ" → توجہ سرخ اشیاء پر مرکوز ہوتی ہے

**لینگویج-ایکشن ایٹنشن:**
- ایکشن ٹوکنز ہدایات کے معنویات پر مشروط ہوتے ہیں
- مثال: "ہلکے سے" → ایکشن پیرامیٹرز میں کم فورس/ویلوسیٹی

**ٹیمپورل ایٹنشن:**
- موجودہ اقدامات پہلے کے مشاہدات پر منحصر ہوتے ہیں
- کثیر مرحلہ کاموں کے لیے سلسلہ وار استدلال کو فعال کرتا ہے

## ٹوکنیزیشن کی حکمت عملیاں

### وژن ٹوکنیزیشن

**پیچ-بیسڈ (ViT-اسٹائل):**
```python
def tokenize_image(image, patch_size=16):
    # Image: [3, H, W]
    patches = rearrange(
        image,
        'c (h p1) (w p2) -> (h w) (p1 p2 c)',
        p1=patch_size, p2=patch_size
    )
    # Patches: [N_patches, patch_dim]
    return patches
```

**CNN فیچرز:**
- ResNet/EfficientNet سے فیچرز نکالیں
- فضائی جملوں کو ٹوکن سیکوئنس میں تبدیل کریں

**پکسل-لیول (ہائی ریزولیوشن کے لیے):**
- براہ راست پکسلز کو وژن ٹرانسفورمرز کے ساتھ پروسیس کریں
- کمپیوٹیشنل مہنگا لیکن تفصیل برقرار رکھتا ہے

### زبان کی ٹوکنیزیشن

**سب ورڈ ٹوکنیزیشن (BPE/SentencePiece):**
```python
from transformers import AutoTokenizer

tokenizer = AutoTokenizer.from_pretrained("gpt2")
text = "Pick up the red cube and place it on the table"
tokens = tokenizer.encode(text, return_tensors="pt")
# tokens: [1, 13] -> token IDs
```

**ہدایت کی کوڈنگ:**
- قدرتی زبان کے احکامات → ایمبیڈنگ اسپیس
- گراؤنڈنگ کے لیے معنوی معنی برقرار رکھتا ہے

### ایکشن ٹوکنیزیشن

**مسلسل اقدامات (ڈسکرٹائزیشن):**
```python
def tokenize_actions(actions, bins=256):
    # actions: [T, action_dim] (continuous)
    # Normalize to [-1, 1]
    normalized = (actions - action_min) / (action_max - action_min) * 2 - 1
    # Discretize into bins
    tokens = ((normalized + 1) / 2 * bins).long().clamp(0, bins-1)
    return tokens  # [T, action_dim] (discrete)
```

**گریپر اسٹیٹ (بائنری):**
- کھلا/بند کو ڈسکرٹ ٹوکنز کے طور پر
- مسلسل جوائنٹ پوزیشنز کے ساتھ ملایا جاتا ہے

**ایکشن سیکوئنسز:**
- کثیر مرحلہ منصوبوں کو ٹوکن سیکوئنسز کے طور پر پیش کریں
- خودکار ایکشن جنریشن کو فعال کرتا ہے

## اہم VLA ماڈل آرکیٹیکچرز

### RT-1 (روبوٹکس ٹرانسفورمر 1)

**آرکیٹیکچر:**
- وژن: EfficientNet-B3 بیک بون
- فیوژن: ٹوکن لرنر + ٹرانسفورمر
- آؤٹ پٹ: ڈسکرٹائزڈ 7-DOF اقدامات (256 بنز)

**ٹریننگ:**
- ڈیٹا سیٹ: 130k روبوٹ ڈیمونسٹریشنز
- کام: پک اینڈ پلیس، ڈرائر کھولنا، سطحوں کو صاف کرنا
- عمومی بنانا: دیکھی گئی کاموں میں 97% کامیابی، نئی کاموں میں 76%

**کلیدی ایجاد:** FiLM (فیچر-وائز لینیئر موڈولیشن) زبان پر مشروط

```python
# Simplified RT-1 concept
class RT1(nn.Module):
    def forward(self, image, instruction):
        # Extract visual features
        visual_features = self.efficientnet(image)  # [B, C, H, W]

        # Token learner: reduce spatial tokens
        visual_tokens = self.token_learner(visual_features)  # [B, 8, D]

        # Language conditioning
        lang_embedding = self.language_encoder(instruction)  # [B, D]

        # FiLM conditioning
        gamma, beta = self.film_generator(lang_embedding)
        conditioned = gamma * visual_tokens + beta

        # Transformer reasoning
        output_tokens = self.transformer(conditioned)

        # Decode to discretized actions
        action_tokens = self.action_head(output_tokens)  # [B, 7, 256]
        return action_tokens
```

### RT-2 (روبوٹکس ٹرانسفورمر 2)

**آرکیٹیکچر:**
- فاؤنڈیشن: PaLI-X وژن-لینگویج ماڈل
- فائن ٹوننگ: روبوٹ ایکشن ٹوکنز کو ووکبیولری میں شامل کیا گیا
- آؤٹ پٹ: زبان اور اقدامات دونوں

**ٹریننگ:**
- پری-ٹریننگ: ویب-اسکیل وژن-لینگویج ڈیٹا
- فائن ٹوننگ: روبوٹ ڈیمونسٹریشنز + VQA کے ساتھ کو-ٹریننگ

**کلیدی ایجاد:** اقدامات کو زبان کے ٹوکنز کے طور پر سمجھنا بڑے پیمانے پر پری-ٹریننگ کو فعال کرتا ہے

**کارکردگی:**
- RT-1 سے 3 گنا بہتر عمومی بنانا
- ابھرتے ہوئے صلاحیتیں: اشیاء کے خواص، فضائی تعلقات پر استدلال

### PaLM-E (ایمبوڈڈ ملٹی موڈل لینگویج ماڈل)

**آرکیٹیکچر:**
- زبان: PaLM (540B پیرامیٹر لینگویج ماڈل)
- وژن: ViT-22B
- فیوژن: انٹرلیوڈ تصویر اور ٹیکسٹ ٹوکنز

**صلاحیتیں:**
- قدرتی زبان سے ہائی لیول ٹاسک پلاننگ
- روبوٹ ماحول کے بارے میں بصری سوالوں کے جوابات
- منیپولیشن کے لیے چین-آف-تھاٹ ریزننگ

**مثال:**
```
Human: "I spilled my coffee, can you help?"
PaLM-E: [observes scene] "I see spilled liquid on the table.
         I'll need to: 1) Get a paper towel, 2) Wipe the spill,
         3) Dispose of the towel. Let me start by navigating to
         the paper towel dispenser."
```

### OpenVLA (اوپن سورس VLA)

**آرکیٹیکچر:**
- وژن: DINOv2 یا SigLIP
- زبان: Llama 2/3
- فیوژن: کراس-ایٹنشن لیئرز
- آؤٹ پٹ: مسلسل اقدامات (ڈفیوژن پالیسی کے ذریعے)

**ٹریننگ:**
- ڈیٹا سیٹ: Open X-Embodiment (1M+ ٹریجیکٹریز، 22 روبوٹ ٹائپس)
- پری-ٹریننگ: وژن-لینگویج ہمراہی
- فائن ٹوننگ: روبوٹ مخصوص ڈیٹا سیٹس

**فوائد:**
- اوپن سورس اور قابل ترمیم
- کثیر روبوٹ مورفولوجیز کی حمایت کرتا ہے
- Open X-Embodiment بینچ مارکس پر جدید ترین

```python
# Using OpenVLA (conceptual)
from openvla import OpenVLA

model = OpenVLA.from_pretrained("openvla-7b")

image = camera.get_rgb()  # [H, W, 3]
instruction = "pick up the apple"
robot_state = get_joint_positions()  # [7]

action = model.predict_action(
    image=image,
    instruction=instruction,
    proprio=robot_state
)
# action: [7] continuous joint velocities
```

## ایمبوڈڈ AI اور گراؤنڈنگ

**ایمبوڈڈ انٹلیجنس:** AI ایجنٹس جو دنیا کے ساتھ جسمانی تعامل کے ذریعے سیکھتے ہیں۔

### سمبل گراؤنڈنگ