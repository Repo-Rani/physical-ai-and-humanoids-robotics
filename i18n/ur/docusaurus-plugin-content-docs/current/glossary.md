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

# اصطلاحات کی فہرست

یہ فہرست Physical AI & Humanoid Robotics نصاب میں استعمال ہونے والی تکنیکی اصطلاحات، مخففات، اور تصورات کی وضاحت فراہم کرتی ہے۔ اصطلاحات کو حروف تہجی کے مطابق ترتیب دیا گیا ہے اور متعلقہ ابواب کے حوالہ جات دیے گئے ہیں۔

## A

### Action
ROS 2 میں ایک مواصلاتی نمونہ جو طویل مدتی کاموں کے لیے ڈیزائن کیا گیا ہے جو عملدرآمد کے دوران فیڈبیک فراہم کرتے ہیں اور قبل از وقت روکا جا سکتا ہے۔ Actions نیویگیشن، ہیرا پھیری، اور دیگر ہدف محور کاموں کے لیے مثالی ہیں۔ ([باب 1.2](./module-1-ros2/chapter-1-2.md))

### Action Server
ایک ROS 2 نوڈ جو ایکشن پیٹرن کو نافذ کرتا ہے، ہدف قبول کرتا ہے، فیڈبیک فراہم کرتا ہے، اور نتائج واپس کرتا ہے۔ ([باب 1.2](./module-1-ros2/chapter-1-2.md))

### Algorithmic Differentiation (AD)
ایک تکنیک جو کمپیوٹر پروگراموں کے ذریعہ بیان کردہ فنکشنز کے مشتق کی حساب لگانے کے لیے استعمال ہوتی ہے، جو روبوٹکس میں ٹریجیکٹری آپٹیمائزیشن اور کنٹرول کے لیے استعمال ہوتی ہے۔ ([باب 3.5](./module-3-isaac/chapter-3-5.md))

### Apache License 2.0
ایک اوپن سورس سافٹ ویئر لائسنس جو صارفین کو سافٹ ویئر کو آزادانہ طور پر استعمال کرنے، ترمیم کرنے، اور تقسیم کرنے کی اجازت دیتا ہے جبکہ کاپی رائٹ نوٹس اور انکارنامے کو برقرار رکھتا ہے۔ ROS 2 اور کئی روبوٹکس فریم ورکس کے ذریعہ استعمال ہوتا ہے۔ ([باب 1.3](./module-1-ros2/chapter-1-3.md))

### Application Programming Interface (API)
سافٹ ویئر ایپلی کیشنز کی تعمیر اور ان کے ساتھ تعامل کے لیے قواعد اور پروٹوکولز کا ایک سیٹ۔ روبوٹکس میں، APIs مختلف سافٹ ویئر اجزاء کے درمیان مواصلات کی اجازت دیتے ہیں۔ ([باب 4.4](./module-4-vla/chapter-4-4.md))

### Artificial Intelligence (AI)
ماشینوں، خاص طور پر کمپیوٹر سسٹمز کے ذریعہ انسانی ذہانت کے عمل کی نقل۔ روبوٹکس میں، AI ادراک، فیصلہ سازی، اور سیکھنے کی صلاحیتوں کو فعال بناتی ہے۔ ([باب 0.3](./module-0-getting-started/chapter-0-3.md))

### Asynchronous Programming
ایک پروگرامنگ پیراڈائم جو متعدد آپریشنز کو ایک ساتھ چلانے کی اجازت دیتا ہے بغیر اصلی عملدرآمد تھریڈ کو روکے۔ ردعملی روبوٹک سسٹمز کے لیے ضروری۔ ([باب 1.2](./module-1-ros2/chapter-1-2.md))

## B

### Behavior Tree
روبوٹکس میں پیچیدہ رویوں کو منظم کرنے اور عملدرآمد کرنے کے لیے استعمال ہونے والی ایک ہیرارکیکل ساخت۔ Behavior trees روبوٹ کنٹرول اور فیصلہ سازی کے لیے ایک ماڈیولر نقطہ نظر فراہم کرتے ہیں۔ ([باب 3.6](./module-3-isaac/chapter-3-6.md))

### Best Effort (QoS Policy)
ROS 2 میں ایک Quality of Service پالیسی جہاں پیغامات کھو سکتے ہیں بغیر کسی دوبارہ کوشش کے۔ سینسر ڈیٹا کے لیے موزوں جہاں کچھ نقصان قابل قبول ہے۔ ([باب 1.1](./module-1-ros2/chapter-1-1.md))

### Bidirectional Encoder Representations from Transformers (BERT)
نچلی سطح کی زبان کی پروسیسنگ کے لیے ایک ٹرانسفارمر پر مبنی مشین لرننگ تکنیک۔ روبوٹکس ایپلی کیشنز میں زبان کی سمجھ کے لیے استعمال ہوتا ہے۔ ([باب 4.3](./module-4-vla/chapter-4-3.md))

### Bridge
ایک جزو جو دو مختلف مواصلاتی سسٹمز یا پروٹوکولز کو جوڑتا ہے۔ روبوٹکس میں، برجز اکثر ROS 2 کو دیگر سمولیشن یا ہارڈویئر پلیٹ فارمز سے جوڑتے ہیں۔ ([باب 2.5](./module-2-digital-twin/chapter-2-5.md))

### Build System
سافٹ ویئر ٹولز جو سورس کوڈ کو قابل عمل پروگراموں میں کمپائل کرنے کے عمل کو خودکار بناتے ہیں۔ ROS 2 میں، Colcon بنیادی بلڈ سسٹم ہے۔ ([باب 1.3](./module-1-ros2/chapter-1-3.md))

## C

### Cartesian Space
X، Y، اور Z کوآرڈینیٹس کے ذریعہ بیان کردہ تین جملہ فضا، جو روبوٹکس میں پوزیشنز اور حرکات کی وضاحت کے لیے استعمال ہوتی ہے۔ ([باب 3.6](./module-3-isaac/chapter-3-6.md))

### Chain of Thought (CoT)
ایک استدلال کا نقطہ نظر جہاں AI ماڈلز پیچیدہ مسائل کو حل کرنے کے لیے درمیانی استدلال کے اقدامات پیدا کرتے ہیں۔ روبوٹکس میں ٹاسک پلاننگ اور فیصلہ سازی کے لیے استعمال ہوتا ہے۔ ([باب 4.3](./module-4-vla/chapter-4-3.md))

### Collision Detection
کمپیوٹیشنل مسئلہ کہ آیا دو یا زیادہ اشیاء ایک دوسرے کو کاٹ رہی ہیں۔ محفوظ روبوٹ نیویگیشن اور ہیرا پھیری کے لیے ضروری۔ ([باب 2.3](./module-2-digital-twin/chapter-2-3.md))

### Command Line Interface (CLI)
کمپیوٹر پروگرامز یا آپریٹنگ سسٹمز کے ساتھ تعامل کے لیے ایک متن پر مبنی انٹرفیس۔ روبوٹکس ترقی اور سسٹم انتظام کے لیے ضروری۔ ([باب 0.2](./module-0-getting-started/chapter-0-2.md))

### Colcon
ROS 2 میں استعمال ہونے والا بلڈ سسٹم جو پیکیجز کی تعمیر اور انسٹالیشن کے لیے استعمال ہوتا ہے۔ Colcon متعدد بلڈ سسٹمز اور پیکیج قسموں کی حمایت کرتا ہے۔ ([باب 1.3](./module-1-ros2/chapter-1-3.md))

### Computational Graph
نوڈز اور ٹاپکس کا نیٹ ورک جو ROS سسٹم میں تمام عملدرآمد اور مواصلاتی چینلز کی نمائندگی کرتا ہے۔ ([باب 1.1](./module-1-ros2/chapter-1-1.md))

### Computer Vision
مصنوعی ذہانت کا ایک میدان جو کمپیوٹرز کو بصری دنیا کی تشریح اور سمجھنے کے لیے تربیت دیتا ہے۔ روبوٹکس میں ادراک اور نیویگیشن کے لیے وسیع پیمانے پر استعمال ہوتا ہے۔ ([باب 3.3](./module-3-isaac/chapter-3-3.md))

### Continuous Joint
URDF میں جوائنٹ کی ایک قسم جو ایک واحد محور کے گرد لامتناہی گھومنے کی اجازت دیتا ہے، جیسے کہ ایک پہیہ یا مسلسل گھومنے والا سرو۔ ([باب 1.4](./module-1-ros2/chapter-1-4.md))

### Control Theory
انجینئرنگ اور ریاضی کی ایک بین الضابطہ شاخ جو انپٹس کے ساتھ ڈائنامیکل سسٹمز کے رویے سے نمٹتی ہے، اور ان کے رویے کو فیڈبیک کے ذریعہ کیسے تبدیل کیا جاتا ہے۔ روبوٹکس کے لیے بنیادی۔ ([باب 5.6](./module-5-capstone/chapter-5-6.md))

### Convolutional Neural Network (CNN)
گہری نیورل نیٹ ورکس کی ایک کلاس جو بصری امیجری کی تجزیہ کے لیے عام طور پر لاگو کی جاتی ہے۔ روبوٹکس میں آبجیکٹ ڈٹیکشن اور منظر کی سمجھ کے لیے وسیع پیمانے پر استعمال ہوتا ہے۔ ([باب 3.3](./module-3-isaac/chapter-3-3.md))

## D

### Data Distribution Service (DDS)
ریل ٹائم، قابل توسیع، اور قابل اعتماد مواصلات کے لیے ایک مڈل ویئر معیار جو ROS 2 کی بنیاد بناتا ہے۔ ([باب 1.1](./module-1-ros2/chapter-1-1.md))

###