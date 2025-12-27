---
id: module-1-ros2-index
title: "Module 1: The Robotic Nervous System - ROS 2 (Weeks 3-5)"
sidebar_label: "Module 1: ROS 2 Fundamentals"
sidebar_position: 0
description: "Master ROS 2 fundamentals for building distributed robotic systems: architecture, nodes, topics, services, URDF, and best practices"
keywords: [ros2, robotics, middleware, dds, qos, nodes, topics, services, urdf]
estimated_time: 10
prerequisites:
  - module-0-getting-started-index
  - basic-linux-command-line
learning_outcomes:
  - Implement publisher-subscriber patterns for sensor data streaming
  - Create services for synchronous robot configuration requests
  - Build action servers for preemptible long-running tasks
  - Design robot structure using URDF and Xacro macros
  - Apply ROS 2 best practices for debugging and performance optimization
hardware_tier: proxy
---

# ماڈیول 1: روبوٹک عصبی نظام - ROS 2 (ہفتے 3-5)

یہ ماڈیول آپ کو ROS 2 (روبوٹ آپریٹنگ سسٹم 2) سے متعارف کراتا ہے، جو تقسیم شدہ روبوٹک سسٹمز کے لیے عصبی نظام کے طور پر کام کرنے والے مڈل ویئر ہے۔ آپ ROS 2 آرکیٹیکچر، کمیونیکیشن پیٹرنز کے بنیادی تصورات سیکھیں گے، اور کیسے مضبوط روبوٹک ایپلی کیشنز بنائی جائیں۔

## ماڈیول کا جائزہ

ROS 2 تقریباً تمام جدید روبوٹکس ترقی کی بنیاد ہے۔ یہ ماڈیول بنیادی آرکیٹیکچر کے تصورات سے لے کر اعلیٰ سطحی پیکج بنانے اور ڈیباگنگ تکنیکوں تک سب کچھ شامل کرتا ہے۔ اس ماڈیول کے اختتام تک، آپ مناسب کمیونیکیشن پیٹرنز اور ڈیباگنگ طریقوں کے ساتھ ملٹی نوڈ ROS 2 ایپلی کیشنز بنانے کے قابل ہوں گے۔

## ہارڈویئر کی ضروریات

یہ ماڈیول **Proxy** ہارڈویئر ٹائر (صرف سیمیولیشن) کے ساتھ مکمل کیا جا سکتا ہے:
- 16GB RAM، Quad-core CPU
- Ubuntu 22.04 (یا مساوی Linux تقسیم)
- ROS 2 Humble Hawksbill

کلاؤڈ متبادل: AWS t3.xlarge انسٹنس ($0.17/گھنٹہ) Ubuntu 22.04 اور ROS 2 Humble کے ساتھ پہلے سے انسٹال شدہ۔

## پیش ضروریات

اس ماڈیول کو شروع کرنے سے پہلے، آپ کے پاس ہونا چاہیے:
- ماڈیول 0: شروع کرنا مکمل کر لیا ہو
- Linux کمانڈ لائن کی بنیادی واقفیت
- Python پروگرامنگ کا تجربہ (یا سیکھنے کی خواہش)

## ماڈیول کی ساخت

یہ ماڈیول ROS 2 کے بنیادی تصورات پر مشتمل 6 ابواب پر مشتمل ہے:

- **ابواب 1.1**: ROS 2 آرکیٹیکچر اور بنیادی تصورات (ہفتہ 3)
- **ابواب 1.2**: نوڈز، ٹاپکس، سروسز، اور ایکشنز (ہفتہ 3)
- **ابواب 1.3**: Python کے ساتھ ROS 2 پیکجز بنانا (ہفتہ 4)
- **ابواب 1.4**: URDF: روبوٹ کی ساخت کا بیان (ہفتہ 4)
- **ابواب 1.5**: لانچ فائلز اور پیرامیٹر مینجمنٹ (ہفتہ 5)
- **ابواب 1.6**: ROS 2 کے بہترین طریقے اور ڈیباگنگ (ہفتہ 5)

## سیکھنے کا راستہ

یہ ماڈیول تمام بعد کے ماڈیولز کے لیے بنیاد قائم کرتا ہے۔ آپ نصاب کے دوران ROS 2 کے تصورات کا استعمال کریں گے جب آپ پرسیپشن، نیویگیشن، اور کنٹرول سسٹمز کو نافذ کریں گے۔

## اگلے اقدامات

ROS 2 میں گہرائی سے جانے کے لیے تیار ہیں؟ [ابواب 1.1: ROS 2 آرکیٹیکچر اور بنیادی تصورات](./chapter-1-1.md) سے شروع کریں تاکہ DDS مڈل ویئر اور کمپیوٹیشنل گراف کو سمجھ سکیں جو تمام ROS 2 سسٹمز کی بنیاد ہیں۔