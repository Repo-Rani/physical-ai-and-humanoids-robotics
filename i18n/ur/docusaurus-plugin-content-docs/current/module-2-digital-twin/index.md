---
id: module-2-digital-twin-index
title: "Module 2: The Digital Twin - Gazebo & Unity (Weeks 6-7)"
sidebar_label: "Module 2: Digital Twin"
sidebar_position: 0
description: "Create physics-accurate simulations for rapid prototyping and testing: Gazebo environment setup, physics simulation, sensor simulation, and Unity integration"
keywords: [gazebo, unity, simulation, physics, robot-simulation, digital-twin]
estimated_time: 10
prerequisites:
  - module-1-ros2-index
  - basic-physics-concepts
learning_outcomes:
  - Set up Gazebo simulation environment with custom world files
  - Configure physics parameters including gravity, collisions, and forces
  - Simulate sensors including LiDAR, cameras, and IMUs with realistic noise models
  - Integrate Unity for high-fidelity visualization and photorealistic rendering
  - Convert URDF robot descriptions to SDF format for Gazebo compatibility
hardware_tier: miniature
---

# ماڈیول 2: ڈیجیٹل ٹوئن - گیزیبو اور یونیٹی (ہفتے 6-7)

یہ ماڈیول گیزیبو اور یونیٹی کا استعمال کرتے ہوئے روبوٹک سسٹمز کے ڈیجیٹل ٹوئنز بنانے پر توجہ مرکوز کرتا ہے۔ آپ سیکھیں گے کہ کیسے فزکس-درست سمولیشنز بنائیں جو روبوٹک الگورتھمز کی تیزی سے پروٹوٹائپنگ اور ٹیسٹنگ کو ممکن بناتی ہیں، انہیں حقیقی ہارڈویئر پر تعینات کرنے سے پہلے۔

## ماڈیول کا جائزہ

ڈیجیٹل ٹوئنز روبوٹکس کی ترقی کے لیے ضروری ہیں، جو آپ کو مہنگے ہارڈویئر کو نقصان پہنچانے کے خطرے سے پہلے ایک محفوظ، کنٹرول شدہ ماحول میں الگورتھمز کی جانچ کرنے کی اجازت دیتے ہیں۔ یہ ماڈیول فزکس سمولیشن کے لیے گیزیبو اور ہائی-فائیڈیلٹی ویژولائزیشن کے لیے یونیٹی دونوں کو کور کرتا ہے، جو آپ کو مختلف سمولیشن ضروریات کے لیے ٹولز فراہم کرتا ہے۔

## ہارڈویئر کی ضروریات

اس ماڈیول کے لیے **مینیچر** ہارڈویئر ٹائر کی ضرورت ہے:
- RTX 3060+ GPU (12GB VRAM کم از کم)
- 32GB RAM
- Ubuntu 22.04
- CUDA 12.1+

کلاؤڈ متبادل: AWS g5.xlarge انسٹنس ($1.50/گھنٹہ) GPU سپورٹ اور Ubuntu 22.04 کے ساتھ۔

## پیش ضروریات

اس ماڈیول کو شروع کرنے سے پہلے، آپ کے پاس ہونا چاہیے:
- ماڈیول 1: ROS 2 فنڈامینٹلز مکمل کیا ہوا
- فزکس کے تصورات کی بنیادی سمجھ (گریویٹی، فورسز، کولیشنز)
- ROS 2 کمیونیکیشن پیٹرنز سے واقفیت

## ماڈیول کی ساخت

یہ ماڈیول سمولیشن کے تصورات کو کور کرنے والے 6 ابواب پر مشتمل ہے:

- **ابواب 2.1**: روبوٹ سمولیشن کا تعارف (ہفتہ 6)
- **ابواب 2.2**: گیزیبو ماحول کی سیٹ اپ (ہفتہ 6)
- **ابواب 2.3**: فزکس سمولیشن: گریویٹی، کولیشنز، فورسز (ہفتہ 6)
- **ابواب 2.4**: سینسر سمولیشن: LiDAR، کیمرے، IMUs (ہفتہ 7)
- **ابواب 2.5**: URDF اور SDF روبوٹ کی وضاحتیں (ہفتہ 7)
- **ابواب 2.6**: ہائی-فائیڈیلٹی ویژولائزیشن کے لیے یونیٹی (ہفتہ 7)

## سیکھنے کا راستہ

اس ماڈیول سے سمولیشن کے ہنر کو پورے نصاب میں استعمال کیا جائے گا، خاص طور پر جب ماڈیول 3 میں پرسیپشن الگورتھمز کی جانچ کی جائے گی اور بعد کے ماڈیولز میں کنٹرول سسٹمز کی توثیق کی جائے گی۔

## اگلے اقدامات

اپنا پہلا ڈیجیٹل ٹوئن بنانے کے لیے تیار ہیں؟ [ابواب 2.1: روبوٹ سمولیشن کا تعارف](./chapter-2-1.md) سے شروع کریں تاکہ روبوٹکس کی ترقی کے لیے سمولیشن کے فوائد اور حدود کو سمجھ سکیں۔