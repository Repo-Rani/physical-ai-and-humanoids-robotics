---
id: module-3-isaac-index
title: "Module 3: The AI-Robot Brain - NVIDIA Isaac (Weeks 8-10)"
sidebar_label: "Module 3: NVIDIA Isaac"
sidebar_position: 0
description: "Implement GPU-accelerated perception, navigation, and learning using NVIDIA Isaac platform: Isaac Sim, Isaac ROS, VSLAM, Nav2, and reinforcement learning"
keywords: [nvidia-isaac, isaac-sim, isaac-ros, vslam, nav2, reinforcement-learning, gpu-acceleration]
estimated_time: 10
prerequisites:
  - module-2-digital-twin-index
  - basic-machine-learning
learning_outcomes:
  - Launch and configure Isaac Sim for photorealistic simulation
  - Implement GPU-accelerated VSLAM using NVIDIA NVSLAM pipeline
  - Configure Nav2 for path planning with custom planners for bipedal movement
  - Train robot control policies using Isaac Gym reinforcement learning
  - Apply domain randomization techniques for sim-to-real transfer
hardware_tier: miniature
---

# ماڈیول 3: AI-روبوٹ برین - NVIDIA Isaac (ہفتے 8-10)

یہ ماڈیول آپ کو NVIDIA Isaac سے متعارف کراتا ہے، جو روبوٹک سسٹمز کے لیے GPU-تیز رفتار پرسپشن، نیویگیشن، اور سیکھنے کے لیے پلیٹ فارم ہے۔ آپ فوٹوریلسٹک سمولیشن کے لیے Isaac Sim، ہارڈویئر-تیز رفتار پرسپشن کے لیے Isaac ROS، اور سمولیشن سے حقیقت تک سیکھنے کے منتقل کرنے کی تکنیکوں کا جائزہ لیں گے۔

## ماڈیول کا جائزہ

NVIDIA Isaac AI-روبوٹکس انٹیگریشن کی جدید ترین مثال ہے، جو بصری SLAM، راہ کی منصوبہ بندی، اور تقویت سیکھنے جیسے حساباً گراں کاموں کے لیے GPU تیز رفتار کا استعمال کرتا ہے۔ یہ ماڈیول پورے Isaac ایکو سسٹم کو کور کرتا ہے اور ظاہر کرتا ہے کہ اسے پیچیدہ روبوٹکس مسائل پر کیسے لاگو کیا جائے۔

## ہارڈویئر کی ضروریات

اس ماڈیول کے لیے **Miniature** ہارڈویئر ٹائر کی ضرورت ہے:
- RTX 3060+ GPU (12GB VRAM کم از کم)
- 32GB RAM
- Ubuntu 22.04
- CUDA 12.1+
- NVIDIA Isaac Sim 2023.1.1+

کلاؤڈ متبادل: AWS g5.2xlarge انسٹنس ($3.00/گھنٹہ) GPU سپورٹ اور Isaac Sim پیش نصب کے ساتھ۔

## پیش شرطیں

اس ماڈیول کو شروع کرنے سے پہلے، آپ کے پاس ہونا چاہیے:
- ماڈیول 2: ڈجیٹل ٹوئن مکمل کیا ہوا
- مشین لرننگ کے تصورات کی بنیادی سمجھ
- ROS 2 اور سمولیشن ماحول سے واقفیت

## ماڈیول کی ساخت

یہ ماڈیول Isaac پلیٹ فارم کے تصورات کو کور کرنے والے 6 ابواب پر مشتمل ہے:

- **ابواب 3.1**: NVIDIA Isaac پلیٹ فارم کا جائزہ (ہفتہ 8)
- **ابواب 3.2**: Isaac Sim: فوٹوریلسٹک سمولیشن (ہفتہ 8)
- **ابواب 3.3**: Isaac ROS: ہارڈویئر-تیز رفتار VSLAM (ہفتہ 9)
- **ابواب 3.4**: Nav2: دو پاؤں حرکت کے لیے راہ کی منصوبہ بندی (ہفتہ 9)
- **ابواب 3.5**: روبوٹ کنٹرول کے لیے تقویت سیکھنا (ہفتہ 10)
- **ابواب 3.6**: سمولیشن سے حقیقت تک منتقل کرنے کی تکنیکیں (ہفتہ 10)

## سیکھنے کا راستہ

اس ماڈیول سے پرسپشن اور کنٹرول کی تکنیکیں ماڈیولز 4 اور 5 میں اعلیٰ روبوٹکس ایپلی کیشنز کے لیے ضروری ہوں گی، جہاں آپ AI کی صلاحیتوں کو جسمانی سسٹمز کے ساتھ مربوط کریں گے۔

## اگلے اقدامات

کیا آپ AI کے ساتھ اپنی روبوٹکس کو تیز کرنے کے لیے تیار ہیں؟ [ابواب 3.1: NVIDIA Isaac پلیٹ فارم کا جائزہ](./chapter-3-1.md) سے شروع کریں تاکہ Isaac ایکو سسٹم اور یہ ROS 2 کے ساتھ کیسے مربوط ہوتا ہے کو سمجھ سکیں۔