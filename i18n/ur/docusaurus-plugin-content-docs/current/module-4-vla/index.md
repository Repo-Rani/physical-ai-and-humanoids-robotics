---
id: module-4-vla-index
title: "Module 4: Vision-Language-Action Models (Weeks 11-12)"
sidebar_label: "Module 4: VLA Models"
sidebar_position: 0
description: "Enable natural language control and multimodal interaction using vision-language-action models: OpenAI integration, cognitive planning, and multi-modal robotics"
keywords: [vla-models, vision-language-action, openai, whisper, gpt, multimodal-robotics, cognitive-planning]
estimated_time: 10
prerequisites:
  - module-3-isaac-index
  - basic-nlp-concepts
learning_outcomes:
  - Integrate OpenAI Whisper for voice-to-action command processing
  - Implement cognitive planning using language models for task decomposition
  - Build conversational robotics interfaces with safety filtering
  - Fuse multiple modalities (speech, gesture, vision) for human-robot interaction
  - Apply chain-of-thought prompting for complex robotics tasks
hardware_tier: premium
---

# ماڈیول 4: ویژن-لینگویج-ایکشن ماڈلز (ہفتے 11-12)

یہ ماڈیول روبوٹکس کی جدید ترین ٹیکنالوجی پر بحث کرتا ہے: ویژن-لینگویج-ایکشن (VLA) ماڈلز جو قدرتی زبان کے کنٹرول اور ملٹی ماڈل انٹریکشن کو ممکن بناتے ہیں۔ آپ سیکھیں گے کہ بڑے زبان کے ماڈلز کو روبوٹک سسٹمز کے ساتھ کیسے مربوط کیا جائے، بات چیت کے انٹرفیس بنائے جائیں جو انسانی ارادوں اور روبوٹک ایکشنز کے درمیان پل کا کام کریں۔

## ماڈیول کا جائزہ

VLA ماڈلز مصنوعی ذہانت اور روبوٹکس کے امتزاج کی نمائندگی کرتے ہیں، جو روبوٹس کو قدرتی زبان میں اظہار کیے گئے پیچیدہ احکامات کو سمجھنے اور ان پر عمل کرنے کی صلاحیت دیتے ہیں۔ یہ ماڈیول LLMs کو روبوٹک سسٹمز کے ساتھ مربوط کرنے پر محیط ہے، بنیادی آواز کے احکامات کی پروسیسنگ سے لے کر پیچیدہ علمی منصوبہ بندی تک۔

## ہارڈویئر کی ضروریات

اس ماڈیول کے لیے **پریمیئم** ہارڈویئر ٹائر کی ضرورت ہے:
- فزیکل AI ایج کٹ (Jetson Orin Nano + RealSense D435i + مائیکروفون)
- یا کلاؤڈ GPU رسائی (AWS g5.xlarge for inference)
- RealSense D435i ڈیپتھ کیمرا
- آواز کے ان پٹ کے لیے USB مائیکروفون

## پیش ضروریات

اس ماڈیول کو شروع کرنے سے پہلے، آپ کے پاس ہونا چاہیے:
- ماڈیول 3: NVIDIA Isaac مکمل کیا ہوا
- قدرتی زبان کی پروسیسنگ کے تصورات کی بنیادی سمجھ
- ROS 2 کے ساتھ بیرونی APIs کی مربوط کرنے کی واقفیت

## ماڈیول کی ساخت

یہ ماڈیول VLA تصورات پر محیط 5 ابواب پر مشتمل ہے:

- **ابواب 4.1**: LLMs اور روبوٹکس کا امتزاج (ہفتہ 11)
- **ابواب 4.2**: OpenAI Whisper کے ساتھ آواز سے ایکشن (ہفتہ 11)
- **ابواب 4.3**: زبان کے ماڈلز کے ساتھ علمی منصوبہ بندی (ہفتہ 11)
- **ابواب 4.4**: بات چیت کے روبوٹکس کے لیے GPT ماڈلز کی مربوط کرنا (ہفتہ 12)
- **ابواب 4.5**: ملٹی ماڈل انٹریکشن: گفتگو، اشارہ، ویژن (ہفتہ 12)

## سیکھنے کا راستہ

اس ماڈیول میں سیکھی گئی VLA صلاحیتوں کو ماڈیول 5 کے کپ اسٹون پراجیکٹ میں مربوط کیا جائے گا، جہاں آپ ایک مکمل خودکار انسانی نما روبوٹ بنائیں گے جو قدرتی زبان کے انٹریکشن کی صلاحیت رکھتا ہو۔

## اگلے اقدامات

کیا آپ اپنے روبوٹ کو انسانی زبان سمجھنے کے لیے تیار ہیں؟ شروع کریں [ابواب 4.1: LLMs اور روبوٹکس کا امتزاج](./chapter-4-1.md) سے تاکہ سمجھ سکیں کہ LLMs روبوٹکس ایپلی کیشنز کو کیسے تبدیل کر رہے ہیں۔