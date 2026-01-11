# urdu-translation-skill ⭐

## Description

Provides reusable translation guidance for converting technical educational content into Urdu.

## Components

### Technical Terminology Preservation
- Identify terms that should remain in English
- Create Urdu equivalents for translatable technical terms
- Maintain consistency across all content
- Build and maintain a technical glossary
- Handle acronyms and abbreviations appropriately

### Educational Tone Control
- Use student-friendly language appropriate for educational context
- Balance formal and accessible tone
- Adapt complexity for different age groups
- Maintain encouraging and supportive voice
- Use culturally relevant examples and analogies

### Language Consistency Rules
- Establish translation conventions and style guide
- Ensure consistent terminology across chapters
- Maintain consistent grammar and syntax patterns
- Follow Urdu typography and punctuation standards
- Preserve formatting and structure from original

## Responsibilities

### Translate Content Clearly and Accurately

**Translation Principles:**

1. **Clarity First:**
   - Prioritize reader comprehension over literal translation
   - Use simple, direct sentences
   - Break complex sentences into multiple simpler ones
   - Avoid ambiguous constructions

2. **Accuracy:**
   - Preserve exact meaning of technical concepts
   - Verify translations with subject matter experts
   - Cross-reference with established Urdu technical literature
   - Maintain numerical accuracy (values, formulas, measurements)

3. **Readability:**
   - Use natural Urdu sentence structure (SOV - Subject-Object-Verb)
   - Avoid awkward word-for-word translations
   - Employ appropriate Urdu idioms and expressions
   - Ensure smooth flow between sentences

**Translation Process:**

```
Step 1: Pre-Translation Analysis
- Identify technical terms and concepts
- Mark terms to keep in English
- Note cultural references that need adaptation
- Review prerequisite vocabulary

Step 2: Initial Translation
- Translate content section by section
- Maintain paragraph structure
- Preserve formatting (headings, lists, code blocks)
- Keep code comments in English (with Urdu explanations separate)

Step 3: Technical Review
- Verify technical accuracy with SME
- Confirm terminology consistency
- Check formula and code correctness
- Validate examples and analogies

Step 4: Language Review
- Check grammar and syntax
- Ensure natural flow and readability
- Verify tone appropriateness
- Confirm consistency with style guide

Step 5: Final Proofreading
- Check typography and punctuation
- Verify all terms in glossary
- Ensure formatting consistency
- Test readability with target audience
```

### Maintain Technical Meaning

**Technical Term Handling:**

**Category 1: Keep in English (with Urdu explanation)**
Terms that are universally used in English in technical contexts:

```
Examples:
- Robot (روبوٹ - phonetic, keep English)
- Sensor (سینسر - phonetic, keep English)
- Microcontroller (مائیکرو کنٹرولر - phonetic, keep English)
- Algorithm (الگورتھم - phonetic, keep English)
- Programming (پروگرامنگ - phonetic, keep English)
- Code (کوڈ - phonetic, keep English)
- Loop (لوپ - phonetic, keep English)
- Function (فنکشن - phonetic, keep English)
- Variable (ویری ایبل - phonetic, keep English)
- Array (اریے - phonetic, keep English)

Format: English (اردو تلفظ - phonetic)
Then provide Urdu explanation: "Robot ایک خودکار مشین ہے"
```

**Category 2: Translate with Technical Precision**
Terms that have established Urdu equivalents:

```
Examples:
- Input → ان پٹ / داخلہ
- Output → آؤٹ پٹ / خارجہ
- Speed → رفتار
- Distance → فاصلہ
- Temperature → درجہ حرارت
- Motor → موٹر
- Circuit → سرکٹ / برقی راستہ
- Current → کرنٹ / برقی رو
- Voltage → وولٹیج / برقی دباؤ
- Power → طاقت / قوت
```

**Category 3: Create Descriptive Urdu Equivalents**
For terms without established translations, create clear descriptive phrases:

```
Examples:
- Servo Motor → سروو موٹر (درست زاویے پر گھومنے والی موٹر)
- Ultrasonic Sensor → الٹراسونک سینسر (آواز کی لہروں سے فاصلہ ناپنے والا آلہ)
- PWM (Pulse Width Modulation) → PWM (وقفہ جاتی چوڑائی میں تبدیلی)
- Servo → سروو (پوزیشن کنٹرول موٹر)
- Actuator → محرک (حرکت پیدا کرنے والا آلہ)
```

**Preservation Strategies:**

1. **Technical Formulas:**
```
Keep formulas in standard mathematical notation:
Distance = Speed × Time
فاصلہ = رفتار × وقت

Always provide:
- Formula in standard notation
- Urdu translation of terms
- Worked example with Urdu explanation
```

2. **Code Blocks:**
```python
# Keep code in English
def calculate_distance(speed, time):
    distance = speed * time
    return distance

# پروگرام کی وضاحت:
# یہ فنکشن رفتار اور وقت سے فاصلہ نکالتا ہے
```

**Explanation in Urdu:**
```
اوپر دی گئی کوڈ میں:
- `calculate_distance` ایک فنکشن ہے جو فاصلہ نکالتا ہے
- `speed` رفتار کو ظاہر کرتا ہے
- `time` وقت کو ظاہر کرتا ہے
- یہ فاصلہ واپس کرتا ہے (return)
```

3. **Diagrams and Labels:**
- Keep technical labels in English
- Provide Urdu caption explaining the diagram
- Use bilingual legends when appropriate

### Ensure Student-Friendly Urdu

**Tone Guidelines:**

**1. Encouraging and Supportive:**
```
Good (Supportive):
"آئیں اس تصور کو سمجھتے ہیں۔ یہ پہلے مشکل لگ سکتا ہے لیکن آسان ہے۔"
"Let's understand this concept. It may seem difficult at first but it's easy."

Avoid (Intimidating):
"یہ ایک پیچیدہ تصور ہے جو سمجھنا ضروری ہے۔"
"This is a complex concept that must be understood."
```

**2. Age-Appropriate Language:**

**For Elementary (Ages 8-12):**
```
"روبوٹ ایک ایسی مشین ہے جو ہماری مدد کر سکتی ہے۔ یہ ہماری بات سن سکتی ہے اور کام کر سکتی ہے!"

"A robot is a machine that can help us. It can listen to us and do work!"
```

**For Middle School (Ages 13-15):**
```
"روبوٹ ایک خودکار مشین ہے جو سینسرز کی مدد سے اپنے اردگرد کی چیزیں محسوس کرتی ہے اور پھر مقررہ پروگرام کے مطابق کام کرتی ہے۔"

"A robot is an autonomous machine that senses its surroundings using sensors and then works according to its programmed instructions."
```

**For High School/College (Ages 16+):**
```
"روبوٹکس میں مکینیکل انجینئرنگ، الیکٹرانکس، اور کمپیوٹر سائنس کا امتزاج ہے۔ ایک روبوٹ سینسر ڈیٹا کو پروسیس کرکے اپنے actuators کو کنٹرول کرتا ہے۔"

"Robotics combines mechanical engineering, electronics, and computer science. A robot processes sensor data to control its actuators."
```

**3. Interactive and Engaging:**
```
Good (Engaging):
"کیا آپ نے کبھی سوچا ہے کہ روبوٹ کیسے چلتا ہے؟ آئیں دریافت کرتے ہیں!"
"Have you ever wondered how a robot walks? Let's discover!"

Avoid (Dry):
"روبوٹ کی حرکت کی وضاحت۔"
"Explanation of robot motion."
```

**4. Use Examples and Analogies:**
```
"سینسر آپ کی آنکھوں کی طرح ہے۔ جیسے آپ روشنی دیکھتے ہیں، سینسر روشنی کو محسوس کرتا ہے۔"

"A sensor is like your eyes. Just as you see light, a sensor detects light."
```

**Language Quality Standards:**

**1. Grammar and Syntax:**
- Use correct Urdu grammar (اعراب - diacritical marks when needed for clarity)
- Follow proper sentence structure (SOV pattern)
- Use appropriate conjunctions (اور، لیکن، کیونکہ، اس لیے)
- Maintain verb agreement

**2. Vocabulary:**
- Use common, everyday Urdu words when possible
- Introduce technical terms gradually
- Provide definitions for new terms
- Build vocabulary progressively across chapters

**3. Punctuation:**
- Use Urdu punctuation conventions
- Question mark: ؟
- Comma: ،
- Period: ۔
- Quotation marks: "" or ''
- Proper spacing around punctuation

**4. Typography:**
- Right-to-left text direction
- Proper Nastaliq or Naskh font
- Appropriate font size for readability
- Consistent formatting across all content

## Translation Best Practices

### Style Guide Essentials

**1. Consistency Table:**
Create and maintain a terminology table:

| English | Urdu | Notes |
|---------|------|-------|
| Robot | روبوٹ | Keep English with Urdu phonetic |
| Sensor | سینسر | Keep English with Urdu phonetic |
| Motor | موٹر | Universal term |
| Speed | رفتار | Use Urdu equivalent |
| Distance | فاصلہ | Use Urdu equivalent |
| Program | پروگرام | Keep English with Urdu phonetic |
| Variable | ویری ایبل | Keep English, explain: "متغیر" |
| Loop | لوپ | Keep English, explain: "دہرانا" |

**2. Common Phrases:**

| English | Urdu Translation |
|---------|------------------|
| "Let's learn" | "آئیں سیکھتے ہیں" |
| "For example" | "مثال کے طور پر" |
| "In other words" | "دوسرے لفظوں میں" |
| "Step by step" | "قدم بہ قدم" |
| "Important note" | "اہم نوٹ" |
| "Try it yourself" | "خود کوشش کریں" |
| "Summary" | "خلاصہ" |
| "Practice exercise" | "مشق کی مثال" |

**3. Section Formatting:**

```markdown
# Chapter Title (English kept for navigation)
# باب: روبوٹکس کا تعارف

## Section Heading
## سیکشن: سینسر کیا ہے؟

### Subsection
### ذیلی سیکشن: سینسر کی اقسام

**Important:** اہم
**Note:** نوٹ
**Example:** مثال
**Exercise:** مشق
```

### Quality Assurance Checklist

**Pre-Publishing Checklist:**

- [ ] All technical terms checked against glossary
- [ ] Formulas and equations verified for accuracy
- [ ] Code examples tested and working
- [ ] Grammar and spelling checked
- [ ] Tone appropriate for target age group
- [ ] Examples culturally relevant
- [ ] Formatting consistent throughout
- [ ] Images and diagrams have Urdu captions
- [ ] All hyperlinks working
- [ ] Readability tested with sample audience
- [ ] SME review completed
- [ ] Native Urdu speaker review completed

### Cultural Adaptation

**1. Localize Examples:**
```
Instead of: "Consider a traffic light in New York..."
Use: "اسلام آباد یا کراچی کی ٹریفک لائٹ کے بارے میں سوچیں..."
"Think about a traffic light in Islamabad or Karachi..."
```

**2. Use Familiar References:**
```
Instead of: "Like a baseball game..."
Use: "کرکٹ میچ کی طرح..." (Like a cricket match...)
```

**3. Adapt Scenarios:**
```
Make examples relevant to Pakistani/South Asian context:
- Use local currency (Rupees) in examples
- Reference local places and landmarks
- Use culturally familiar scenarios
```

**4. Respect Cultural Sensitivities:**
- Use gender-inclusive language
- Avoid examples that may be culturally inappropriate
- Use respectful forms of address
- Consider religious and cultural diversity

### Translation Tools and Resources

**Recommended Resources:**

1. **Dictionaries:**
   - Urdu Lughat (اردو لغت) - comprehensive Urdu dictionary
   - Technical Urdu dictionaries from universities
   - Feroz-ul-Lughat for modern Urdu

2. **Style Guides:**
   - Pakistani government Urdu style guides
   - University Urdu departments publications
   - National Language Authority guidelines

3. **Technical Glossaries:**
   - Computer terminology in Urdu
   - Engineering terms in Urdu
   - Scientific terminology databases

4. **Translation Memory:**
   - Maintain translation memory database
   - Reuse consistent translations
   - Build institutional knowledge

### Common Pitfalls to Avoid

**1. Literal Translation:**
```
Bad (Too Literal):
English: "The robot moves forward."
Wrong Urdu: "وہ روبوٹ آگے حرکت کرتا ہے۔"
Correct Urdu: "روبوٹ آگے بڑھتا ہے۔"
```

**2. Over-Translation:**
```
Bad (Over-translated):
"The for-loop"
Wrong: "کے لیے لوپ" (literal translation)
Correct: "فار لوپ" (keep technical term)
```

**3. Inconsistent Terminology:**
```
Bad:
First instance: "Sensor کو سینسر کہتے ہیں"
Later: "محسوس کرنے والا آلہ"

Good:
Always use: "سینسر (Sensor)"
```

**4. Awkward Sentence Structure:**
```
Bad (English word order):
"روبوٹ کو آگے move کرنے کے لیے، ہم use کرتے ہیں motor"

Good (Natural Urdu structure):
"روبوٹ کو آگے حرکت دینے کے لیے ہم موٹر استعمال کرتے ہیں"
```

## Usage Guidelines

When this skill is invoked, agents should:

1. **Consult the terminology glossary** before translating technical terms
2. **Maintain consistent style** across all translated content
3. **Prioritize clarity** over literal accuracy
4. **Verify technical accuracy** with subject matter experts
5. **Test readability** with target audience when possible
6. **Document decisions** for future reference

## Integration with Other Skills

**Content Personalization + Urdu Translation:**
- Adapt translation complexity based on user skill level
- Use simpler Urdu for beginners
- Use more technical Urdu vocabulary for advanced learners

**RAG + Urdu Translation:**
- Store both English and Urdu versions in vector database
- Enable bilingual search capabilities
- Retrieve context in user's preferred language

**Authentication + Urdu Translation:**
- Detect user's language preference
- Serve content in appropriate language
- Allow language switching mid-session

## Evaluation Metrics

**Translation Quality Metrics:**
- **Accuracy**: Technical correctness (target: 100%)
- **Fluency**: Natural Urdu flow (target: >90% native speaker approval)
- **Consistency**: Terminology uniformity (target: 100%)
- **Readability**: Age-appropriate comprehension (target: >85%)
- **Completeness**: All content translated (target: 100%)

**User Feedback Metrics:**
- User satisfaction with Urdu content
- Comprehension rates (quiz scores)
- Time spent on Urdu vs English content
- Language preference patterns

## Example Translation

**English Original:**
```
# Introduction to Sensors

A sensor is a device that detects changes in the environment. Robots use
sensors to understand their surroundings, just like how you use your eyes
to see and your ears to hear.

## Types of Sensors

1. **Light Sensor**: Detects brightness
2. **Distance Sensor**: Measures how far away objects are
3. **Temperature Sensor**: Detects heat or cold

### Example: Light Sensor

A light sensor can tell if a room is dark or bright. We can use this to
make a robot that turns on a light when it gets dark.

```python
if light_sensor.value < 100:  # If it's dark
    turn_on_light()
```

**Try it yourself!** Build a simple light-detecting circuit.
```

**Urdu Translation:**
```
# سینسرز کا تعارف

سینسر (Sensor) ایک ایسا آلہ ہے جو ماحول میں ہونے والی تبدیلیوں کو محسوس کرتا ہے۔
روبوٹ اپنے اردگرد کو سمجھنے کے لیے سینسرز استعمال کرتے ہیں، بالکل اسی طرح جیسے
آپ دیکھنے کے لیے اپنی آنکھیں اور سننے کے لیے اپنے کان استعمال کرتے ہیں۔

## سینسرز کی اقسام

۱۔ **لائٹ سینسر**: روشنی کو محسوس کرتا ہے
۲۔ **ڈسٹنس سینسر**: اشیاء کی دوری ناپتا ہے
۳۔ **ٹمپریچر سینسر**: گرمی یا سردی کو محسوس کرتا ہے

### مثال: لائٹ سینسر

لائٹ سینسر بتا سکتا ہے کہ کمرہ اندھیرا ہے یا روشن۔ ہم اس کا استعمال ایک ایسا
روبوٹ بنانے کے لیے کر سکتے ہیں جو اندھیرا ہونے پر لائٹ جلا دے۔

```python
if light_sensor.value < 100:  # اگر اندھیرا ہے
    turn_on_light()
```

**خود کوشش کریں!** روشنی محسوس کرنے والا ایک سادہ سرکٹ بنائیں۔
```

---

This skill provides comprehensive guidance for high-quality, student-friendly Urdu translations that preserve technical accuracy while ensuring clarity and cultural relevance.
