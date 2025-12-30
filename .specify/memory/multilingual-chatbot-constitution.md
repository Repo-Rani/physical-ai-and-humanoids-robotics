---
title: "Multilingual Docusaurus + RAG Chatbot Constitution"
author: "Physical AI Curriculum Team"
description: "Governance framework for multilingual support in Docusaurus documentation and RAG chatbot with English, Urdu, Arabic, Spanish, French, German, Chinese, and Japanese"
project_type: "Web Application + AI Chatbot"
constitution_version: "1.0.0"
ratification_date: "2025-12-28"
last_amended: "2025-12-28"
parent_constitution: ".specify/memory/constitution.md"
---

# Multilingual Docusaurus + RAG Chatbot Constitution

## Document Metadata

**Feature Name**: Multilingual Support for Docusaurus + RAG Chatbot
**Constitution Version**: 1.0.0
**Ratified**: 2025-12-28
**Last Amended**: 2025-12-28
**Parent Constitution**: `.specify/memory/constitution.md` (Physical AI & Humanoid Robotics Textbook)
**Methodology**: Spec-Kit Plus + Claude Code
**Tech Stack**:
- **Frontend**: Docusaurus 3.x, React 18.x
- **Backend**: FastAPI (Python 3.10+)
- **AI Services**: OpenRouter API (`mistralai/devstral-2512:free`)
- **RAG Stack**: Qdrant (vector DB) + Cohere (embeddings)

---

## Project Overview

This constitution governs the implementation of internationalization (i18n) features for both:
1. **Docusaurus Documentation Website**: Multi-language documentation with RTL support
2. **RAG Chatbot**: Real-time translation and multilingual conversation capabilities

**Supported Languages** (8 total):
- English (en) - Default
- Urdu (ur) - RTL
- Arabic (ar) - RTL
- Spanish (es)
- French (fr)
- German (de)
- Chinese (zh)
- Japanese (ja)

**Key Features**:
- Docusaurus i18n infrastructure with language switcher
- RTL layout for Urdu and Arabic
- Chatbot language selector with 8 languages
- Real-time conversation translation
- Bidirectional multilingual chat
- Text selection handler with language context
- Backend translation API endpoint

---

## Core Principles (Non-Negotiable)

### Principle 1: Language Parity & Inclusivity

**Rule**: All supported languages MUST provide equivalent functionality, user experience, and content quality without degradation or missing features.

**Implementation Requirements**:

1. **Feature Parity**:
   - Language switcher works identically across all pages
   - Chatbot functionality identical in all 8 languages
   - No feature gating based on language selection
   - Search functionality works for all languages
   - Navigation structure consistent across locales

2. **Content Quality**:
   - Technical terms MUST maintain accuracy across translations
   - Preserve transliteration format: `Native (English)`
   - Example (Urdu): `روبوٹ (Robot)`
   - Example (Arabic): `روبوت (Robot)`
   - Code examples remain in English (universal standard)
   - Comments in code can be translated (optional)

3. **User Experience Consistency**:
   - Page load times similar across all languages
   - Font rendering quality equal for all character sets
   - No broken glyphs or character encoding issues
   - Responsive design works for all languages
   - Touch targets same size across languages

4. **Translation Infrastructure**:
   - OpenRouter API used for all machine translations
   - Human review required before publishing translations
   - Translation memory to ensure consistency
   - Glossary of technical terms with approved translations
   - Version control for all translation files

**Rationale**: Language selection should never result in a degraded experience. Non-English speakers deserve the same quality as English speakers. Maintaining technical accuracy across languages is critical for educational content.

**Validation Process**:
- Native speaker review for each language
- Automated tests for feature parity across locales
- Performance benchmarks for all language versions
- Font rendering tests across browsers and devices
- User testing with speakers of each language

---

### Principle 2: RTL (Right-to-Left) Layout Excellence

**Rule**: RTL languages (Urdu, Arabic) MUST have fully mirrored layouts with proper text rendering, bidirectional text handling, and no layout breaks.

**Implementation Requirements**:

1. **HTML Structure**:
   - Set `dir="rtl"` on root HTML element for RTL locales
   - Set `lang="ur"` or `lang="ar"` for correct font selection
   - Preserve LTR for code blocks: `<code dir="ltr">`
   - Preserve LTR for numbers and technical terms when embedded

2. **CSS Logical Properties**:
   - Use `margin-inline-start` instead of `margin-left`
   - Use `margin-inline-end` instead of `margin-right`
   - Use `padding-inline-start` instead of `padding-left`
   - Use `border-inline-start` instead of `border-left`
   - Use `text-align: start` instead of `text-align: left`
   - NO hardcoded directional properties (left/right)

3. **Component Mirroring**:
   - Navbar: Logo flips to right, menu items to left
   - Sidebar: Flips to right side of screen
   - Breadcrumbs: Arrows point left instead of right
   - Pagination: Previous/Next buttons swap positions
   - Dropdown menus: Open to left instead of right
   - Icons with directionality: Mirror horizontally

4. **Bidirectional Text (Bidi)**:
   - Technical terms in English within RTL text render correctly
   - Use Unicode bidi control characters when needed:
     - `\u202B` (RLE - Right-to-Left Embedding)
     - `\u202C` (PDF - Pop Directional Formatting)
   - Example: `ROS 2 \u202Bروبوٹ سسٹم\u202C framework`
   - Numbers maintain LTR direction within RTL text

5. **Chatbot RTL Behavior**:
   - Message bubbles align right for RTL languages
   - User messages: Right-aligned, green gradient preserved
   - Bot messages: Right-aligned, white/gray preserved
   - Timestamps: Right-aligned
   - Input field: Text flows right-to-left
   - Send button: Positioned on left side
   - Scrollbar: Appears on left side

6. **Font Selection for RTL**:
   - **Urdu**: Use Noto Nastaliq Urdu or Jameel Noori Nastaleeq
   - **Arabic**: Use Noto Sans Arabic or Cairo
   - Fallback chain:
     ```css
     font-family: 'Noto Nastaliq Urdu', 'Noto Sans Arabic',
                  'Arial', sans-serif;
     ```
   - Ensure web fonts loaded before rendering

7. **Testing Requirements**:
   - Test all pages in RTL mode
   - Test chatbot in RTL languages
   - Test text selection in RTL content
   - Test form inputs with RTL text
   - Test copy-paste operations
   - Browser compatibility: Chrome, Firefox, Safari, Edge

**Rationale**: RTL languages represent over 500 million speakers globally. Proper RTL support is not optional—it's a fundamental accessibility requirement. Poor RTL implementation creates cognitive load and frustration for native speakers.

**Validation Process**:
- Manual testing by native Urdu and Arabic speakers
- Automated visual regression tests comparing LTR/RTL layouts
- Browser DevTools inspection for logical properties
- Accessibility audit with RTL-aware tools
- User feedback collection from RTL language users

---

### Principle 3: Translation API Reliability & Quality

**Rule**: The translation API MUST provide accurate, contextually appropriate translations with proper error handling, rate limiting, and fallback mechanisms.

**Implementation Requirements**:

1. **API Endpoint Specification**:
   - **URL**: `POST /api/v1/translate`
   - **Request Schema**:
     ```json
     {
       "text": "string (required, max 5000 chars)",
       "target_language": "string (required, ISO 639-1 code)",
       "source_language": "string (optional, auto-detect if missing)",
       "context": "string (optional, domain hint: 'robotics', 'ai', 'technical')"
     }
     ```
   - **Response Schema**:
     ```json
     {
       "translated_text": "string",
       "source_language": "string",
       "target_language": "string",
       "model": "string",
       "confidence": "number (0-1)",
       "cached": "boolean"
     }
     ```
   - **Error Response**:
     ```json
     {
       "error": "string",
       "error_code": "string",
       "details": "object (optional)"
     }
     ```

2. **Translation Prompt Engineering**:
   - System prompt template:
     ```
     You are a technical translator specializing in robotics, AI,
     and computer science documentation. Translate the following text
     to {target_language}.

     RULES:
     - Maintain all technical terminology accuracy
     - Preserve code snippets without translation
     - Keep mathematical notation unchanged
     - Provide transliteration for technical terms: Native (English)
     - Maintain original formatting (line breaks, lists, emphasis)
     - Only return the translated text, no explanations

     Text to translate:
     {text}
     ```
   - Context-aware variations:
     - Robotics: Emphasize ROS 2, URDF, simulation terminology
     - AI: Emphasize VLA, neural networks, machine learning terms
     - General: Standard technical translation

3. **OpenRouter Configuration**:
   - **Model**: `mistralai/devstral-2512:free`
   - **Base URL**: `https://openrouter.ai/api/v1`
   - **API Key**: From environment variable `OPENROUTER_API_KEY`
   - **Timeout**: 30 seconds
   - **Retry Logic**: 3 attempts with exponential backoff
   - **Max tokens**: 2048

4. **Rate Limiting**:
   - **Per IP**: 100 requests per minute
   - **Per user session**: 50 requests per minute
   - **Burst allowance**: 10 requests per second
   - HTTP 429 response when exceeded:
     ```json
     {
       "error": "Rate limit exceeded",
       "error_code": "RATE_LIMIT_EXCEEDED",
       "retry_after": 60
     }
     ```

5. **Caching Strategy**:
   - **In-memory cache**: 15-minute TTL
   - **Cache key**: `hash(text + source_lang + target_lang)`
   - **Cache size**: 1000 entries (LRU eviction)
   - **Cache hit metrics**: Log cache hit rate
   - Benefits:
     - Reduced API costs
     - Faster response times
     - Consistent translations for same text

6. **Error Handling**:
   - **OpenRouter API errors**: Return 502 Bad Gateway
   - **Invalid language code**: Return 400 Bad Request
   - **Text too long**: Return 413 Payload Too Large
   - **Timeout**: Return 504 Gateway Timeout
   - **Fallback**: Return original text with error flag
   - Log all errors with context for debugging

7. **Quality Assurance**:
   - Technical term validation:
     - Maintain whitelist of terms that should NOT be translated
     - Example: `ROS 2`, `SLAM`, `URDF`, `Docker`, `Python`
   - Post-processing:
     - Check for preserved formatting
     - Validate no code blocks were altered
     - Ensure mathematical notation unchanged
   - Confidence scoring:
     - Flag low-confidence translations (< 0.7) for review
     - Log confidence scores for monitoring

**Rationale**: Poor translations can misrepresent technical concepts, confuse students, and damage credibility. The translation API is mission-critical infrastructure that must be reliable, fast, and accurate. Caching prevents redundant API calls and reduces costs.

**Validation Process**:
- Unit tests for all API endpoints
- Integration tests with OpenRouter API
- Load testing: 1000 concurrent translation requests
- Translation quality assessment by native speakers
- Monitor API latency, error rates, cache hit rates
- Regular audits of cached translations for accuracy

---

### Principle 4: Chatbot Multilingual User Experience

**Rule**: The chatbot MUST provide seamless language switching, persistent language preferences, and intuitive translation controls without disrupting conversation flow.

**Implementation Requirements**:

1. **Language Selector UI**:
   - **Location**: Chatbot header, top-right corner
   - **Design**: Dropdown button with current language + flag emoji
   - **Languages**:
     ```javascript
     [
       { code: 'en', name: 'English', flag: '🇺🇸', dir: 'ltr' },
       { code: 'ur', name: 'اردو', flag: '🇵🇰', dir: 'rtl' },
       { code: 'ar', name: 'العربية', flag: '🇸🇦', dir: 'rtl' },
       { code: 'es', name: 'Español', flag: '🇪🇸', dir: 'ltr' },
       { code: 'fr', name: 'Français', flag: '🇫🇷', dir: 'ltr' },
       { code: 'de', name: 'Deutsch', flag: '🇩🇪', dir: 'ltr' },
       { code: 'zh', name: '中文', flag: '🇨🇳', dir: 'ltr' },
       { code: 'ja', name: '日本語', flag: '🇯🇵', dir: 'ltr' }
     ]
     ```
   - **Styling**: Match green gradient theme, smooth hover transitions
   - **Accessibility**: Keyboard navigable, ARIA labels

2. **"Translate Conversation" Button**:
   - **Location**: Chatbot header, next to language selector
   - **Icon**: 🌐 or translation icon from lucide-react
   - **Behavior**:
     - Click to translate ALL messages to selected language
     - Show loading state: Progress bar + "Translating..."
     - Disable button during translation (prevent double-click)
     - Re-enable after completion
   - **Animation**: Smooth fade-in/out for translated text
   - **Undo**: Click again to revert to original language

3. **Message Storage Structure**:
   - localStorage key: `chatbot_messages`
   - Message schema:
     ```javascript
     {
       id: string,  // UUID
       type: 'user' | 'bot',
       text: string,  // Currently displayed text
       originalText: string,  // Original language text (before translation)
       language: string,  // Language code of current text
       originalLanguage: string,  // Language code of original text
       timestamp: number,
       sources: array (optional),  // For bot messages
       selectedContext: string (optional)  // For user messages with text selection
     }
     ```
   - State management:
     ```javascript
     {
       messages: Message[],
       selectedLanguage: string,  // Current chatbot language
       isTranslated: boolean,  // Are messages currently translated?
       translationInProgress: boolean
     }
     ```

4. **Real-time Translation Flow**:
   - **User clicks "Translate Conversation"**:
     1. Set `translationInProgress = true`
     2. Show loading UI with progress: "Translating message 3/15..."
     3. For each message in `messages`:
        ```javascript
        if (message.language !== selectedLanguage) {
          // Call translation API
          const translated = await fetch('/api/v1/translate', {
            method: 'POST',
            body: JSON.stringify({
              text: message.originalText || message.text,
              target_language: selectedLanguage,
              source_language: message.originalLanguage || message.language
            })
          })

          // Update message
          message.text = translated.translated_text
          message.language = selectedLanguage
          if (!message.originalText) {
            message.originalText = message.text  // Save original
            message.originalLanguage = message.language
          }
        }
        ```
     4. Update localStorage
     5. Re-render chat UI
     6. Set `translationInProgress = false`, `isTranslated = true`

5. **Bidirectional Conversation**:
   - **User sends message in selected language**:
     - Message stored with `language = selectedLanguage`
     - Sent to backend with language metadata:
       ```json
       {
         "message": "user message text",
         "language": "ur",
         "context": "selected text context",
         "session_id": "uuid"
       }
       ```
   - **Bot responds in same language**:
     - Backend detects user message language
     - Generates response in same language
     - If RAG retrieves English content, translate to user's language
     - Response includes language metadata:
       ```json
       {
         "response": "bot response text",
         "language": "ur",
         "sources": [...],
         "translated": true
       }
       ```

6. **Language Persistence**:
   - localStorage key: `chatbot_language`
   - Save on language change: `localStorage.setItem('chatbot_language', 'ur')`
   - Load on chatbot init:
     ```javascript
     const savedLanguage = localStorage.getItem('chatbot_language') || 'en'
     setSelectedLanguage(savedLanguage)
     ```
   - Persist across:
     - Page reloads
     - Navigation between pages
     - Browser sessions

7. **Text Selection Handler Integration**:
   - TextSelectionHandler must detect page language
   - When user selects text:
     - Detect if text is RTL or LTR
     - Send to chatbot with language context:
       ```javascript
       {
         selectedText: "text",
         pageLanguage: "ur",  // From <html lang="ur">
         textDirection: "rtl"
       }
       ```
   - Chatbot responds in same language as selected text

8. **Loading States**:
   - **Translation in progress**:
     - Show overlay on chat window
     - Progress bar: "Translating 3/15 messages..."
     - Disable input field and send button
   - **Individual message translation**:
     - Shimmer effect on message being translated
   - **API error**:
     - Toast notification: "Translation failed. Try again."
     - Revert to original language

**Rationale**: Language switching must feel instant and natural. Users should never lose their conversation history when changing languages. Storing both original and translated text enables instant switching without re-translating. Persistent preferences prevent repetitive language selection.

**Validation Process**:
- User testing with speakers of all 8 languages
- Test language switching mid-conversation
- Test persistence across page reloads
- Test translation of 50+ message conversation (performance)
- Test error handling with API failures
- Accessibility testing for screen readers in each language

---

### Principle 5: Docusaurus i18n Best Practices

**Rule**: Docusaurus i18n configuration MUST follow official best practices, use proper folder structure, and enable smooth build/deployment for all locales.

**Implementation Requirements**:

1. **docusaurus.config.js Configuration**:
   ```javascript
   module.exports = {
     i18n: {
       defaultLocale: 'en',
       locales: ['en', 'ur', 'ar', 'es', 'fr', 'de', 'zh', 'ja'],
       localeConfigs: {
         en: {
           label: 'English',
           direction: 'ltr',
           htmlLang: 'en-US',
           calendar: 'gregory',
           path: 'en',
         },
         ur: {
           label: 'اردو',
           direction: 'rtl',
           htmlLang: 'ur-PK',
           calendar: 'gregory',
           path: 'ur',
         },
         ar: {
           label: 'العربية',
           direction: 'rtl',
           htmlLang: 'ar-SA',
           calendar: 'gregory',
           path: 'ar',
         },
         // ... (other languages)
       },
     },
     themeConfig: {
       navbar: {
         items: [
           {
             type: 'localeDropdown',
             position: 'right',
           },
         ],
       },
     },
   }
   ```

2. **Folder Structure**:
   ```
   ai-and-humanoid-robotics-hackathon/
   ├── docs/                                    # English (default)
   │   ├── intro.md
   │   ├── module-1-ros2/
   │   └── ...
   ├── i18n/
   │   ├── ur/                                  # Urdu translations
   │   │   ├── docusaurus-plugin-content-docs/
   │   │   │   └── current/
   │   │   │       ├── intro.md
   │   │   │       ├── module-1-ros2/
   │   │   │       └── ...
   │   │   ├── docusaurus-theme-classic/
   │   │   │   ├── navbar.json
   │   │   │   └── footer.json
   │   │   └── code.json                        # UI strings
   │   ├── ar/                                  # Arabic translations
   │   │   └── ...
   │   └── ...                                  # Other languages
   └── docusaurus.config.js
   ```

3. **Translation Workflow**:
   - **Step 1: Write translations command**
     ```bash
     npm run write-translations -- --locale ur
     ```
     Generates: `i18n/ur/code.json` with all UI strings

   - **Step 2: Copy English docs**
     ```bash
     # Windows PowerShell
     xcopy docs i18n\ur\docusaurus-plugin-content-docs\current\ /E /I

     # Linux/Mac
     mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
     cp -r docs/* i18n/ur/docusaurus-plugin-content-docs/current/
     ```

   - **Step 3: Translate markdown files**
     - Translate content in `i18n/ur/docusaurus-plugin-content-docs/current/`
     - Keep frontmatter structure identical
     - Preserve code blocks in English
     - Translate headings, paragraphs, lists

   - **Step 4: Translate UI strings**
     - Edit `i18n/ur/code.json`
     - Translate all navbar labels, button text, placeholders
     - Example:
       ```json
       {
         "theme.navbar.search.placeholder": "تلاش کریں",
         "theme.docs.sidebar.collapseButtonTitle": "سائڈبار بند کریں",
         "theme.docs.paginator.next": "اگلا"
       }
       ```

4. **Build Scripts in package.json**:
   ```json
   {
     "scripts": {
       "start": "docusaurus start",
       "start:ur": "docusaurus start --locale ur",
       "start:ar": "docusaurus start --locale ar",
       "build": "docusaurus build",
       "build:ur": "docusaurus build --locale ur",
       "build:all": "npm run build && npm run build:ur && npm run build:ar",
       "serve": "docusaurus serve",
       "write-translations": "docusaurus write-translations",
       "write-translations:ur": "docusaurus write-translations --locale ur"
     }
   }
   ```

5. **Language Switcher in Navbar**:
   - Automatically generated by `type: 'localeDropdown'`
   - Position: `right` (top-right corner)
   - Shows current language with dropdown
   - Clicking switches to that language version
   - URL structure:
     - English: `https://example.com/intro`
     - Urdu: `https://example.com/ur/intro`
     - Arabic: `https://example.com/ar/intro`

6. **Metadata for SEO**:
   - **hreflang tags** (auto-generated by Docusaurus):
     ```html
     <link rel="alternate" hreflang="en" href="https://example.com/intro" />
     <link rel="alternate" hreflang="ur" href="https://example.com/ur/intro" />
     <link rel="alternate" hreflang="ar" href="https://example.com/ar/intro" />
     <link rel="alternate" hreflang="x-default" href="https://example.com/intro" />
     ```
   - **Sitemap** includes all language versions
   - **robots.txt** allows all locales

7. **Content Translation Guidelines**:
   - **DO translate**:
     - Headings, paragraphs, lists
     - Image alt text
     - Link text (not URLs)
     - Admonition content (notes, warnings, tips)
   - **DO NOT translate**:
     - Code blocks
     - File paths
     - Command-line commands
     - URLs
     - Technical terms (transliterate instead)
     - Frontmatter keys (only values)

8. **Testing Locales**:
   ```bash
   # Test English (default)
   npm start

   # Test Urdu
   npm run start:ur
   # Visit http://localhost:3000/ur

   # Build all locales
   npm run build:all

   # Serve production build
   npm run serve
   ```

**Rationale**: Following Docusaurus official i18n conventions ensures compatibility with future updates, proper SEO, and correct build output. Structured folder hierarchy makes translations maintainable. Build scripts enable easy testing and deployment of all language versions.

**Validation Process**:
- `docusaurus build` succeeds for all locales
- All language URLs resolve correctly
- Language switcher works on all pages
- hreflang tags present in HTML
- Sitemap includes all locales
- No console errors or warnings

---

### Principle 6: Performance & Scalability

**Rule**: Multilingual features MUST NOT degrade performance. Translation operations MUST be optimized for speed and cost efficiency.

**Implementation Requirements**:

1. **Frontend Performance**:
   - **Language switching**: < 200ms
   - **Conversation translation**: < 3 seconds for 20 messages
   - **Individual message translation**: < 1 second
   - **Lazy loading**: Only load fonts for active language
   - **Code splitting**: Separate bundles per locale
   - **Asset optimization**: Compress images, minify JS/CSS

2. **Backend Performance**:
   - **Translation API latency**: < 500ms (cached), < 2s (uncached)
   - **Rate limiting**: Prevent abuse, protect API quota
   - **Caching hit rate**: Target > 70% for common phrases
   - **Concurrent requests**: Handle 100+ simultaneous translations
   - **Database queries**: Optimize for multilingual content retrieval

3. **Caching Strategy**:
   - **Browser cache**:
     - Cache translated pages: 1 hour
     - Cache font files: 1 year
     - Cache static assets: 1 week
   - **API cache**:
     - In-memory cache: 15 minutes (1000 entries)
     - Redis cache (optional): 24 hours
     - CDN cache: 1 hour for translated content

4. **Cost Optimization**:
   - **OpenRouter API**:
     - Using free tier: `mistralai/devstral-2512:free`
     - Caching prevents redundant API calls
     - Batch translation requests when possible
   - **Monitoring**:
     - Track API usage: requests per day
     - Alert if approaching rate limits
     - Log cache hit/miss rates

5. **Build Performance**:
   - **Full build** (all locales): < 10 minutes
   - **Incremental build** (single locale): < 2 minutes
   - **Development mode**: Hot reload < 1 second
   - **Parallel builds**: Build locales concurrently

6. **Scalability Considerations**:
   - **Adding new language**: < 1 hour setup time
   - **Translation updates**: Incremental (not full rebuild)
   - **Content growth**: Minimal impact on build time
   - **User growth**: Horizontal scaling of translation API

**Rationale**: Poor performance defeats the purpose of accessibility. Users will not tolerate slow language switching or laggy translations. Caching is essential to reduce API costs and improve response times. Build optimization ensures CI/CD pipelines remain fast.

**Validation Process**:
- Lighthouse performance audit: Score > 90
- Translation API load testing: 1000 concurrent requests
- Cache hit rate monitoring: Target > 70%
- Build time benchmarks on CI/CD
- Real user monitoring (RUM) for translation features

---

### Principle 7: Accessibility & WCAG Compliance

**Rule**: All multilingual features MUST meet WCAG 2.1 Level AA standards for accessibility, including screen reader support, keyboard navigation, and proper semantic markup.

**Implementation Requirements**:

1. **Screen Reader Support**:
   - Set `lang` attribute on all text content
   - Announce language changes: `<span lang="ur">اردو</span>`
   - ARIA labels for language selector:
     ```jsx
     <button aria-label="Select language" aria-haspopup="true">
       {currentLanguage.name}
     </button>
     ```
   - Alt text for flag emojis (decorative):
     ```jsx
     <span role="img" aria-hidden="true">🇵🇰</span>
     ```

2. **Keyboard Navigation**:
   - Language dropdown: Tab to focus, Enter to open, Arrow keys to navigate
   - Chatbot input: Works with all keyboard layouts (Arabic, Chinese, etc.)
   - Translate button: Keyboard accessible, Space/Enter to activate
   - All interactive elements: Visible focus indicators

3. **Color Contrast**:
   - Text on backgrounds: Minimum 4.5:1 ratio
   - Link text: 4.5:1 ratio, underlined on hover
   - Button text: 4.5:1 ratio
   - Error messages: 4.5:1 ratio, not relying on color alone

4. **Font Readability**:
   - Minimum font size: 16px (1rem)
   - Line height: 1.5 for body text
   - Font weight: 400 (regular) or 500 (medium)
   - No all-caps text (except abbreviations)
   - Adequate letter spacing for RTL languages

5. **Form Accessibility**:
   - Chatbot input: Proper `<label>` or `aria-label`
   - Placeholder text: Not only method of instruction
   - Error messages: Associated with input using `aria-describedby`
   - Required fields: Marked with `aria-required="true"`

6. **Mobile Accessibility**:
   - Touch targets: Minimum 44x44px
   - Pinch-to-zoom: Enabled
   - Orientation: Works in portrait and landscape
   - Text reflow: Content adapts to viewport width

7. **Testing Requirements**:
   - **Automated testing**:
     - axe DevTools: Zero violations
     - WAVE: Zero errors
     - Lighthouse accessibility audit: Score 100
   - **Manual testing**:
     - Screen reader testing: NVDA (Windows), VoiceOver (Mac/iOS)
     - Keyboard-only navigation
     - Browser zoom: 200% zoom level
   - **User testing**:
     - Users with disabilities
     - Native speakers of each language

**Rationale**: Accessibility is not optional. Students with disabilities deserve equal access to educational content. Proper semantic markup and ARIA labels ensure screen readers can navigate multilingual content correctly. WCAG compliance is a legal requirement in many jurisdictions.

**Validation Process**:
- Automated accessibility scans on every PR
- Manual screen reader testing quarterly
- User feedback from accessibility community
- Third-party accessibility audit (annual)

---

## Governance & Amendment Process

### Amendment Procedure

This constitution is subordinate to the parent constitution (`.specify/memory/constitution.md`). To amend:

1. **Proposal**: GitHub Issue with label `constitution-amendment-multilingual`
2. **Review**: Technical review by frontend + backend engineers
3. **Approval**: Curriculum lead approval required
4. **Versioning**: Semantic versioning (MAJOR.MINOR.PATCH)
5. **Migration**: Update dependent code and documentation

### Compliance Review

- **Pre-merge**: All PRs checked for i18n compliance
- **Monthly audits**: Translation quality and performance checks
- **User feedback**: Collect feedback from non-English speakers

---

## Version History

| Version | Date | Changes | Rationale |
|---------|------|---------|-----------|
| **1.0.0** | 2025-12-28 | Initial constitution for multilingual Docusaurus + RAG chatbot. Defined 7 core principles: Language Parity, RTL Excellence, Translation API Quality, Chatbot UX, Docusaurus i18n, Performance, Accessibility. | Initial ratification for multilingual feature implementation. |

---

## Appendix A: Language Metadata

| Language | ISO 639-1 | Direction | Font Family | Translation Priority |
|----------|-----------|-----------|-------------|---------------------|
| English | en | LTR | Inter, Arial | P1 (Default) |
| Urdu | ur | RTL | Noto Nastaliq Urdu | P1 (Primary target) |
| Arabic | ar | RTL | Noto Sans Arabic | P2 |
| Spanish | es | LTR | Inter, Arial | P2 |
| French | fr | LTR | Inter, Arial | P3 |
| German | de | LTR | Inter, Arial | P3 |
| Chinese | zh | LTR | Noto Sans SC | P3 |
| Japanese | ja | LTR | Noto Sans JP | P3 |

---

## Appendix B: OpenRouter API Configuration

```python
# backend/config.py
OPENROUTER_CONFIG = {
    "api_key": os.getenv("OPENROUTER_API_KEY"),
    "base_url": "https://openrouter.ai/api/v1",
    "model": "mistralai/devstral-2512:free",
    "timeout": 30,
    "max_retries": 3,
    "rate_limit": {
        "per_ip": 100,  # requests per minute
        "per_session": 50,  # requests per minute
        "burst": 10  # requests per second
    },
    "cache": {
        "enabled": True,
        "ttl": 900,  # 15 minutes
        "max_entries": 1000
    }
}
```

---

## Appendix C: Chatbot State Management

```javascript
// src/components/chatbot/types.ts
interface Message {
  id: string;
  type: 'user' | 'bot';
  text: string;
  originalText?: string;
  language: string;
  originalLanguage?: string;
  timestamp: number;
  sources?: Source[];
  selectedContext?: string;
}

interface ChatbotState {
  messages: Message[];
  selectedLanguage: string;
  isTranslated: boolean;
  translationInProgress: boolean;
  error: string | null;
}

// localStorage schema
interface ChatbotStorage {
  messages: Message[];
  language: string;
  version: string;
}
```

---

**End of Constitution**

---

**Summary**: This constitution defines the governance framework for implementing multilingual support in a Docusaurus documentation site and RAG chatbot. It enforces 7 core principles (Language Parity, RTL Excellence, Translation API Quality, Chatbot UX, Docusaurus i18n, Performance, Accessibility) with concrete validation processes. All features MUST comply with these principles to ensure a high-quality multilingual experience for global users.

For questions or clarifications, consult the parent constitution or propose an amendment via GitHub Issue.
