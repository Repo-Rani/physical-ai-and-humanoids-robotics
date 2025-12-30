# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of multilingual support for the Docusaurus documentation site and RAG chatbot, enabling access to AI and humanoid robotics educational content in 8 languages (English, Urdu, Arabic, Spanish, French, German, Chinese, Japanese). The solution includes RTL layout support for Urdu and Arabic, a translation API using OpenRouter, real-time conversation translation, and persistent language preferences. The architecture follows a web application pattern with a FastAPI backend for translation services and a Docusaurus frontend with React chatbot components that maintain WCAG 2.1 Level AA compliance and performance standards (language switching < 200ms, conversation translation < 3 seconds).

## Technical Context

**Language/Version**: Python 3.10+ (backend), JavaScript/TypeScript (frontend), React 18.x
**Primary Dependencies**: Docusaurus 3.x, FastAPI, OpenRouter API (`mistralai/devstral-2512:free`), Qdrant (vector DB), Cohere (embeddings)
**Storage**: localStorage (client-side message storage), Qdrant vector database (RAG content), in-memory cache (translations)
**Testing**: pytest (backend), Jest/React Testing Library (frontend), automated accessibility scans (axe/WAVE)
**Target Platform**: Web application (Docusaurus documentation + React chatbot UI)
**Project Type**: Web application (frontend/backend architecture)
**Performance Goals**: Language switching < 200ms, conversation translation < 3 seconds for 20 messages, cached translation API response < 500ms
**Constraints**: Translation API rate limits (100/min per IP, 50/min per session), WCAG 2.1 Level AA compliance, 95% cache hit rate target
**Scale/Scope**: Support for 8 languages (English, Urdu, Arabic, Spanish, French, German, Chinese, Japanese), RTL support for Urdu/Arabic

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Constitution gates for Physical AI & Humanoid Robotics textbook project:
- Content Accuracy & Technical Rigor: All technical explanations, ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA, and AI concepts must be correct, verified, and cited. Code snippets must run, be well-documented, and safe for hardware deployment. ✓ (Translation API maintains technical accuracy with proper validation)
- Educational Clarity & Accessibility: Ensure gradual progression from foundational Physical AI concepts to advanced humanoid robotics projects with clear learning outcomes and module objectives. ✓ (Multilingual support increases accessibility for global audience)
- Consistency & Standards: Maintain uniform terminology for AI, robotics, and simulation concepts with proper glossary and formatting rules. ✓ (Transliteration format maintained: Native (English) as specified in constitution)
- Lab & Hardware Guidelines: Verify hardware requirements (RTX PCs, Jetson kits, RealSense cameras, etc.) and cloud vs on-premise options are properly addressed. ✓ (Web-based solution requires no special hardware beyond standard web browser)
- Docusaurus Structure & Quality: Ensure chapters are navigable, searchable, and maintainable with proper metadata and interactive elements. ✓ (Follows Docusaurus i18n best practices with proper SEO and navigation consistency)
- Code & Simulation Quality: Verify all ROS 2 packages, Gazebo simulations, Isaac pipelines, and VLA code examples are reproducible with safety warnings included. ✓ (Translation API preserves code examples in English as universal standard)

Constitution gates for Multilingual Docusaurus + RAG Chatbot:
- Language Parity & Inclusivity: All 8 languages (English, Urdu, Arabic, Spanish, French, German, Chinese, Japanese) provide equivalent functionality and content quality ✓
- RTL Excellence: Urdu and Arabic have proper RTL layout support with bidirectional text handling ✓
- Translation API Quality: OpenRouter API integration with proper error handling, rate limiting, and caching ✓
- Chatbot UX: Seamless language switching, persistent preferences, and intuitive translation controls ✓
- Docusaurus i18n: Follows official best practices with proper folder structure and SEO ✓
- Performance & Scalability: Maintains performance standards with caching and optimization ✓
- Accessibility: WCAG 2.1 Level AA compliance for all multilingual features ✓

## Project Structure

### Documentation (this feature)

```text
specs/001-multilingual-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application (frontend + backend architecture)
backend/
├── main.py              # FastAPI application entry point
├── config.py            # Configuration including OpenRouter settings
├── api/
│   ├── __init__.py
│   ├── translate.py     # Translation API endpoints
│   └── chat.py          # Chat API endpoints
├── services/
│   ├── __init__.py
│   ├── translation.py   # Translation service with caching
│   └── chatbot.py       # Chatbot service with language detection
├── models/
│   ├── __init__.py
│   ├── translation.py   # Translation request/response models
│   └── chat.py          # Chat message models
└── tests/
    ├── __init__.py
    ├── test_translate.py
    └── test_chat.py

frontend/ (integrated with existing Docusaurus structure)
├── src/
│   ├── components/
│   │   ├── chatbot/
│   │   │   ├── chatbot.jsx      # Main chatbot component
│   │   │   ├── message.jsx      # Message display component
│   │   │   ├── language-selector.jsx  # Language selector UI
│   │   │   └── TextSelectionHandler.jsx  # Text selection handler
│   │   └── i18n/
│   │       ├── LanguageSwitcher.jsx  # Docusaurus language switcher
│   │       └── RTLHandler.jsx       # RTL layout handler
│   ├── pages/
│   └── services/
│       ├── translation-api.js      # Translation API client
│       └── chat-api.js            # Chat API client
└── static/
    └── fonts/                     # Web fonts for RTL languages

# Docusaurus i18n structure
i18n/
├── ur/                           # Urdu translations
│   ├── docusaurus-plugin-content-docs/
│   │   └── current/
│   │       ├── intro.md
│   │       └── ...
│   ├── docusaurus-theme-classic/
│   │   ├── navbar.json
│   │   └── footer.json
│   └── code.json                 # UI strings
├── ar/                           # Arabic translations
│   └── ...
└── ...                           # Other language translations

# Existing project structure (modified for i18n)
├── docs/                         # English documentation (default)
├── docusaurus.config.js          # Updated with i18n configuration
├── sidebars.js
└── package.json                  # Updated with i18n scripts
```

**Structure Decision**: Web application architecture selected to support Docusaurus documentation with multilingual RAG chatbot. Backend provides translation API and chat services, while frontend integrates chatbot into Docusaurus with proper i18n support for 8 languages including RTL layouts.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | All constitution gates passed | N/A |
