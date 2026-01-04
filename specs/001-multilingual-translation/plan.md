# Implementation Plan: Multi-Language Translation System

**Branch**: `001-multilingual-translation` | **Date**: January 5, 2026 | **Spec**: [link to spec](./spec.md)
**Input**: Feature specification from `/specs/001-multilingual-translation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a comprehensive multi-language translation system for the Physical AI & Humanoid Robotics educational platform. The system will support English (existing), Urdu, and Hindi languages with seamless switching capabilities while maintaining exact styling, structure, and all existing functionality. The solution will leverage Docusaurus i18n capabilities with custom components for language switching, including RTL support for Urdu content.

## Technical Context

**Language/Version**: TypeScript/JavaScript (for Docusaurus customization), Markdown for content
**Primary Dependencies**: Docusaurus 3.0+ with i18n plugin, React for custom components
**Storage**: File-based (Markdown files in /i18n directory structure)
**Testing**: Jest for component testing, manual verification for translation accuracy
**Target Platform**: Web (Docusaurus static site generation)
**Project Type**: Web/documentation platform
**Performance Goals**: Language switching under 2 seconds, build time increase <20%
**Constraints**: Must preserve all existing English content without modification, maintain RTL compatibility for Urdu
**Scale/Scope**: 3 languages (English/Urdu/Hindi), ~50-100 content pages to translate

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Status: PASSED** - All constitution requirements have been addressed in the implementation approach:

- Content Accuracy & Technical Rigor: All technical explanations, ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA, and AI concepts must be correct, verified, and cited. Code snippets must run, be well-documented, and safe for hardware deployment. Translations must maintain technical accuracy of original content. ✓ RESOLVED: Translation process preserves technical accuracy while ensuring cultural appropriateness.
- Educational Clarity & Accessibility: Ensure gradual progression from foundational Physical AI concepts to advanced humanoid robotics projects with clear learning outcomes and module objectives. Multilingual support must enhance accessibility for diverse student populations while maintaining educational quality. ✓ RESOLVED: Multi-language support with proper UI components enhances accessibility for Urdu/Hindi speakers.
- Consistency & Standards: Maintain uniform terminology for AI, robotics, and simulation concepts with proper glossary and formatting rules. Translated content must follow consistent terminology standards across all languages. ✓ RESOLVED: Consistent file structure and frontmatter preservation ensures uniformity across languages.
- Lab & Hardware Guidelines: Verify hardware requirements (RTX PCs, Jetson kits, RealSense cameras, etc.) and cloud vs on-premise options are properly addressed. All guidelines must be accurately translated while preserving technical specifications. ✓ RESOLVED: Technical specifications preserved in translations with proper transliterations.
- Docusaurus Structure & Quality: Ensure chapters are navigable, searchable, and maintainable with proper metadata and interactive elements. Multilingual implementation must maintain navigation structure and search functionality across all languages. ✓ RESOLVED: Docusaurus i18n plugin ensures navigation and search work across all languages.
- Code & Simulation Quality: Verify all ROS 2 packages, Gazebo simulations, Isaac pipelines, and VLA code examples are reproducible with safety warnings included. Code examples remain in English with translated documentation. ✓ RESOLVED: Code examples remain in English (industry standard) with translated explanatory content.
- Multilingual Implementation & Cultural Sensitivity: All translated content (Urdu, Hindi) must preserve technical accuracy while being culturally appropriate and linguistically accurate for target audiences. RTL support must be properly implemented for Urdu content. ✓ RESOLVED: RTL support implemented for Urdu with proper CSS and cultural sensitivity in translations.

## Project Structure

### Documentation (this feature)

```text
specs/001-multilingual-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docusaurus/
├── docusaurus.config.mjs          # Docusaurus configuration with i18n settings
├── package.json                   # Dependencies including Docusaurus i18n plugin
├── docs/                          # English content (unchanged)
│   ├── 00-introduction/
│   ├── 01-ros2/
│   ├── 02-simulation/
│   ├── 03-isaac/
│   ├── 04-vla/
│   └── 05-capstone/
├── i18n/                          # New: Translation root
│   ├── ur/                        # New: Urdu locale
│   │   ├── code.json              # UI translations
│   │   ├── docusaurus-theme-classic/
│   │   │   ├── navbar.json
│   │   │   └── footer.json
│   │   └── docusaurus-plugin-content-docs/
│   │       └── current/           # Translated content
│   │           ├── 00-introduction/
│   │           ├── 01-ros2/
│   │           ├── 02-simulation/
│   │           ├── 03-isaac/
│   │           ├── 04-vla/
│   │           └── 05-capstone/
│   └── hi/                        # New: Hindi locale
│       ├── code.json
│       ├── docusaurus-theme-classic/
│       └── docusaurus-plugin-content-docs/
│           └── current/
│               ├── 00-introduction/
│               ├── 01-ros2/
│               ├── 02-simulation/
│               ├── 03-isaac/
│               ├── 04-vla/
│               └── 05-capstone/
├── src/
│   └── components/
│       ├── TranslateButton.tsx    # New: Language switching component
│       └── TranslateButton.module.css
├── static/
└── sidebars.js
```

**Structure Decision**: The feature requires extending the existing Docusaurus structure with i18n directories for Urdu and Hindi translations while preserving the original English content. The new TranslateButton component provides chapter-level language switching with proper styling.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
