# Quickstart Guide: Multi-Language Translation System

## Getting Started

This guide will help you set up and run the multilingual translation system for the Physical AI & Humanoid Robotics textbook.

## Prerequisites

- Node.js 18+ (for Docusaurus)
- npm or yarn package manager
- Git for version control
- Basic knowledge of Markdown and React (for custom components)

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-and-humanoid-robotics-hackathon
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Start Development Server
```bash
npm start
```

This will start the Docusaurus development server at `http://localhost:3000` with the default English content.

## Running with Different Languages

### Start with Urdu Content
```bash
npm run start:ur
# or
docusaurus start --locale ur
```

### Start with Hindi Content
```bash
npm run start:hi
# or
docusaurus start --locale hi
```

### Build for Production (All Languages)
```bash
npm run build
# This builds all locales
```

### Build Specific Language
```bash
npm run build:ur  # Build only Urdu
npm run build:hi  # Build only Hindi
```

## Key Features

### 1. Language Switching
- **Navbar Dropdown**: Located in the top right, allows switching between English, Urdu, and Hindi
- **Chapter-Level Button**: The "TranslateButton" component at the start of each chapter for granular language control

### 2. RTL Support
- Urdu content automatically displays with right-to-left text direction
- Navigation and layout adapt to RTL conventions
- CSS includes `[dir='rtl']` selectors for proper styling

### 3. Content Structure
- English content remains in `/docs/` (unchanged)
- Urdu translations in `/i18n/ur/docusaurus-plugin-content-docs/current/`
- Hindi translations in `/i18n/hi/docusaurus-plugin-content-docs/current/`

## Adding New Translations

### 1. Create Translation File
For a new English page at `/docs/introduction/new-topic.md`, create:
- Urdu: `/i18n/ur/docusaurus-plugin-content-docs/current/introduction/new-topic.md`
- Hindi: `/i18n/hi/docusaurus-plugin-content-docs/current/introduction/new-topic.md`

### 2. Preserve Frontmatter
Ensure translated files include the same frontmatter as the original:

```yaml
---
sidebar_position: 3
title: "Title in Target Language"
description: "Description in Target Language"
---
```

### 3. Maintain Content Structure
- Keep the same headings and document structure
- Preserve all code examples (these remain in English)
- Translate explanatory text, headings, and descriptions
- Keep technical terms with appropriate transliterations

## Configuration

### Docusaurus Configuration (`docusaurus.config.mjs`)
The i18n configuration is set up in the main config file:

```javascript
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'hi'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
      hi: {
        label: 'हिंदी',
        direction: 'ltr',
      },
    },
  },
};
```

## Troubleshooting

### Language Switching Not Working
1. Verify locale configuration in `docusaurus.config.mjs`
2. Check that translation files exist in the correct `/i18n/{locale}/` directories
3. Ensure frontmatter is properly preserved in translated files

### RTL Layout Issues
1. Check that Urdu locale has `direction: 'rtl'` in config
2. Verify CSS includes RTL-specific selectors
3. Test in different browsers to ensure consistency

### Build Errors
1. Run `npm run build` to identify specific issues
2. Verify all required translation files exist
3. Check for syntax errors in Markdown or frontmatter

## Development Tips

### Working with the TranslateButton Component
The custom `TranslateButton.tsx` component handles language switching. When modifying:
- Test in all supported languages
- Ensure RTL styles work properly for Urdu
- Verify navigation persistence across language switches

### Testing Translation Completeness
- Use the fallback mechanism to identify missing translations
- Verify all navigation links work in each language
- Test chapter-level and global language switching