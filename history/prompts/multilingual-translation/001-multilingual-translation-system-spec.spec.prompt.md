---
id: "001"
title: "Multilingual Translation System Spec"
stage: "spec"
date_iso: "2026-01-05"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "multilingual-translation"
branch: "001-multilingual-translation"
user: "HP"
command: "/sp.specify"
labels:
  - "spec"
  - "i18n"
  - "multilingual"
  - "docusaurus"
links:
  SPEC: "specs/001-multilingual-translation/spec.md"
  TICKET: "null"
  ADR: "null"
  PR: "null"
files_yaml: |
  - C:\\Users\\HP\\Desktop\\ai-and-humanoid-robotics-hackathon\\specs\\001-multilingual-translation\\spec.md
  - C:\\Users\\HP\\Desktop\\ai-and-humanoid-robotics-hackathon\\specs\\001-multilingual-translation\\checklists\\requirements.md
tests_yaml: |
  - "None"
outcome: "Successfully created comprehensive specification for multilingual translation system"
evaluation: "Specification includes user scenarios, functional requirements, and success criteria for English/Urdu/Hindi translation system"
---

# Prompt History Record: Multilingual Translation System Spec

## Original Prompt

```
# Multi-Language Translation System Specification

## Document Information
- **Project**: Physical AI & Humanoid Robotics Textbook
- **Feature**: Multi-Language Translation System
- **Version**: 1.0.0
- **Date**: December 30, 2025
- **Author**: System Architect
- **Status**: Implementation Ready

---

## 1. Executive Summary

### 1.1 Overview
Implementation of a comprehensive multi-language translation system for the Physical AI & Humanoid Robotics educational platform. The system will support English (existing), Urdu, and Hindi languages with seamless switching capabilities while mainta├── ContentArea
│   ├── TranslateButton ← NEW
│   ├── ReadingTime
│   ├── PersonalizeButton
│   └── ChapterContent
└── Footer
```

### 2.2 File Structure Specification

```
physical-ai-robotics-textbook/
├── docusaurus/
│   ├── docusaurus.config.mjs          # MODIFY: Add i18n config
│   ├── package.json                    # MODIFY: Add i18n dependencies
│   │
│   ├── docs/                           # PROTECTED: Do not modify
│   │   ├── 00-introduction/
│   │   │   ├── 01-welcome.md
│   │   │   ├── 02-prerequisites.md
│   │   │   ├── 03-hardware-requirements.md
│   │   │   ├── 04-how-to-use.md
│   │   │   ├── 05-syllabus.md
│   │   │   ├── index.md
│   │   │   └── _category_.json
│   │   ├── 01-ros2/
│   │   │   ├── 01-architecture.md
│   │   │   ├── 02-nodes-topics.md
│   │   │   ├── 03-services-actions.md
│   │   │   ├── 04-python-packages.md
│   │   │   ├── 05-urdf-humanoids.md
│   │   │   ├── 06-launch-files.md
│   │   │   ├── index.md
│   │   │   └── _category_.json
│   │   ├── 02-simulation/
│   │   ├── 03-isaac/
│   │   ├── 04-vla/
│   │   ├── 05-capstone/
│   │   └── index.md
│   │
│   ├── i18n/                           # NEW: Translation root
│   │   ├── ur/                         # NEW: Urdu locale
│   │   │   ├── code.json               # NEW: UI translations
│   │   │   ├── docusaurus-theme-classic/
│   │   │   │   ├── navbar.json
│   │   │   │   ├── footer.json
│   │   │   │   └── ...
│   │   │   └── docusaurus-plugin-content-docs/
│   │   │       └── current/
│   │   │           ├── 00-introduction/
│   │   │           │   ├── 01-welcome.md
│   │   │           │   ├── 02-prerequisites.md
│   │   │           │   ├── 03-hardware-requirements.md
│   │   │           │   ├── 04-how-to-use.md
│   │   │           │   ├── 05-syllabus.md
│   │   │           │   ├── index.md
│   │   │           │   └── _category_.json
│   │   │           ├── 01-ros2/
│   │   │           │   ├── 01-architecture.md
│   │   │           │   ├── 02-nodes-topics.md
│   │   │           │   ├── 03-services-actions.md
│   │   │           │   ├── 04-python-packages.md
│   │   │           │   ├── 05-urdf-humanoids.md
│   │   │           │   ├── 06-launch-files.md
│   │   │           │   ├── index.md
│   │   │           │   └── _category_.json
│   │   │           ├── 02-simulation/
│   │   │           ├── 03-isaac/
│   │   │           ├── 04-vla/
│   │   │           ├── 05-capstone/
│   │   │           └── index.md
│   │   │
│   │   └── hi/                         # NEW: Hindi locale
│   │       ├── code.json
│   │       ├── docusaurus-theme-classic/
│   │       │   └── ...
│   │       └── docusaurus-plugin-content-docs/
│   │           └── current/
│   │               └── (same structure as ur/)
│   │
│   ├── src/
│   │   ├── components/
│   │   │   ├── TranslateButton.tsx     # NEW
│   │   │   ├── LanguageDropdown.tsx    # NEW (optional)
│   │   │   ├── PersonalizeButton.tsx   # PROTECTED
│   │   │   ├── Chatbot.tsx             # PROTECTED
│   │   │   └── ...
│   │   │
│   │   ├── css/
│   │   │   ├── custom.css              # MODIFY: Add RTL styles
│   │   │   └── translation.css         # NEW
│   │   │
│   │   └── utils/
│   │       └── i18n.ts                 # NEW: Translation utilities
│   │
│   ├── staticmCss: [
            './src/css/custom.css',
            './src/css/translation.css', // NEW
          ],
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        // ==================== NEW: Language Dropdown ====================
        {
          type: 'localeDropdown',
          position: 'right',
          dropdownItemsAfter: [],
          className: 'language-dropdown-custom',
          queryString: '?persistLocale=true',
        },
        // ================================================================
        // Add spacing before auth buttons
        {
          type: 'html',
          position: 'right',
          value: '<div style="width: 20px;"></div>',
        },
        {
          to: '/login',
          label: 'Login',
          position: 'right',
          className: 'navbar-login-btn',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'navbar-signup-btn',
        },
      ],
    },

    // Color mode configuration
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },

  // ==================== NEW: Custom Webpack Config ====================
  webpack: {
    jsLoader: (isServer) => ({
      loader: require.resolve('swc-loader'),
      options: {
        jsc: {
          parser: {
            syntax: 'typescript',
            tsx: true,
          },
          target: 'es2017',
        },
        module: {
          type: isServer ? 'commonjs' : 'es6',
        },
      },
    }),
  },
  // ====================================================================

  // ==================== NEW: Plugin Configuration ====================
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs',
        path: 'docs',
        routeBasePath: 'docs',
        sidebarPath: require.resolve('./sidebars.js'),
        remarkPlugins: [],
        rehypePlugins: [],
      },
    ],
  ],
  // ===================================================================
};
```

#### 3.1.2 Package.json Updates

```json
{
  "name": "physical-ai-robotics-textbook",
  "version": "1.0.0",
  "private": true,
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "swizzle": "docusaurus swizzle",
    "deploy": "docusaurus deploy",
    "clear": "docusaurus clear",
    "serve": "docusaurus serve",
    "write-translations": "docusaurus write-translations",
    "write-heading-ids": "docusaurus write-heading-ids",
    "start:ur": "docusaurus start --locale ur",
    "start:hi": "docusaurus start --locale hi",
    "build:ur": "docusaurus build --locale ur",
    "build:hi": "docusaurus build --locale hi"
  },
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "@docusaurus/theme-live-codeblock": "^3.0.0",
    "@mdx-js/react": "^3.0.0",
    "clsx": "^2.0.0",
    "prism-react-renderer": "^2.1.0",
    "react": "^18.2.0",
    "react-dom": "^18.2.0"
  },
  "devDependencies": {
    "@docusaurus/module-type-aliases": "^3.0.0",
    "@docusaurus/types": "^3.0.0",
    "typescript": "^5.2.2"
  },
  "browserslist": {
    "production": [
      ">0.5%",
      "not dead",
      "not op_mini all"
    ],
    "development": [
      "last 1 chrome version",
      "last 1 firefox version",
      "last 1 safari version"
    ]
  },
  "engines": {
    "node": ">=18.0"
  }
}
```

### 3.2 Component Specifications

#### 3.2.1 TranslateButton Component

```typescript
// File: src/components/TranslateButton.tsx

import React, { useState, useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './TranslateButton.module.css';

interface TranslateButtonProps {
  compact?: boolean;
  showAllLanguages?: boolean;
}

export default function TranslateButton({
  compact = false,
  showAllLanguages = true
}: TranslateButtonProps): JSX.Element {
  const { i18n } = useDocusaurusContext();
  const history = useHistory();
  const location = useLocation();
  const [currentLocale, setCurrentLocale] = useState(i18n.currentLocale);

  useEffect(() => {
    setCurrentLocale(i18n.currentLocale);
  }, [i18n.currentLocale]);

  const switchLanguage = (targetLocale: string) => {
    if (targetLocale === currentLocale) return;

    // Get current path without locale prefix
    const pathWithoutLocale = location.pathname.replace(
      /^\/(en|ur|hi)\//,
      '/'
    );

    // Construct new path with target locale
    const newPath = targetLocale === 'en'
      ? pathWithoutLocale
      : `/${targetLocale}${pathWithoutLocale}`;

    // Save preference to localStorage
    localStorage.setItem('preferredLocale', targetLocale);

    // Navigate to new path
    history.push(newPath);
  };

  const languages = [
    { code: 'en', label: 'English', nativeLabel: 'English', flag: '🇬🇧' },
    { code: 'ur', label: 'Urdu', nativeLabel: 'اردو', flag: '🇵🇰' },
    { code: 'hi', label: 'Hindi', nativeLabel: 'हिंदी', flag: '🇮🇳' },
  ];

  const otherLanguages = languages.filter(lang => lang.code !== currentLocale);

  if (compact) {
    return (
      <div className={styles.translateButtonCompact}>
        {otherLanguages.map((lang) => (
          <button
            key={lang.code}
            className={styles.compactButton}
            onClick={() => switchLanguage(lang.code)}
            aria-label={`Switch to ${lang.label}`}
          >
            {lang.flag} {lang.nativeLabel}
          </button>
        ))}
      </div>
    );
  }

  return (
    <div className={styles.translateButtonContainer}>
      <div className={styles.translateButtonWrapper}>
        <div className={styles.currentLanguage}>
          <span className={styles.languageIcon}>🌐</span>
          <span className={styles.languageText}>
            {currentLocale === 'en' && 'Reading in English'}
            {currentLocale === 'ur' && 'اردو میں پڑھ رہے ہیں'}
            {currentLocale === 'hi' && 'हिंदी में पढ़ रहे हैं'}
          </span>
        </div>

        {showAllLanguages && (
          <div className={styles.languageButtons}>
            {otherLanguages.map((lang) => (
              <button
                key={lang.code}
                className={styles.languageButton}
                onClick={() => switchLanguage(lang.code)}
                aria-label={`Switch to ${lang.label}`}
              >
                <span className={styles.buttonFlag}>{lang.flag}</span>
                <span className={styles.buttonText}>
                  {lang.code === 'ur' && 'اردو میں پڑھیں'}
                  {lang.code === 'hi' && 'हिंदी में पढ़ें'}
                  {lang.code === 'en' && 'Read in English'}
                </span>
              </button>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}
```

#### 3.2.2 TranslateButton Styles

```css
/* File: src/components/TranslateButton.module.css */

.translateButtonContainer {
  margin: 2rem 0;
  padding: 1.5rem;
  background: linear-gradient(135deg, #fff5f0 0%, #ffffff 100%);
  border-radius: 12px;
  border: 2px solid #ff6b35;
  box-shadow: 0 4px 6px rgba(255, 107, 53, 0.1);
}

.translateButtonWrapper {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.currentLanguage {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-size: 1rem;
  color: #333;
  font-weight: 500;
}

.languageIcon {
  font-size: 1.5rem;
}

.languageText {
  font-size: 1.1rem;
}

.languageButtons {
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
}

.languageButton {
  flex: 1;
  min-width: 150px;
  padding: 0.75rem 1.5rem;
  background: linear-gradient(135deg, #ff6b35 0%, #ff8c42 100%);
  color: white;
  border: none;
  border-radius: 8px;
  font-size: 1rem;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  box-shadow: 0 2px 4px rgba(255, 107, 53, 0.2);
}

.languageButton:hover {
  background: linear-gradient(135deg, #ff8c42 0%, #ff6b35 100%);
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(255, 107, 53, 0.3);
}

.languageButton:active {
  transform: translateY(0);
}

.buttonFlag {
  font-size: 1.2rem;
}

.buttonText {
  font-size: 1rem;
}

/* Compact variant */
.translateButtonCompact {
  display: inline-flex;
  gap: 0.5rem;
  margin: 0.5rem 0;
}

.compactButton {
  padding: 0.5rem 1rem;
  background: linear-gradient(135deg, #ff6b35 0%, #ff8c42 100%);
  color: white;
  border: none;
  border-radius: 6px;
  font-size: 0.9rem;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  display: flex;
  align-items: center;
  gap: 0.3rem;
}

.compactButton:hover {
  background: linear-gradient(135deg, #ff8c42 0%, #ff6b35 100%);
  transform: scale(1.05);
}

/* RTL Support */
[dir='rtl'] .translateButtonWrapper {
  direction: rtl;
}

[dir='rtl'] .currentLanguage {
  flex-direction: row-reverse;
}

[dir='rtl'] .languageButton {
  flex-direction: row-reverse;
}

/* Responsive Design */
@media (max-width: 768px) {
  .translateButtonContainer {
    padding: 1rem;
  }

  .languageButtons {
    flex-direction: column;
  }

  .languageButton {
    min-width: 100%;
  }
}

/* Dark Mode Support */
[data-theme='dark'] .translateButtonContainer {
  background: linear-gradient(135deg, #2a2a2a 0%, #1a1a1a 100%);
  border-color: #ff8c42;
}

[data-theme='dark'] .currentLanguage {
  color: #e0e0e0;
}

/* Animation */
@keyframes slideIn {
  from {
    opacity: 0;
    transform: translateY(-10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.translateButtonContainer {
  animation: slideIn 0.3s ease-out;
}
```

### 3.3 CSS Specifications

#### 3.3.1 RTL Support Styles

```css
/* File: src/css/translation.css */

/* ====================== RTL Support ====================== */

/* Urdu RTL Support */
[dir='rtl'] {
  direction: rtl;
  text-align: right;
}

[dir='rtl'] .navbar__items {
  flex-direction: row-reverse;
}

[dir='rtl'] .navbar__item {
  padding-right: 0;
  padding-left: var(--ifm-navbar-item-padding-horizontal);
}

[dir='rtl'] .menu {
  padding-right: 0;
  padding-left:ecification provides comprehensive, implementation-ready guidelines for the multi-language translation system. All sections are designed to be actionable and measurable, ensuring successful project completion and hackathon bonus point achievement.
```

## Response Summary

Successfully created a comprehensive feature specification for the multilingual translation system that supports English, Urdu, and Hindi languages. The specification includes:

1. User scenarios and testing requirements with prioritized user stories
2. Functional requirements covering language support, content preservation, UI elements, and system behavior
3. Success criteria with measurable outcomes
4. Key entities and data structures
5. Edge cases and implementation constraints

The specification follows the template structure and includes all mandatory sections. It focuses on user value and business needs while maintaining technology-agnostic requirements that can be implemented with various approaches. The specification also includes a quality checklist to validate completeness before proceeding to the planning phase.