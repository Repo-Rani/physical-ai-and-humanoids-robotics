# Quickstart: Urdu Translation Implementation

## Prerequisites
- Node.js 18+ and npm
- Docusaurus 3.0+ project setup
- Existing English documentation in `docs/` directory

## Setup Steps

### 1. Configure Docusaurus for Urdu i18n
Update `docusaurus.config.js` to add Urdu language support:

```javascript
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
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
};
```

### 2. Create Urdu Translation Directory Structure
```bash
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current/
```

### 3. Generate Urdu Translations
For each English documentation file in `docs/`, create a corresponding Urdu translation in `i18n/ur/docusaurus-plugin-content-docs/current/`:

```bash
# Example for intro.md
cp docs/intro.md i18n/ur/docusaurus-plugin-content-docs/current/intro.md
# Then translate the content in the Urdu file while preserving:
# - Frontmatter structure
# - Code blocks
# - Links to other documentation
# - Technical terms (keep in English where appropriate)
```

### 4. Build and Test
```bash
# Build the site with both languages
npm run build

# Start development server to test language switching
npm run start
```

## Verification Checklist
- [ ] All English pages have corresponding Urdu translations
- [ ] Language switcher appears in navigation bar
- [ ] Technical terminology preserved appropriately in Urdu versions
- [ ] Navigation and cross-references work in both languages
- [ ] Site builds without errors
- [ ] Urdu text displays correctly with proper RTL support

## Common Issues
1. **RTL Text Display**: Ensure proper CSS support for right-to-left text in Urdu
2. **Code Block Preservation**: Verify that all code snippets remain unchanged in translations
3. **Link Resolution**: Confirm that internal links point to correct language versions
4. **Frontmatter Consistency**: Ensure all metadata fields are properly maintained