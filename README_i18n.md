# Multilingual Docusaurus + Chatbot Setup

This document provides instructions for setting up and using the multilingual features of the Docusaurus documentation website with chatbot integration.

## Features

- **Docusaurus i18n**: English (en) and Urdu (ur) locales with RTL support
- **Chatbot Language Support**: English, Urdu, Arabic, Spanish, French, German, Chinese, Japanese
- **Real-time Translation**: Translate entire conversation history to any supported language
- **Text Selection Handler**: Automatically detects language when text is selected
- **RTL Layout Support**: Proper right-to-left rendering for Arabic and Urdu

## Setup Instructions

### 1. Docusaurus i18n Configuration

The website is configured with:
- Default locale: English (en)
- Secondary locale: Urdu (ur) with RTL direction
- Language switcher in the navbar

### 2. Running the Application

```bash
# Install dependencies
npm install

# Run English version
npm run start:en

# Run Urdu version
npm run start:ur

# Build for production (all locales)
npm run build:i18n

# Generate translation files
npm run i18n:write
npm run i18n:urdu
```

### 3. Backend API Setup

The backend includes a new translation endpoint:

```python
# Translation endpoint
POST /api/v1/translate
{
  "text": "Text to translate",
  "target_language": "ur"  # Supported: en, ur, ar, es, fr, de, zh, ja
}
```

### 4. Chatbot Multilingual Features

#### Language Selection
- Click the language dropdown button in the chatbot header
- Select from 8 supported languages
- Language preference is saved to localStorage

#### Conversation Translation
- Click the globe icon to translate the entire conversation
- Click the return icon to switch back to original language
- Original messages are preserved in memory

#### Text Selection
- Select text in any language (English, Urdu, Arabic, etc.)
- The floating "Ask AI about this" button appears
- Language is automatically detected and chatbot switches to that language

## Supported Languages

| Code | Language | RTL | Flag |
|------|----------|-----|------|
| en   | English  | No  | 🇺🇸  |
| ur   | اردو     | Yes | 🇵🇰  |
| ar   | العربية  | Yes | 🇸🇦  |
| es   | Español  | No  | 🇪🇸  |
| fr   | Français | No  | 🇫🇷  |
| de   | Deutsch  | No  | 🇩🇪  |
| zh   | 中文     | No  | 🇨🇳  |
| ja   | 日本語   | No  | 🇯🇵  |

## Configuration Files

### Docusaurus Configuration
- `docusaurus.config.js`: Contains i18n configuration
- `i18n/ur/code.json`: Urdu UI translations

### Chatbot Component
- `src/components/chatbot/chatbot.jsx`: Multilingual chatbot with translation features
- `src/components/chatbot/TextSelectionHandler.jsx`: Updated with language detection

### Backend API
- `backend/main.py`: Added `/api/v1/translate` endpoint

### Package Scripts
- Added i18n-related npm scripts in `package.json`

## Testing Instructions

1. **Language Switching**: Test switching between English and Urdu in the navbar
2. **Chatbot Language**: Test language selection in the chatbot
3. **Translation**: Test translating conversations to different languages
4. **Text Selection**: Select text in different languages and verify detection
5. **RTL Support**: Verify proper RTL rendering for Arabic and Urdu

## Adding New Languages

To add a new language:

1. Update `docusaurus.config.js` with the new locale
2. Create translation files in `i18n/[locale]/`
3. Add the language to the `languages` array in `chatbot.jsx`
4. Update the language detection regex in `TextSelectionHandler.jsx` if needed
5. Add the language to the translation mapping in `main.py`

## Troubleshooting

- If translations fail, verify the OpenRouter API key is valid
- If RTL doesn't work, ensure the language code is in the RTL array
- If language detection fails, check the character range regex patterns
- Clear localStorage to reset language preferences

## Technical Details

- Uses OpenRouter API for translations (Mistral DevStral model)
- Translation prompt maintains technical terminology
- Messages are stored with language metadata
- Original conversation is preserved during translation
- Responsive design supports both LTR and RTL layouts