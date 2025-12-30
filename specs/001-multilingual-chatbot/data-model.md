# Data Model: Multilingual Docusaurus + RAG Chatbot

## Message Entity
**Description**: Represents a chat message with multilingual support
- **id**: string (UUID) - Unique identifier for the message
- **type**: enum ['user', 'bot'] - Type of message sender
- **text**: string - Currently displayed text in the selected language
- **originalText**: string (optional) - Original text before translation (preserved for language switching)
- **language**: string - Language code of current text (ISO 639-1)
- **originalLanguage**: string (optional) - Language code of original text
- **timestamp**: number - Unix timestamp of message creation
- **sources**: array (optional) - Source documents for bot responses (RAG)
- **selectedContext**: string (optional) - Context from text selection for user messages

**Validation Rules**:
- id must be valid UUID format
- text must not exceed 5000 characters
- language must be one of supported languages: ['en', 'ur', 'ar', 'es', 'fr', 'de', 'zh', 'ja']
- timestamp must be current or past time

## ChatbotState Entity
**Description**: Manages the current conversation state
- **messages**: Message[] - Array of messages in the conversation
- **selectedLanguage**: string - Current chatbot language (ISO 639-1)
- **isTranslated**: boolean - Whether messages are currently translated
- **translationInProgress**: boolean - Whether translation is currently in progress
- **error**: string (optional) - Error message if any

**Validation Rules**:
- selectedLanguage must be one of supported languages
- messages array length must not exceed 1000 (prevent memory issues)

## TranslationCache Entity
**Description**: Stores previously translated text to improve performance
- **cacheKey**: string - Hash of (text + source_lang + target_lang)
- **translatedText**: string - The translated text result
- **sourceLanguage**: string - Original language code
- **targetLanguage**: string - Target language code
- **timestamp**: number - Unix timestamp of cache creation
- **ttl**: number - Time-to-live in seconds (typically 900 for 15 minutes)

**Validation Rules**:
- cacheKey must be unique
- ttl must be positive number
- timestamp + ttl must be in the future for valid cache entries

## LanguagePreference Entity
**Description**: Stores user's language preferences across sessions
- **language**: string - Preferred language code (ISO 639-1)
- **timestamp**: number - Unix timestamp of last update
- **version**: string - Schema version for migration purposes

**Validation Rules**:
- language must be one of supported languages
- version must follow semantic versioning

## TextSelectionContext Entity
**Description**: Handles selected text with language context for chatbot
- **selectedText**: string - The text that was selected by user
- **pageLanguage**: string - Language of the page where text was selected
- **textDirection**: enum ['ltr', 'rtl'] - Direction of the selected text
- **timestamp**: number - Unix timestamp of selection

**Validation Rules**:
- selectedText must not exceed 1000 characters
- pageLanguage must be one of supported languages
- textDirection must be either 'ltr' or 'rtl'

## API Request/Response Models

### Translation Request
- **text**: string (required, max 5000 chars) - Text to translate
- **target_language**: string (required) - Target language code (ISO 639-1)
- **source_language**: string (optional) - Source language code, auto-detect if missing
- **context**: string (optional) - Domain hint ('robotics', 'ai', 'technical')

### Translation Response
- **translated_text**: string - The translated text
- **source_language**: string - Detected/confirmed source language
- **target_language**: string - Target language that was used
- **model**: string - Model that performed the translation
- **confidence**: number (0-1) - Confidence score of translation
- **cached**: boolean - Whether the result was from cache

### Chat Request
- **message**: string - User message text
- **language**: string - Language code of the message
- **context**: string (optional) - Context from text selection
- **session_id**: string - Unique session identifier

### Chat Response
- **response**: string - Bot response text
- **language**: string - Language of the response
- **sources**: array - Source documents used for response (RAG)
- **translated**: boolean - Whether the response was translated