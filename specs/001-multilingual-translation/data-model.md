# Data Model: Multi-Language Translation System

## Entity: LanguageContent
**Description**: Represents translated content for each supported language, with relationships to original English content by path structure

**Fields**:
- `locale`: string (en, ur, hi) - The language code for this content
- `path`: string - File path relative to content directory
- `content`: string - The translated content in markdown format
- `originalPath`: string - Reference to the original English content path
- `status`: enum (translated, in-progress, missing) - Translation status
- `lastUpdated`: datetime - When this translation was last updated

**Relationships**:
- Links to original English content via `originalPath`
- Belongs to a specific locale

## Entity: LocaleConfiguration
**Description**: Represents Docusaurus i18n configuration including locale settings, direction (LTR/RTL), and UI labels

**Fields**:
- `locale`: string (en, ur, hi) - The language code
- `label`: string - Display name for the language (e.g., "English", "اردو", "हिंदी")
- `direction`: enum (ltr, rtl) - Text direction (LTR for English/Hindi, RTL for Urdu)
- `htmlLang`: string - HTML language attribute (e.g., "en-US", "ur-PK", "hi-IN")
- `localizedUI`: object - Localized UI strings for navigation, buttons, etc.
- `rtlSupport`: boolean - Whether RTL styling is enabled for this locale

**Relationships**:
- Contains multiple LanguageContent entities
- Defines UI translations for the locale

## Entity: TranslationMapping
**Description**: Represents the relationship between original and translated content by file path and structure

**Fields**:
- `englishPath`: string - Path to original English content
- `urduPath`: string (optional) - Path to Urdu translation if available
- `hindiPath`: string (optional) - Path to Hindi translation if available
- `category`: string - Content category (e.g., "introduction", "ros2", "simulation")
- `sidebarPosition`: number - Position in sidebar navigation
- `status`: enum (complete, partial, missing) - Translation completeness

**Relationships**:
- Links to English, Urdu, and Hindi content paths
- Maintains navigation structure across languages

## Validation Rules
- All translated content must preserve original frontmatter (sidebar_position, title, description)
- Locale codes must be valid (en, ur, hi)
- Content paths must follow Docusaurus i18n directory structure conventions
- RTL locales (ur) must have proper text direction configuration
- Missing translations should fall back to English content

## State Transitions
- `missing` → `in-progress`: When translation work begins
- `in-progress` → `translated`: When translation is completed and reviewed
- `translated` → `in-progress`: When updates are needed to existing translation