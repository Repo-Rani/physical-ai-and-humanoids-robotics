# Data Model: Documentation Translation Structure

## Documentation Page Entity
**Description**: Represents a single documentation page in both English and Urdu versions

**Fields**:
- `id`: Unique identifier for the page (matches file name without extension)
- `title`: Page title in respective language
- `content`: Markdown content in respective language
- `sidebar_label`: Navigation label in respective language
- `sidebar_position`: Position in sidebar navigation
- `description`: Meta description for SEO in respective language
- `keywords`: SEO keywords in respective language (technical terms remain in English)

## Language Configuration Entity
**Description**: Represents language configuration for the Docusaurus site

**Fields**:
- `locale`: Language code (e.g., 'en', 'ur')
- `label`: Display label for language switcher (e.g., 'English', 'اردو')
- `direction`: Text direction ('ltr' for English, 'rtl' for Urdu)
- `path`: URL path prefix for the language

## Navigation Structure Entity
**Description**: Represents the hierarchical navigation structure that must be preserved in translation

**Fields**:
- `category`: Documentation category/module
- `pages`: List of pages within the category
- `position`: Order of appearance in navigation
- `translations`: Mapping of equivalent pages in different languages

## Relationships
- One `Language Configuration` has many `Documentation Pages`
- One `Navigation Structure` contains many `Documentation Pages`
- Each `Documentation Page` exists in multiple languages (English/Urdu)

## Validation Rules
- All documentation pages must have corresponding translations
- Navigation structure must be preserved across languages
- Technical terms in content must follow constitution guidelines for accuracy
- SEO metadata must be properly localized
- Cross-references must point to correct language versions of linked pages