# Docusaurus Urdu Translation Setup

This project includes automated Urdu translation for Docusaurus documentation using OpenRouter AI.

## Features

- **Automatic Translation**: Translates all documentation markdown files from English to Urdu
- **Preserves Structure**: Maintains frontmatter, code blocks, and markdown formatting
- **Progress Tracking**: Keeps track of translation status and can resume interrupted processes
- **Error Handling**: Gracefully handles API errors and continues with other files
- **Rate Limiting**: Includes appropriate delays to respect API limits

## Setup

### Prerequisites

- Python 3.7+
- Node.js (for Docusaurus)
- OpenRouter API key (optional, uses default key if not provided)

### Installation

1. Install Python dependencies:
```bash
pip install requests
```

2. The script uses the same OpenRouter API key as your existing chatbot setup:
   - Default key: `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`
   - Or set environment variable: `OPENROUTER_API_KEY`

## Usage

### Translate Documentation

```bash
# Using npm script (recommended)
npm run translate-docs

# Or run directly with Python
python translate_docs.py

# With custom directories
python translate_docs.py --docs-dir docs --output-dir i18n/ur/docusaurus-plugin-content-docs/current

# With custom API key
python translate_docs.py --api-key your-openrouter-api-key
```

### Available Options

- `--docs-dir`: Source directory for English docs (default: `docs`)
- `--output-dir`: Target directory for Urdu docs (default: `i18n/ur/docusaurus-plugin-content-docs/current`)
- `--api-key`: OpenRouter API key (optional)

### Running the Website

After translation:

```bash
# Start English version
npm start

# Start Urdu version
npm run start:ur

# Build for production
npm run build:i18n
```

## How It Works

1. **Frontmatter Extraction**: Preserves YAML frontmatter (title, description, etc.)
2. **Code Block Preservation**: Extracts code blocks to prevent translation of code
3. **Content Translation**: Translates the actual content using OpenRouter API
4. **Structure Restoration**: Reconstructs the file with translated content and original structure
5. **Progress Tracking**: Saves progress to `translation_progress.json` to resume if interrupted

## Files Created/Modified

- `translate_docs.py` - Main translation script
- `translation_progress.json` - Translation status tracking
- `i18n/ur/docusaurus-plugin-content-docs/current/` - Translated documentation files

## Error Handling

- If translation fails for a file, it's logged and the process continues
- Failed files are marked in `translation_progress.json`
- Rate limiting prevents API abuse
- Progress is saved after each successful translation

## Supported Content Types

- Text content (translated)
- Frontmatter (preserved)
- Code blocks (preserved)
- Inline code (preserved)
- Markdown formatting (preserved)
- Links and images (preserved)

## Troubleshooting

### Common Issues

1. **API Key Issues**: If you get API errors, verify your OpenRouter API key
2. **Rate Limits**: The script includes delays to respect API limits
3. **File Encoding**: Script uses UTF-8 encoding for proper Urdu character support

### Resume Translation

If the process is interrupted, simply run the script again - it will skip already translated files.

### Check Progress

View `translation_progress.json` to see the status of each file.

## Technical Details

- Uses OpenRouter with Mistral DevStral model for translation
- Preserves Docusaurus-specific markdown features
- Handles RTL (right-to-left) text appropriately
- Maintains file structure and relative paths
- Idempotent operation (safe to run multiple times)