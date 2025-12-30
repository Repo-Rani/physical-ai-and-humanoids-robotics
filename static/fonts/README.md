# Font Directory

This directory is reserved for local font files. Currently, we're using Google Fonts via CDN for:

- Noto Nastaliq Urdu - For Urdu text
- Noto Sans Arabic - For Arabic text
- Other Noto fonts for additional language support

To use local fonts:

1. Download font files from Google Fonts or other sources
2. Place .woff2, .woff, .ttf files in this directory
3. Update docusaurus.config.js to reference local fonts instead of CDN
4. Update src/css/rtl.css to use local font paths