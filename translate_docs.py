#!/usr/bin/env python3
"""
Docusaurus Urdu Translation Script

This script translates English documentation files to Urdu for a Docusaurus website.
It preserves frontmatter, code blocks, and markdown structure while translating
the content using OpenRouter API.
"""

import os
import re
import json
import time
import requests
from pathlib import Path
from typing import Dict, List, Optional
import argparse
import logging
from urllib.parse import urlparse

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DocusaurusUrduTranslator:
    def __init__(self, openrouter_api_key: str = None, base_url: str = "https://openrouter.ai/api/v1"):
        """
        Initialize the translator with OpenRouter API credentials.

        Args:
            openrouter_api_key: OpenRouter API key (defaults to environment variable)
            base_url: OpenRouter API base URL
        """
        self.api_key = openrouter_api_key or os.getenv("OPENROUTER_API_KEY", "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")
        self.base_url = base_url
        self.model = "mistralai/devstral-2512:free"
        self.headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        # Track translation progress
        self.progress_file = "translation_progress.json"
        self.translated_files = self._load_progress()

    def _load_progress(self) -> Dict[str, str]:
        """Load translation progress from file."""
        if os.path.exists(self.progress_file):
            try:
                with open(self.progress_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                logger.warning(f"Could not load progress file: {e}")
                return {}
        return {}

    def _save_progress(self):
        """Save translation progress to file."""
        try:
            with open(self.progress_file, 'w', encoding='utf-8') as f:
                json.dump(self.translated_files, f, ensure_ascii=False, indent=2)
        except Exception as e:
            logger.error(f"Could not save progress file: {e}")

    def _extract_frontmatter(self, content: str) -> tuple:
        """
        Extract frontmatter from markdown content.

        Args:
            content: Raw markdown content

        Returns:
            tuple: (frontmatter, content_without_frontmatter)
        """
        if content.startswith('---'):
            # Find the end of frontmatter
            parts = content.split('---', 2)
            if len(parts) >= 3:
                frontmatter = parts[1].strip()
                content_without_frontmatter = parts[2].strip()
                return frontmatter, content_without_frontmatter

        return "", content

    def _preserve_code_blocks(self, content: str) -> tuple:
        """
        Extract and preserve code blocks from content.

        Args:
            content: Markdown content

        Returns:
            tuple: (content_with_placeholders, code_blocks_dict)
        """
        code_blocks = {}
        placeholder_pattern = r'```.*?\n.*?\n```'
        code_matches = list(re.finditer(placeholder_pattern, content, re.DOTALL))

        # Replace code blocks with placeholders
        content_with_placeholders = content
        for i, match in enumerate(code_matches):
            placeholder = f"__CODE_BLOCK_{i}__"
            code_blocks[placeholder] = match.group(0)
            content_with_placeholders = content_with_placeholders.replace(
                match.group(0),
                placeholder,
                1
            )

        # Also handle inline code blocks
        inline_code_pattern = r'`[^`]*`'
        inline_code_matches = list(re.finditer(inline_code_pattern, content_with_placeholders))

        for i, match in enumerate(inline_code_matches, start=len(code_matches)):
            placeholder = f"__INLINE_CODE_{i}__"
            code_blocks[placeholder] = match.group(0)
            content_with_placeholders = content_with_placeholders.replace(
                match.group(0),
                placeholder,
                1
            )

        return content_with_placeholders, code_blocks

    def _restore_code_blocks(self, content: str, code_blocks: Dict[str, str]) -> str:
        """Restore code blocks from placeholders."""
        result = content
        for placeholder, code_block in code_blocks.items():
            result = result.replace(placeholder, code_block)
        return result

    def _translate_text(self, text: str) -> Optional[str]:
        """
        Translate text using OpenRouter API.

        Args:
            text: Text to translate

        Returns:
            Translated text or None if failed
        """
        if not text.strip():
            return text

        try:
            # Construct translation prompt
            prompt = f"""Translate the following text to Urdu (اردو).
Maintain technical terminology where appropriate.
Preserve markdown formatting and structure.
Only return the translated text, no explanations.

Text: {text}"""

            # Prepare API request
            payload = {
                "model": self.model,
                "messages": [
                    {
                        "role": "system",
                        "content": "You are a professional translator. Translate text accurately while maintaining the meaning and markdown formatting. Only return the translated text without any additional explanations."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                "temperature": 0.3,
                "max_tokens": 2000
            }

            # Make API request
            response = requests.post(
                f"{self.base_url}/chat/completions",
                headers=self.headers,
                json=payload
            )

            if response.status_code == 200:
                result = response.json()
                translated_text = result['choices'][0]['message']['content']
                logger.info(f"Successfully translated {len(text)} characters")
                return translated_text
            else:
                logger.error(f"Translation API error: {response.status_code} - {response.text}")
                return None

        except Exception as e:
            logger.error(f"Translation error: {e}")
            return None

    def _translate_content(self, content: str) -> Optional[str]:
        """
        Translate markdown content while preserving structure.

        Args:
            content: Raw markdown content

        Returns:
            Translated content or None if failed
        """
        # Extract frontmatter
        frontmatter, content_without_frontmatter = self._extract_frontmatter(content)

        # Preserve code blocks
        content_with_placeholders, code_blocks = self._preserve_code_blocks(content_without_frontmatter)

        # Translate the content
        translated_content = self._translate_text(content_with_placeholders)

        if translated_content is None:
            return None

        # Restore code blocks
        final_content = self._restore_code_blocks(translated_content, code_blocks)

        # Reconstruct with frontmatter
        if frontmatter:
            final_content = f"---\n{frontmatter}\n---\n\n{final_content}"

        return final_content

    def translate_file(self, source_path: str, target_path: str) -> bool:
        """
        Translate a single markdown file.

        Args:
            source_path: Path to source English file
            target_path: Path to target Urdu file

        Returns:
            True if successful, False otherwise
        """
        try:
            # Check if file is already translated
            if source_path in self.translated_files and self.translated_files[source_path] == "completed":
                logger.info(f"Skipping {source_path} - already translated")
                return True

            logger.info(f"Translating: {source_path} -> {target_path}")

            # Read source file
            with open(source_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Translate content
            translated_content = self._translate_content(content)

            if translated_content is None:
                logger.error(f"Failed to translate {source_path}")
                self.translated_files[source_path] = "failed"
                self._save_progress()
                return False

            # Ensure target directory exists
            target_dir = os.path.dirname(target_path)
            os.makedirs(target_dir, exist_ok=True)

            # Write translated file
            with open(target_path, 'w', encoding='utf-8') as f:
                f.write(translated_content)

            logger.info(f"Successfully translated {source_path}")
            self.translated_files[source_path] = "completed"
            self._save_progress()

            # Rate limiting
            time.sleep(1)

            return True

        except Exception as e:
            logger.error(f"Error translating {source_path}: {e}")
            self.translated_files[source_path] = "failed"
            self._save_progress()
            return False

    def translate_directory(self, source_dir: str, target_dir: str) -> Dict[str, bool]:
        """
        Translate all markdown files in a directory structure.

        Args:
            source_dir: Source directory with English files
            target_dir: Target directory for Urdu files

        Returns:
            Dictionary with file paths and success status
        """
        results = {}

        # Walk through source directory
        for root, dirs, files in os.walk(source_dir):
            for file in files:
                if file.endswith('.md'):
                    source_path = os.path.join(root, file)

                    # Calculate target path
                    rel_path = os.path.relpath(source_path, source_dir)
                    target_path = os.path.join(target_dir, rel_path)

                    success = self.translate_file(source_path, target_path)
                    results[source_path] = success

        return results

    def translate_all_docs(self, docs_dir: str = "docs", output_dir: str = "i18n/ur/docusaurus-plugin-content-docs/current"):
        """
        Translate all documentation files.

        Args:
            docs_dir: Source documentation directory
            output_dir: Target output directory for translated files
        """
        logger.info(f"Starting translation of documentation from {docs_dir} to {output_dir}")

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # Translate all markdown files
        results = self.translate_directory(docs_dir, output_dir)

        # Print summary
        total = len(results)
        successful = sum(1 for success in results.values() if success)
        failed = total - successful

        logger.info(f"Translation completed: {successful}/{total} files successful, {failed} failed")

        if failed > 0:
            logger.warning("Failed files:")
            for file_path, success in results.items():
                if not success:
                    logger.warning(f"  - {file_path}")

def main():
    parser = argparse.ArgumentParser(description='Translate Docusaurus documentation to Urdu')
    parser.add_argument('--docs-dir', default='docs', help='Source documentation directory (default: docs)')
    parser.add_argument('--output-dir', default='i18n/ur/docusaurus-plugin-content-docs/current',
                       help='Target output directory (default: i18n/ur/docusaurus-plugin-content-docs/current)')
    parser.add_argument('--api-key', help='OpenRouter API key (optional, defaults to environment variable)')

    args = parser.parse_args()

    # Initialize translator
    translator = DocusaurusUrduTranslator(openrouter_api_key=args.api_key)

    # Perform translation
    translator.translate_all_docs(args.docs_dir, args.output_dir)

    print("\n Urdu translation process completed!")
    print(f"Check {translator.progress_file} for translation progress details.")

if __name__ == "__main__":
    main()