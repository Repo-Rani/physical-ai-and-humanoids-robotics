/**
 * Text Extractor Utility
 * Extracts translatable text from DOM while excluding code blocks, URLs, and other non-translatable content
 */

/**
 * Extract text content from DOM, excluding code blocks, URLs, and other non-translatable elements
 * @param {HTMLElement} container - DOM element to extract text from
 * @returns {Array<Object>} Array of objects containing text and corresponding DOM node
 */
export const extractTranslatableText = (container = document) => {
  if (!container) {
    return [];
  }

  // Create tree walker to traverse text nodes
  const walker = document.createTreeWalker(
    container,
    NodeFilter.SHOW_TEXT,
    {
      acceptNode: function(node) {
        const parent = node.parentElement;
        const text = node.textContent.trim();

        // Skip if text is empty
        if (!text) {
          return NodeFilter.FILTER_REJECT;
        }

        // Skip text inside code blocks, pre tags, links with URLs
        if (
          parent?.tagName === 'CODE' ||
          parent?.tagName === 'PRE' ||
          parent?.tagName === 'A' ||
          parent?.closest('code') ||
          parent?.closest('pre') ||
          parent?.closest('[class*="codeBlock"]') ||
          parent?.closest('[class*="code-block"]') ||
          parent?.closest('.codeBlockContainer') ||
          parent?.closest('.code-block-container') ||
          parent?.closest('kbd') ||
          parent?.closest('samp') ||
          parent?.closest('var') ||
          parent?.closest('mark') // Skip highlighted code
        ) {
          return NodeFilter.FILTER_REJECT;
        }

        // Skip if text looks like a URL, file path, or command
        if (
          text.includes('http://') ||
          text.includes('https://') ||
          text.includes('www.') ||
          text.match(/^[\w\-_]+\.js(on)?$/) || // file extensions
          text.match(/^[\w\-_]+\.py$/) ||
          text.match(/^[\w\-_]+\.md$/) ||
          text.match(/^\w+:\w+/) || // protocol-like patterns
          text.match(/^\$.*$/) || // command lines starting with $
          text.match(/^`.*`$/) // inline code
        ) {
          return NodeFilter.FILTER_REJECT;
        }

        return NodeFilter.FILTER_ACCEPT;
      }
    }
  );

  const texts = [];
  const nodes = [];

  let node;
  while (node = walker.nextNode()) {
    const text = node.textContent.trim();
    if (text) {
      texts.push(text);
      nodes.push(node);
    }
  }

  return { texts, nodes };
};

/**
 * Split text array into chunks for translation API
 * @param {Array<string>} texts - Array of text strings to chunk
 * @param {number} maxChunkSize - Maximum size of each chunk (default: 2000 characters)
 * @returns {Array<Object>} Array of chunk objects with text and original indices
 */
export const chunkTextsForTranslation = (texts, maxChunkSize = 2000) => {
  const chunks = [];
  let currentChunk = '';
  let currentIndices = [];

  for (let i = 0; i < texts.length; i++) {
    const text = texts[i];

    // If adding this text would exceed the chunk size and we already have content
    if (currentChunk.length + text.length > maxChunkSize && currentChunk) {
      // Save the current chunk
      chunks.push({
        text: currentChunk.trim(),
        indices: currentIndices
      });

      // Start a new chunk with the current text
      currentChunk = text;
      currentIndices = [i];
    } else {
      // Add to current chunk
      currentChunk += (currentChunk ? ' ' : '') + text;
      currentIndices.push(i);
    }
  }

  // Add the final chunk if it has content
  if (currentChunk) {
    chunks.push({
      text: currentChunk.trim(),
      indices: currentIndices
    });
  }

  return chunks;
};

/**
 * Restore translated chunks back to original text array structure
 * @param {Array<Object>} chunks - Array of chunk objects with text and indices
 * @param {number} totalTextCount - Total number of original text items
 * @returns {Array<string>} Array of translated text in original structure
 */
export const restoreChunkedTexts = (chunks, totalTextCount) => {
  const restored = new Array(totalTextCount);

  chunks.forEach(chunk => {
    const { text, indices } = chunk;

    // Simple approach: split the translated text back to match the number of original texts
    if (indices.length === 1) {
      restored[indices[0]] = text;
    } else {
      // More complex case: distribute the translated text across multiple original texts
      const textParts = splitTextIntoParts(text, indices.length);
      indices.forEach((originalIndex, partIndex) => {
        restored[originalIndex] = textParts[partIndex] || '';
      });
    }
  });

  return restored;
};

/**
 * Split a single text into multiple parts based on number of targets
 * @param {string} text - Text to split
 * @param {number} numParts - Number of parts to split into
 * @returns {Array<string>} Array of text parts
 */
const splitTextIntoParts = (text, numParts) => {
  if (numParts <= 1) return [text];

  const words = text.split(' ');
  const partSize = Math.ceil(words.length / numParts);
  const parts = [];

  for (let i = 0; i < numParts; i++) {
    const start = i * partSize;
    const end = Math.min((i + 1) * partSize, words.length);
    const part = words.slice(start, end).join(' ');
    parts.push(part);
  }

  return parts.filter(part => part.length > 0);
};

/**
 * Extract text from specific DOM elements while preserving structure
 * @param {Array<HTMLElement>} elements - Array of DOM elements to extract text from
 * @returns {Array<Object>} Array of objects with text, element, and node information
 */
export const extractTextFromElements = (elements) => {
  const results = [];

  elements.forEach(element => {
    const { texts, nodes } = extractTranslatableText(element);
    results.push({
      element,
      texts,
      nodes
    });
  });

  return results;
};

/**
 * Get page ID from current URL
 * @returns {string} Page ID for caching purposes
 */
export const getPageId = () => {
  const path = window.location.pathname;
  // Remove leading /docs/ and trailing slashes, replace / with _
  return path.replace('/docs/', '').replace(/\/$/, '').replace(/\//g, '_') || 'home';
};

/**
 * Get all translatable elements on the page
 * @param {HTMLElement} container - Container element to search in (default: document.body)
 * @returns {Array<HTMLElement>} Array of elements that contain translatable text
 */
export const getTranslatableElements = (container = document.body) => {
  // Get all elements that might contain translatable text
  const selector = 'article, .markdown, h1, h2, h3, h4, h5, h6, p, div, span, li, td, th, .container, .content';
  const allElements = container.querySelectorAll(selector);

  // Filter to only elements that actually have text content
  return Array.from(allElements).filter(element => {
    const text = element.textContent.trim();
    return text.length > 0 && !element.closest('code, pre, kbd, samp, var');
  });
};

/**
 * Extract and prepare content for translation
 * @param {HTMLElement} container - Container element to extract from
 * @returns {Object} Object containing texts, nodes, and page info
 */
export const prepareContentForTranslation = (container = document) => {
  const { texts, nodes } = extractTranslatableText(container);
  const pageId = getPageId();
  const chunks = chunkTextsForTranslation(texts);

  return {
    texts,
    nodes,
    pageId,
    chunks,
    totalChars: texts.reduce((sum, text) => sum + text.length, 0)
  };
};