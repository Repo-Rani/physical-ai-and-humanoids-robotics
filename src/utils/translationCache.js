/**
 * Translation Cache Utility Functions
 * Handles caching of translated content in localStorage
 */

// Cache key prefix for documentation translations
const DOC_TRANSLATION_PREFIX = 'doc_translation_';
const ORIGINAL_CONTENT_PREFIX = 'original_content_';
const PREFERRED_DOC_LANGUAGE_KEY = 'preferred_doc_language';
const CHATBOT_MESSAGES_KEY = 'chatbot_messages';
const CHATBOT_LANGUAGE_KEY = 'chatbot_language';

/**
 * Get cached translation for a specific page and language
 * @param {string} pageId - Unique identifier for the page
 * @param {string} language - Target language code (e.g., 'ur', 'ar')
 * @returns {object|null} Cached translation data or null if not found
 */
export const getTranslationCache = (pageId, language) => {
  try {
    const cacheKey = `${DOC_TRANSLATION_PREFIX}${pageId}_${language}`;
    const cachedData = localStorage.getItem(cacheKey);

    if (cachedData) {
      const parsed = JSON.parse(cachedData);
      // Check if cache is still valid (not expired)
      const maxAge = 7 * 24 * 60 * 60 * 1000; // 7 days in milliseconds
      const isExpired = parsed.timestamp && (Date.now() - parsed.timestamp) > maxAge;

      if (!isExpired) {
        return parsed;
      } else {
        // Remove expired cache
        localStorage.removeItem(cacheKey);
        return null;
      }
    }
    return null;
  } catch (error) {
    console.error('Error getting translation cache:', error);
    return null;
  }
};

/**
 * Set cached translation for a specific page and language
 * @param {string} pageId - Unique identifier for the page
 * @param {string} language - Target language code
 * @param {object} translationData - Translation data to cache
 */
export const setTranslationCache = (pageId, language, translationData) => {
  try {
    const cacheKey = `${DOC_TRANSLATION_PREFIX}${pageId}_${language}`;
    const cacheData = {
      ...translationData,
      timestamp: Date.now()
    };
    localStorage.setItem(cacheKey, JSON.stringify(cacheData));
  } catch (error) {
    console.error('Error setting translation cache:', error);
  }
};

/**
 * Get original content for a page (before translation)
 * @param {string} pageId - Unique identifier for the page
 * @returns {array|null} Original content array or null if not found
 */
export const getOriginalContent = (pageId) => {
  try {
    const cacheKey = `${ORIGINAL_CONTENT_PREFIX}${pageId}`;
    const originalContent = localStorage.getItem(cacheKey);
    return originalContent ? JSON.parse(originalContent) : null;
  } catch (error) {
    console.error('Error getting original content:', error);
    return null;
  }
};

/**
 * Set original content for a page (before translation)
 * @param {string} pageId - Unique identifier for the page
 * @param {array} originalContent - Original content array to store
 */
export const setOriginalContent = (pageId, originalContent) => {
  try {
    const cacheKey = `${ORIGINAL_CONTENT_PREFIX}${pageId}`;
    localStorage.setItem(cacheKey, JSON.stringify(originalContent));
  } catch (error) {
    console.error('Error setting original content:', error);
  }
};

/**
 * Clear translation cache for a specific page and language
 * @param {string} pageId - Unique identifier for the page
 * @param {string} language - Target language code
 */
export const clearTranslationCache = (pageId, language) => {
  try {
    const cacheKey = `${DOC_TRANSLATION_PREFIX}${pageId}_${language}`;
    localStorage.removeItem(cacheKey);
  } catch (error) {
    console.error('Error clearing translation cache:', error);
  }
};

/**
 * Clear all translation caches for a specific page
 * @param {string} pageId - Unique identifier for the page
 */
export const clearPageCache = (pageId) => {
  try {
    // Get all keys that start with the page prefix
    const keysToRemove = [];
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key && key.startsWith(`${DOC_TRANSLATION_PREFIX}${pageId}_`)) {
        keysToRemove.push(key);
      }
    }

    keysToRemove.forEach(key => localStorage.removeItem(key));
  } catch (error) {
    console.error('Error clearing page cache:', error);
  }
};

/**
 * Get preferred documentation language
 * @returns {string|null} Preferred language code or null if not set
 */
export const getPreferredDocLanguage = () => {
  try {
    return localStorage.getItem(PREFERRED_DOC_LANGUAGE_KEY);
  } catch (error) {
    console.error('Error getting preferred doc language:', error);
    return null;
  }
};

/**
 * Set preferred documentation language
 * @param {string} language - Language code to set as preferred
 */
export const setPreferredDocLanguage = (language) => {
  try {
    localStorage.setItem(PREFERRED_DOC_LANGUAGE_KEY, language);
  } catch (error) {
    console.error('Error setting preferred doc language:', error);
  }
};

/**
 * Get cached chatbot messages
 * @returns {array|null} Array of chatbot messages or null if not found
 */
export const getChatbotMessages = () => {
  try {
    const messages = localStorage.getItem(CHATBOT_MESSAGES_KEY);
    return messages ? JSON.parse(messages) : null;
  } catch (error) {
    console.error('Error getting chatbot messages:', error);
    return null;
  }
};

/**
 * Set cached chatbot messages
 * @param {array} messages - Array of chatbot messages to store
 */
export const setChatbotMessages = (messages) => {
  try {
    localStorage.setItem(CHATBOT_MESSAGES_KEY, JSON.stringify(messages));
  } catch (error) {
    console.error('Error setting chatbot messages:', error);
  }
};

/**
 * Get preferred chatbot language
 * @returns {string|null} Preferred language code or null if not set
 */
export const getChatbotLanguage = () => {
  try {
    return localStorage.getItem(CHATBOT_LANGUAGE_KEY);
  } catch (error) {
    console.error('Error getting chatbot language:', error);
    return null;
  }
};

/**
 * Set preferred chatbot language
 * @param {string} language - Language code to set as preferred
 */
export const setChatbotLanguage = (language) => {
  try {
    localStorage.setItem(CHATBOT_LANGUAGE_KEY, language);
  } catch (error) {
    console.error('Error setting chatbot language:', error);
  }
};

/**
 * Clear all translation caches
 */
export const clearAllTranslationCaches = () => {
  try {
    const keysToRemove = [];
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key && (
        key.startsWith(DOC_TRANSLATION_PREFIX) ||
        key.startsWith(ORIGINAL_CONTENT_PREFIX) ||
        key === PREFERRED_DOC_LANGUAGE_KEY ||
        key === CHATBOT_MESSAGES_KEY ||
        key === CHATBOT_LANGUAGE_KEY
      )) {
        keysToRemove.push(key);
      }
    }

    keysToRemove.forEach(key => localStorage.removeItem(key));
  } catch (error) {
    console.error('Error clearing all translation caches:', error);
  }
};

/**
 * Get cache statistics
 * @returns {object} Object with cache statistics
 */
export const getCacheStats = () => {
  try {
    const stats = {
      total: 0,
      translationCaches: 0,
      originalContents: 0,
      other: 0
    };

    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key) {
        stats.total++;
        if (key.startsWith(DOC_TRANSLATION_PREFIX)) {
          stats.translationCaches++;
        } else if (key.startsWith(ORIGINAL_CONTENT_PREFIX)) {
          stats.originalContents++;
        } else {
          stats.other++;
        }
      }
    }

    return stats;
  } catch (error) {
    console.error('Error getting cache stats:', error);
    return { total: 0, translationCaches: 0, originalContents: 0, other: 0 };
  }
};