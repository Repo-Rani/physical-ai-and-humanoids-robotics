// src/utils/language-preferences.js
// Utility functions for handling persistent language preferences

const LANGUAGE_PREFERENCES_KEY = 'language_preferences';

// Default language preferences structure
const DEFAULT_PREFERENCES = {
  documentation: 'en',
  chatbot: 'en',
  version: '1.0.0'
};

/**
 * Get stored language preferences
 * @returns {Object} Language preferences object
 */
export const getLanguagePreferences = () => {
  try {
    const stored = localStorage.getItem(LANGUAGE_PREFERENCES_KEY);
    if (stored) {
      const parsed = JSON.parse(stored);
      // Ensure the stored preferences have the correct structure
      return {
        ...DEFAULT_PREFERENCES,
        ...parsed
      };
    }
  } catch (error) {
    console.error('Error parsing language preferences:', error);
  }

  return { ...DEFAULT_PREFERENCES };
};

/**
 * Save language preferences
 * @param {Object} preferences - Language preferences object
 */
export const saveLanguagePreferences = (preferences) => {
  try {
    const updatedPreferences = {
      ...DEFAULT_PREFERENCES,
      ...getLanguagePreferences(),
      ...preferences,
      version: DEFAULT_PREFERENCES.version
    };

    localStorage.setItem(LANGUAGE_PREFERENCES_KEY, JSON.stringify(updatedPreferences));
    return true;
  } catch (error) {
    console.error('Error saving language preferences:', error);
    return false;
  }
};

/**
 * Get the preferred language for a specific context
 * @param {string} context - Context (e.g., 'documentation', 'chatbot')
 * @returns {string} Preferred language code
 */
export const getPreferredLanguage = (context) => {
  const preferences = getLanguagePreferences();
  return preferences[context] || preferences.documentation || 'en';
};

/**
 * Set the preferred language for a specific context
 * @param {string} context - Context (e.g., 'documentation', 'chatbot')
 * @param {string} language - Language code
 * @returns {boolean} Success status
 */
export const setPreferredLanguage = (context, language) => {
  const currentPreferences = getLanguagePreferences();
  const newPreferences = {
    ...currentPreferences,
    [context]: language
  };

  return saveLanguagePreferences(newPreferences);
};

/**
 * Initialize language preferences with default values if not present
 */
export const initializeLanguagePreferences = () => {
  const currentPreferences = getLanguagePreferences();

  // If no preferences exist, save the defaults
  if (Object.keys(currentPreferences).length === 0) {
    saveLanguagePreferences(DEFAULT_PREFERENCES);
  }
};

/**
 * Get browser's preferred language
 * @returns {string} Browser language code or default 'en'
 */
export const getBrowserLanguage = () => {
  if (typeof navigator !== 'undefined') {
    const browserLang = navigator.language.split('-')[0]; // Get language without region
    return browserLang || 'en';
  }
  return 'en';
};

/**
 * Get fallback language when no preference exists
 * @returns {string} Fallback language code
 */
export const getFallbackLanguage = () => {
  // Try to get from URL, then browser, then default to 'en'
  if (typeof window !== 'undefined') {
    const pathParts = window.location.pathname.split('/');
    const localeFromUrl = pathParts[1] || '';
    const validLocales = ['en', 'ur', 'ar', 'es', 'fr', 'de', 'zh', 'ja'];

    if (validLocales.includes(localeFromUrl)) {
      return localeFromUrl;
    }
  }

  return getBrowserLanguage();
};