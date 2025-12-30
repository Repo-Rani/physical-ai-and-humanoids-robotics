// src/services/translation-api.js
// Translation API client for multilingual Docusaurus + RAG Chatbot

class TranslationAPI {
  constructor(baseURL = process.env.NEXT_PUBLIC_RAG_API_URL || 'http://localhost:8000') {
    this.baseURL = baseURL;
  }

  async translate(text, targetLanguage, sourceLanguage = null, context = null) {
    const startTime = Date.now();
    try {
      console.log(`Starting translation: ${sourceLanguage || 'auto'} -> ${targetLanguage}, text length: ${text.length}`);

      const response = await fetch(`${this.baseURL}/api/v1/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': process.env.OPENROUTER_API_KEY || '' // In a real implementation, this would be handled securely
        },
        body: JSON.stringify({
          text,
          target_language: targetLanguage,
          source_language: sourceLanguage,
          context: context
        })
      });

      const duration = Date.now() - startTime;
      console.log(`Translation API call completed in ${duration}ms`);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        const errorMessage = errorData.error || `HTTP error! status: ${response.status}`;
        console.error('Translation API error:', errorMessage);
        throw new Error(errorMessage);
      }

      const result = await response.json();
      console.log(`Translation completed successfully in ${Date.now() - startTime}ms`);
      return result;
    } catch (error) {
      const duration = Date.now() - startTime;
      console.error(`Translation failed after ${duration}ms:`, error);
      throw error;
    }
  }

  async getTranslationStats() {
    try {
      const response = await fetch(`${this.baseURL}/api/v1/stats`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      return await response.json();
    } catch (error) {
      console.error('Translation stats error:', error);
      throw error;
    }
  }

  async translateConversation(messages, targetLanguage, sourceLanguage = 'auto') {
    const startTime = Date.now();
    try {
      console.log(`Starting conversation translation: ${messages.length} messages, ${sourceLanguage} -> ${targetLanguage}`);

      const response = await fetch(`${this.baseURL}/api/v1/translate-conversation`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          messages,
          target_language: targetLanguage,
          source_language: sourceLanguage
        })
      });

      const duration = Date.now() - startTime;
      console.log(`Conversation translation API call completed in ${duration}ms`);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        const errorMessage = errorData.error || `HTTP error! status: ${response.status}`;
        console.error('Conversation translation API error:', errorMessage);
        throw new Error(errorMessage);
      }

      const result = await response.json();
      console.log(`Conversation translation completed successfully in ${Date.now() - startTime}ms`);
      return result;
    } catch (error) {
      const duration = Date.now() - startTime;
      console.error(`Conversation translation failed after ${duration}ms:`, error);
      throw error;
    }
  }
}

export default new TranslationAPI();