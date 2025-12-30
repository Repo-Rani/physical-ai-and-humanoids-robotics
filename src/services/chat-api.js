class ChatAPI {
  constructor() {
    const baseURL =
      (typeof process !== "undefined" &&
        process.env?.NEXT_PUBLIC_RAG_API_URL) ||
      "http://localhost:8000"; // default URL agar env na mile

    this.baseURL = baseURL;
  }

  async sendMessage(message, language, context = null, sessionId = null) {
    const startTime = Date.now();
    try {
      const response = await fetch(`${this.baseURL}/api/v1/chat`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          message,
          language,
          context: context,
          session_id:
            sessionId || `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        }),
      });

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      return await response.json();
    } catch (error) {
      console.error(error);
      throw error;
    }
  }

  async getSupportedLanguages() {
    const response = await fetch(`${this.baseURL}/api/v1/supported-languages`);
    if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
    return await response.json();
  }
}

export default new ChatAPI();
