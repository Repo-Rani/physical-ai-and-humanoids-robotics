# Quickstart Guide: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Prerequisites
- Python 3.11+
- Node.js 16+ (for Docusaurus)
- API keys for Cohere, Qdrant, and OpenRouter
- Git

## Backend Setup

### 1. Clone and Navigate
```bash
cd backend
```

### 2. Initialize UV Project
```bash
uv init
```

### 3. Install Dependencies
```bash
uv sync
```

### 4. Configure Environment
```bash
cp .env.example .env
# Edit .env with your API keys
```

### 5. Run Documentation Scraper (First Time Only)
```bash
uv run python scrapper.py
```

### 6. Start Backend Server
```bash
uv run python main.py
```

## Frontend Integration

### 1. Create Chatbot Component
- Create `src/components/chatbot/HumanoidChatbot.jsx`
- Create `src/components/chatbot/HumanoidChatbot.css`

### 2. Integrate with Docusaurus
Create `src/theme/Root.jsx`:
```jsx
import React from 'react';
import HumanoidChatbot from '@site/src/components/chatbot/HumanoidChatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <HumanoidChatbot />
    </>
  );
}
```

### 3. Start Docusaurus
```bash
npm start
```

## Testing
- Backend API: `http://localhost:8000/docs`
- Health check: `http://localhost:8000/health`
- Chat endpoint: `http://localhost:8000/chat`

## API Keys Required
1. **Cohere API**: For embeddings
2. **Qdrant Cloud**: For vector storage
3. **OpenRouter API**: For LLM responses

## Troubleshooting
- Check `.env` file has all required keys
- Verify Qdrant cluster is active
- Confirm backend is running on port 8000
- Check CORS settings if frontend can't connect